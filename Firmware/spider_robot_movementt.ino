/*
 * Agricultural Spider Robot — Quadruped Movement Subsystem
 * Target : ESP32 Dev Module | Library: PololuMaestro
 *
 * Diagonal trot gait, 4 legs x 2 servos (hip + knee) = 8 channels total.
 * Commands are read from the shared `currentCommand` string defined in the
 * master combined sketch. Call movementSetup() in setup() and
 * movementUpdate() in loop().
 *
 * Maestro channel map:
 *   0 FL-Hip  1 FL-Knee  2 FR-Hip  3 FR-Knee
 *   4 RL-Hip  5 RL-Knee  6 RR-Hip  7 RR-Knee
 *
 * ESP32 Serial2: TX=GPIO17 → Maestro RX,  RX=GPIO16 → Maestro TX
 */

#include <PololuMaestro.h>

// ── Maestro driver ────────────────────────────────────────────────────────────
#define MAESTRO_SERIAL Serial2
MiniMaestro g_maestro(MAESTRO_SERIAL);
const uint16_t SERVO_ACCELERATION = 0; // 0 = no ramp (instant speed change)

// ── Position: foot coordinate in mm from body centre ─────────────────────────
// x = forward(+) / backward(-)   y = left(+) / right(-)   z = up(+) / down(-)
struct Position {
    int16_t x, y, z;
};

// ── KeyFrame: desired foot position for all four legs at one instant ──────────
struct KeyFrame {
    Position FrontLeft;
    Position FrontRight;
    Position RearLeft;
    Position RearRight;
};

// ── Geometry constants (mm) — adjust to match physical robot ─────────────────
const int16_t MIN_HOME_HEIGHT  = -60;   // Shallowest standing z
const int16_t MAX_HOME_HEIGHT  = -140;  // Deepest standing z
const int16_t FOOT_SLEEP_HEIGHT =  0;   // z when legs folded flat
const int16_t FOOT_FB_DISTANCE =  100;  // Hip joint x-offset from body centre
const int16_t FOOT_WIDTH       =   90;  // Hip joint y-offset from body centre
const int16_t FOOT_WALK_X      =   40;  // Foot fore-aft travel per step (mm)
const int16_t FOOT_LIFT        =   30;  // How high a foot rises during a step
const int16_t THIGH_LENGTH     =   84;  // Hip horn → knee horn (mm)
const int16_t SHIN_LENGTH      =  127;  // Knee horn → toe tip  (mm)
const int8_t  INTERPOLATIONS   =    5;  // Keyframe sub-steps (higher = smoother)

// ── QuadServo: one servo on the Maestro ──────────────────────────────────────
class QuadServo {
public:
    static const uint16_t PULSE_MIN    = 576;
    static const uint16_t PULSE_MAX    = 2464;
    static const uint16_t PULSE_CENTER = PULSE_MIN + (PULSE_MAX - PULSE_MIN) / 2;
    static constexpr float ANGLE_MIN   = -90.0f;
    static constexpr float ANGLE_MAX   =  90.0f;

    void initialize(const char* label, uint8_t channel, bool reversed) {
        m_label = label; m_channel = channel; m_isReversed = reversed;
        m_speed = 0; m_pulseWidth = PULSE_CENTER;
    }

    // Convert degrees to pulse width; flip sign for reversed (right-side) servos
    void setAngle(float deg) {
        deg = constrain(deg, ANGLE_MIN, ANGLE_MAX);
        m_angle = deg;
        if (m_isReversed) deg = -deg;
        m_pulseWidth = (uint16_t)map((int32_t)(deg * 10),
                                     (int32_t)(ANGLE_MIN * 10),
                                     (int32_t)(ANGLE_MAX * 10),
                                     PULSE_MIN, PULSE_MAX);
    }

    void setSpeed(uint8_t spd) { m_speed = constrain(spd, 0, 100); }

    uint8_t     channel()    const { return m_channel; }
    uint16_t    pulseWidth() const { return m_pulseWidth; }
    uint8_t     speed()      const { return m_speed; }
    float       angle()      const { return m_angle; }
    const char* label()      const { return m_label; }

private:
    const char* m_label      = "";
    uint8_t     m_channel    = 0;
    bool        m_isReversed = false;
    uint8_t     m_speed      = 0;
    uint16_t    m_pulseWidth = PULSE_CENTER;
    float       m_angle      = 0.0f;
};

// ── QuadLegId ─────────────────────────────────────────────────────────────────
enum class QuadLegId : uint8_t { FRONT_LEFT, FRONT_RIGHT, REAR_LEFT, REAR_RIGHT };

// ── QuadLeg: 2-DOF leg with hip + knee servos ────────────────────────────────
class QuadLeg {
public:
    QuadLeg(QuadLegId id, int16_t anchorX, int16_t anchorY)
        : m_id(id), m_anchorX(anchorX), m_anchorY(anchorY)
    {
        // Pre-compute the hip anchor bearing to convert world angles to leg-local angles
        m_anchorAngleDeg = atan2f((float)anchorY, (float)anchorX) * 180.0f / (float)M_PI;

        // Assign Maestro channels; right-side hips are reversed so "forward" is consistent
        switch (id) {
            case QuadLegId::FRONT_LEFT:
                m_hipServo.initialize("FL_Hip",  0, false);
                m_kneeServo.initialize("FL_Knee",1, false); break;
            case QuadLegId::FRONT_RIGHT:
                m_hipServo.initialize("FR_Hip",  2, true);
                m_kneeServo.initialize("FR_Knee",3, false); break;
            case QuadLegId::REAR_LEFT:
                m_hipServo.initialize("RL_Hip",  4, false);
                m_kneeServo.initialize("RL_Knee",5, false); break;
            case QuadLegId::REAR_RIGHT:
                m_hipServo.initialize("RR_Hip",  6, true);
                m_kneeServo.initialize("RR_Knee",7, false); break;
        }
    }

    // 2-DOF IK: convert foot Position (body-centre coords) → hip + knee angles
    bool setFootPosition(Position foot, uint8_t speed) {
        // Foot offset relative to this leg's hip anchor
        float dx = (float)(foot.x - m_anchorX);
        float dy = (float)(foot.y - m_anchorY);
        float dz = (float) foot.z;

        // Hip angle: bearing of foot in XY plane minus the anchor bearing
        float hipDeg = atan2f(dy, dx) * 180.0f / (float)M_PI - m_anchorAngleDeg;

        // Horizontal reach from hip to foot (used for knee calculation)
        float hDist = sqrtf(dx * dx + dy * dy);

        // Knee angle via law of cosines on the 2-link (thigh + shin) vertical chain
        float l = sqrtf(hDist * hDist + dz * dz); // straight-line hip-to-toe distance
        float cosKnee = (sq(THIGH_LENGTH) + sq(SHIN_LENGTH) - sq(l))
                        / (2.0f * THIGH_LENGTH * SHIN_LENGTH);
        cosKnee = constrain(cosKnee, -1.0f, 1.0f); // guard against NaN from rounding
        float kneeDeg = acosf(cosKnee) * 180.0f / (float)M_PI - 90.0f;
        // Add depression angle so the shin tilts to actually reach the target z
        kneeDeg += atan2f(-dz, hDist) * 180.0f / (float)M_PI;

        if (isnan(hipDeg) || isnan(kneeDeg)) {
            Serial.printf("[IK] %s: unreachable (%d,%d,%d)\n",
                          m_hipServo.label(), foot.x, foot.y, foot.z);
            return false;
        }

        m_hipServo.setAngle(hipDeg);   m_hipServo.setSpeed(speed);
        m_kneeServo.setAngle(kneeDeg); m_kneeServo.setSpeed(speed);
        m_lastFoot = foot;
        return true;
    }

    QuadServo& hipServo()  { return m_hipServo; }
    QuadServo& kneeServo() { return m_kneeServo; }
    Position   lastFoot()  { return m_lastFoot; }

private:
    QuadLegId  m_id;
    int16_t    m_anchorX = 0, m_anchorY = 0;
    float      m_anchorAngleDeg = 0;
    QuadServo  m_hipServo, m_kneeServo;
    Position   m_lastFoot = {0, 0, 0};
};

// ── Pose helpers ──────────────────────────────────────────────────────────────

// Map height setting 0-9 to a z coordinate between MIN and MAX home height
static inline int16_t footHomeZ(uint8_t h) {
    return (int16_t)map(h, 0, 9, MIN_HOME_HEIGHT, MAX_HOME_HEIGHT);
}

// Sleep: all legs folded flat (z = 0)
KeyFrame posesSleep() {
    return KeyFrame{
        { FOOT_FB_DISTANCE,  FOOT_WIDTH, FOOT_SLEEP_HEIGHT},
        { FOOT_FB_DISTANCE, -FOOT_WIDTH, FOOT_SLEEP_HEIGHT},
        {-FOOT_FB_DISTANCE,  FOOT_WIDTH, FOOT_SLEEP_HEIGHT},
        {-FOOT_FB_DISTANCE, -FOOT_WIDTH, FOOT_SLEEP_HEIGHT}
    };
}

// Home: neutral standing at the selected height
KeyFrame posesHome(uint8_t h) {
    int16_t z = footHomeZ(h);
    return KeyFrame{
        { FOOT_FB_DISTANCE,  FOOT_WIDTH, z},
        { FOOT_FB_DISTANCE, -FOOT_WIDTH, z},
        {-FOOT_FB_DISTANCE,  FOOT_WIDTH, z},
        {-FOOT_FB_DISTANCE, -FOOT_WIDTH, z}
    };
}

// Diagonal trot walk cycle (W0-W3)
// W0: lift pair A (FL + RR)
KeyFrame posesW0(uint8_t h) {
    int16_t z = footHomeZ(h), zu = z + FOOT_LIFT;
    return KeyFrame{
        { FOOT_FB_DISTANCE,  FOOT_WIDTH, zu}, // FL lifted
        { FOOT_FB_DISTANCE, -FOOT_WIDTH, z },
        {-FOOT_FB_DISTANCE,  FOOT_WIDTH, z },
        {-FOOT_FB_DISTANCE, -FOOT_WIDTH, zu}  // RR lifted
    };
}
// W1: swing pair A forward, pair B backward (all feet down)
KeyFrame posesW1(uint8_t h) {
    int16_t z = footHomeZ(h);
    return KeyFrame{
        { FOOT_FB_DISTANCE + FOOT_WALK_X,  FOOT_WIDTH, z}, // FL fwd
        { FOOT_FB_DISTANCE - FOOT_WALK_X, -FOOT_WIDTH, z}, // FR bwd
        {-FOOT_FB_DISTANCE - FOOT_WALK_X,  FOOT_WIDTH, z}, // RL bwd
        {-FOOT_FB_DISTANCE + FOOT_WALK_X, -FOOT_WIDTH, z}  // RR fwd
    };
}
// W2: lift pair B (FR + RL)
KeyFrame posesW2(uint8_t h) {
    int16_t z = footHomeZ(h), zu = z + FOOT_LIFT;
    return KeyFrame{
        { FOOT_FB_DISTANCE,  FOOT_WIDTH, z },
        { FOOT_FB_DISTANCE, -FOOT_WIDTH, zu}, // FR lifted
        {-FOOT_FB_DISTANCE,  FOOT_WIDTH, zu}, // RL lifted
        {-FOOT_FB_DISTANCE, -FOOT_WIDTH, z }
    };
}
// W3: swing pair B forward, pair A backward (all feet down)
KeyFrame posesW3(uint8_t h) {
    int16_t z = footHomeZ(h);
    return KeyFrame{
        { FOOT_FB_DISTANCE - FOOT_WALK_X,  FOOT_WIDTH, z}, // FL bwd
        { FOOT_FB_DISTANCE + FOOT_WALK_X, -FOOT_WIDTH, z}, // FR fwd
        {-FOOT_FB_DISTANCE + FOOT_WALK_X,  FOOT_WIDTH, z}, // RL fwd
        {-FOOT_FB_DISTANCE - FOOT_WALK_X, -FOOT_WIDTH, z}  // RR bwd
    };
}

// Rotate Left: R0/R2 reuse the same lift frames as W0/W2
// R1/R3 swing feet in arcs around the body instead of straight lines
KeyFrame posesR0(uint8_t h) { return posesW0(h); }
KeyFrame posesR1(uint8_t h) {
    int16_t z = footHomeZ(h);
    return KeyFrame{
        { 99,  FOOT_WIDTH + 30, z},          // pair A rotated
        { FOOT_FB_DISTANCE, -FOOT_WIDTH, z},
        {-FOOT_FB_DISTANCE,  FOOT_WIDTH, z},
        {-99, -FOOT_WIDTH - 30, z}           // pair A rotated
    };
}
KeyFrame posesR2(uint8_t h) { return posesW2(h); }
KeyFrame posesR3(uint8_t h) {
    int16_t z = footHomeZ(h);
    return KeyFrame{
        { FOOT_FB_DISTANCE,  FOOT_WIDTH, z},
        { 99, -FOOT_WIDTH - 30, z},          // pair B rotated
        {-99,  FOOT_WIDTH + 30, z},          // pair B rotated
        {-FOOT_FB_DISTANCE, -FOOT_WIDTH, z}
    };
}

// Rotate Right: mirror of left (negate the arc offsets)
KeyFrame posesRR1(uint8_t h) {
    int16_t z = footHomeZ(h);
    return KeyFrame{
        { 99,  FOOT_WIDTH - 30, z},
        { FOOT_FB_DISTANCE, -FOOT_WIDTH, z},
        {-FOOT_FB_DISTANCE,  FOOT_WIDTH, z},
        {-99, -FOOT_WIDTH + 30, z}
    };
}
KeyFrame posesRR3(uint8_t h) {
    int16_t z = footHomeZ(h);
    return KeyFrame{
        { FOOT_FB_DISTANCE,  FOOT_WIDTH, z},
        { 99, -FOOT_WIDTH + 30, z},
        {-99,  FOOT_WIDTH - 30, z},
        {-FOOT_FB_DISTANCE, -FOOT_WIDTH, z}
    };
}

// Wave / HighFive action — 4 sub-frames raise and swing the front-left leg
KeyFrame posesA1_1(uint8_t h) {
    int16_t z = footHomeZ(h);
    return KeyFrame{{ FOOT_FB_DISTANCE, FOOT_WIDTH, z+40},{ FOOT_FB_DISTANCE,-FOOT_WIDTH,z},{-FOOT_FB_DISTANCE,FOOT_WIDTH,z},{-FOOT_FB_DISTANCE,-FOOT_WIDTH,z}};
}
KeyFrame posesA1_2(uint8_t h) {
    int16_t z = footHomeZ(h);
    return KeyFrame{{ FOOT_FB_DISTANCE+40, FOOT_WIDTH-50, z+60},{ FOOT_FB_DISTANCE,-FOOT_WIDTH,z},{-FOOT_FB_DISTANCE,FOOT_WIDTH,z},{-FOOT_FB_DISTANCE,-FOOT_WIDTH,z}};
}
KeyFrame posesA1_3(uint8_t h) {
    int16_t z = footHomeZ(h);
    return KeyFrame{{ FOOT_FB_DISTANCE+40, FOOT_WIDTH-50, z+30},{ FOOT_FB_DISTANCE,-FOOT_WIDTH,z},{-FOOT_FB_DISTANCE,FOOT_WIDTH,z},{-FOOT_FB_DISTANCE,-FOOT_WIDTH,z}};
}
KeyFrame posesA1_4(uint8_t h) {
    int16_t z = footHomeZ(h);
    return KeyFrame{{ FOOT_FB_DISTANCE, FOOT_WIDTH, z},{ FOOT_FB_DISTANCE,-FOOT_WIDTH,z},{-FOOT_FB_DISTANCE,FOOT_WIDTH,z},{-FOOT_FB_DISTANCE,-FOOT_WIDTH,z}};
}

// ── Motion state machine ──────────────────────────────────────────────────────
enum class MovementState : uint8_t {
    SLEEP, HOME, WALK_FORWARD, WALK_BACKWARD, ROTATE_LEFT, ROTATE_RIGHT, ACTION
};

MovementState g_motionState        = MovementState::SLEEP;
MovementState g_motionDesiredState = MovementState::SLEEP;
uint8_t g_walkCycleCount    = 0;
uint8_t g_robotSpeed        = 20;  // 0-100, mapped from MQTT 0-9
uint8_t g_heightSetting     = 5;   // 0=shallowest, 9=deepest
uint8_t g_desiredHeight     = 5;
uint8_t g_desiredAction     = 0;   // 1 = wave
int8_t  g_interpolationCount = -1; // -1 = idle

KeyFrame g_currentKeyFrame = posesSleep();
KeyFrame g_targetKeyFrame  = posesSleep();

// ── Leg instances (anchor positions in mm from body centre) ───────────────────
QuadLeg g_legFL(QuadLegId::FRONT_LEFT,   FOOT_FB_DISTANCE,  FOOT_WIDTH);
QuadLeg g_legFR(QuadLegId::FRONT_RIGHT,  FOOT_FB_DISTANCE, -FOOT_WIDTH);
QuadLeg g_legRL(QuadLegId::REAR_LEFT,   -FOOT_FB_DISTANCE,  FOOT_WIDTH);
QuadLeg g_legRR(QuadLegId::REAR_RIGHT,  -FOOT_FB_DISTANCE, -FOOT_WIDTH);

// ── Maestro commit helpers ────────────────────────────────────────────────────

// Send one servo's speed and position to the Maestro (setTarget expects quarter-µs)
void CommitServo(QuadServo& srv) {
    g_maestro.setSpeed(srv.channel(), srv.speed());
    g_maestro.setTarget(srv.channel(), srv.pulseWidth() * 4);
}

void CommitLeg(QuadLeg& leg) {
    CommitServo(leg.hipServo());
    CommitServo(leg.kneeServo());
}

void CommitAllLegs() {
    CommitLeg(g_legFL); CommitLeg(g_legFR);
    CommitLeg(g_legRL); CommitLeg(g_legRR);
}

// Set all channels to target=0 (Maestro releases hold) when entering sleep
void StopServos() {
    for (uint8_t i = 0; i < 8; i++) { g_maestro.setTarget(i, 0); delay(20); }
}

void CommitAcceleration() {
    for (uint8_t i = 0; i < 8; i++) g_maestro.setAcceleration(i, SERVO_ACCELERATION);
}

// Returns true while any servo channel is still moving toward its target
bool IsServosMoving() { return g_maestro.getMovingState() > 0; }

// ── Interpolation ─────────────────────────────────────────────────────────────

static inline int16_t lerpI(int16_t a, int16_t b, float t) {
    return (int16_t)(a + (b - a) * t);
}

Position InterpolatePosition(Position a, Position b, float t) {
    return Position{lerpI(a.x,b.x,t), lerpI(a.y,b.y,t), lerpI(a.z,b.z,t)};
}

// Compute and push one interpolation step across all legs
void Interpolate() {
    if (g_interpolationCount < 0) return;
    float t = ((float)INTERPOLATIONS - (float)g_interpolationCount) / (float)INTERPOLATIONS;
    g_legFL.setFootPosition(InterpolatePosition(g_currentKeyFrame.FrontLeft,  g_targetKeyFrame.FrontLeft,  t), g_robotSpeed);
    g_legFR.setFootPosition(InterpolatePosition(g_currentKeyFrame.FrontRight, g_targetKeyFrame.FrontRight, t), g_robotSpeed);
    g_legRL.setFootPosition(InterpolatePosition(g_currentKeyFrame.RearLeft,   g_targetKeyFrame.RearLeft,   t), g_robotSpeed);
    g_legRR.setFootPosition(InterpolatePosition(g_currentKeyFrame.RearRight,  g_targetKeyFrame.RearRight,  t), g_robotSpeed);
}

// Queue a new target KeyFrame and start interpolating toward it
void MoveKeyFrame(KeyFrame next) {
    g_targetKeyFrame     = next;
    g_interpolationCount = INTERPOLATIONS;
}

// ── Speed helper ──────────────────────────────────────────────────────────────
// Maps MQTT speed value (0-9) to servo speed (20-100); floor of 20 avoids stall
void SetMovementSpeed(uint8_t mqttSpeed) {
    g_robotSpeed = (uint8_t)map(mqttSpeed, 0, 9, 20, 100);
}

// ── Gait state machine ────────────────────────────────────────────────────────
// Called when the interpolation engine is idle and all servos have settled.
// Selects the next KeyFrame based on current motion state and walk cycle phase.
// Walking states only transition at cycle 1 or 3 (both diagonal pairs grounded).
void DetermineNextMove() {
    bool changeRequested = (g_motionDesiredState != g_motionState)
                        || (g_desiredHeight != g_heightSetting);

    if (changeRequested) {
        g_heightSetting = g_desiredHeight;

        // Non-gait states transition immediately
        if (g_motionState == MovementState::SLEEP  ||
            g_motionState == MovementState::HOME   ||
            g_motionState == MovementState::ACTION) {
            g_motionState    = g_motionDesiredState;
            g_walkCycleCount = 0;
        }

        // Gait states: only switch when both diagonal pairs are on the ground
        if (g_motionState == MovementState::WALK_FORWARD  ||
            g_motionState == MovementState::WALK_BACKWARD ||
            g_motionState == MovementState::ROTATE_LEFT   ||
            g_motionState == MovementState::ROTATE_RIGHT) {
            if (g_walkCycleCount == 1 || g_walkCycleCount == 3) {
                g_motionState    = g_motionDesiredState;
                g_walkCycleCount = 0;
            }
        }

        if (g_motionState == MovementState::HOME)  MoveKeyFrame(posesHome(g_heightSetting));
        if (g_motionState == MovementState::SLEEP) { MoveKeyFrame(posesSleep()); StopServos(); }
    }

    if (g_motionState == MovementState::WALK_FORWARD) {
        switch (g_walkCycleCount) {
            case 0: MoveKeyFrame(posesW0(g_heightSetting)); break; // lift pair A
            case 1: MoveKeyFrame(posesW1(g_heightSetting)); break; // swing pair A fwd
            case 2: MoveKeyFrame(posesW2(g_heightSetting)); break; // lift pair B
            case 3: MoveKeyFrame(posesW3(g_heightSetting)); break; // swing pair B fwd
        }
        g_walkCycleCount = (g_walkCycleCount + 1) % 4;
    }

    if (g_motionState == MovementState::WALK_BACKWARD) {
        switch (g_walkCycleCount) {
            case 0: MoveKeyFrame(posesW0(g_heightSetting)); break;
            case 1: MoveKeyFrame(posesW3(g_heightSetting)); break; // reversed swing
            case 2: MoveKeyFrame(posesW2(g_heightSetting)); break;
            case 3: MoveKeyFrame(posesW1(g_heightSetting)); break; // reversed swing
        }
        g_walkCycleCount = (g_walkCycleCount + 1) % 4;
    }

    if (g_motionState == MovementState::ROTATE_LEFT) {
        switch (g_walkCycleCount) {
            case 0: MoveKeyFrame(posesR0(g_heightSetting));  break;
            case 1: MoveKeyFrame(posesR1(g_heightSetting));  break;
            case 2: MoveKeyFrame(posesR2(g_heightSetting));  break;
            case 3: MoveKeyFrame(posesR3(g_heightSetting));  break;
        }
        g_walkCycleCount = (g_walkCycleCount + 1) % 4;
    }

    if (g_motionState == MovementState::ROTATE_RIGHT) {
        switch (g_walkCycleCount) {
            case 0: MoveKeyFrame(posesR0(g_heightSetting));  break;
            case 1: MoveKeyFrame(posesRR1(g_heightSetting)); break;
            case 2: MoveKeyFrame(posesR2(g_heightSetting));  break;
            case 3: MoveKeyFrame(posesRR3(g_heightSetting)); break;
        }
        g_walkCycleCount = (g_walkCycleCount + 1) % 4;
    }

    if (g_motionState == MovementState::ACTION && g_desiredAction == 1) {
        switch (g_walkCycleCount) {
            case 0: MoveKeyFrame(posesA1_1(g_heightSetting)); break;
            case 1: MoveKeyFrame(posesA1_2(g_heightSetting)); break;
            case 2: MoveKeyFrame(posesA1_3(g_heightSetting)); break;
            case 3: MoveKeyFrame(posesA1_4(g_heightSetting)); break;
        }
        if (++g_walkCycleCount > 3) {
            g_walkCycleCount     = 0;
            g_motionDesiredState = MovementState::HOME; // auto-return to standing
        }
    }
}

// ── Public API — call from master sketch ──────────────────────────────────────

// Call once in setup()
void movementSetup() {
    MAESTRO_SERIAL.begin(115200);
    delay(500);
    CommitAcceleration();
    MoveKeyFrame(posesSleep());
    CommitAllLegs();
    logTag("MOVE", "Movement subsystem ready");
}

// Call every loop() — translates currentCommand → state, drives interpolation
void movementUpdate() {
    // Map the shared MQTT command string to the desired motion state
    if      (currentCommand == "forward")                    { g_motionDesiredState = MovementState::WALK_FORWARD;  SetMovementSpeed(5); }
    else if (currentCommand == "backward")                   { g_motionDesiredState = MovementState::WALK_BACKWARD; SetMovementSpeed(5); }
    else if (currentCommand == "left")                       { g_motionDesiredState = MovementState::ROTATE_LEFT;   SetMovementSpeed(5); }
    else if (currentCommand == "right")                      { g_motionDesiredState = MovementState::ROTATE_RIGHT;  SetMovementSpeed(5); }
    else if (currentCommand == "stop")                       { g_motionDesiredState = MovementState::HOME;          SetMovementSpeed(0); }
    else if (currentCommand == "sleep")                      { g_motionDesiredState = MovementState::SLEEP;         SetMovementSpeed(0); }
    else if (currentCommand == "stand")                      { g_motionDesiredState = MovementState::HOME;          SetMovementSpeed(5); }
    else if (currentCommand == "wave" ||
             currentCommand == "highfive")                   { g_motionDesiredState = MovementState::ACTION; g_desiredAction = 1; SetMovementSpeed(7); }

    // Run one interpolation step if steps remain and the Maestro has settled
    if (g_interpolationCount >= 0 && !IsServosMoving()) {
        Interpolate();
        CommitAllLegs();
        if (g_interpolationCount == 0) g_currentKeyFrame = g_targetKeyFrame;
        g_interpolationCount--;
    }

    // Advance the gait once the interpolation engine is fully idle
    if (g_interpolationCount < 0 && !IsServosMoving()) {
        DetermineNextMove();
    }
}
