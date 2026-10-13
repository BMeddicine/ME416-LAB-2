// RASTICrane servo controller for the ESP32-S3 Super Mini (HW-747 V0.02).
//
// Serial commands (115200 baud, native USB):
//   <joint> <angle>      move one joint, e.g. "1 150"
//   goto <a> <b> <c>     move all joints, arriving at the same time
//   home                 move to the home pose
//   stop                 freeze where the arm is now
//   speed <deg/s>        set max speed          accel <deg/s^2>  set acceleration
//   here <a> <b> <c>     tell the controller where the arm is, without moving it
//   relax / hold         stop / restart the servo pulses
//   status               show positions and settings
//   test                 sweep each joint through its range and back
//
// Motion: every move follows a trapezoidal velocity profile (accelerate, cruise at
// maxSpeed_degps, decelerate). The position is a float and is updated once per
// PWM frame, so the servo sees a continuous ramp instead of whole-degree jumps.
//
// Start-up: hobby servos report no position. The controller assumes the arm is where
// it was last commanded (saved in flash), or at the home pose if nothing was saved,
// holds that pose briefly, then glides to the home pose at the normal smooth speed.

#include <Arduino.h>
#include <Preferences.h>
#include <math.h>
#include "driver/ledc.h"

// ============================================
// JOINT CONFIGURATION (per arm)
// ============================================
// Angles are servo angles in degrees, 0 at minPulse_us and servoTravel_deg at maxPulse_us.
// minAngle_deg / maxAngle_deg: allowed range; commands outside it are rejected.
// homeAngle_deg: the pose assumed at first boot, moved to after every boot, and used by "home".
struct JointConfig {
    const char* name;
    uint8_t pin;
    ledc_channel_t channel;
    float servoTravel_deg;
    uint16_t minPulse_us;
    uint16_t maxPulse_us;
    float minAngle_deg;
    float maxAngle_deg;
    float homeAngle_deg;
};

const JointConfig kJoints[] = {
    // name        pin  channel          travel  minUs  maxUs   min    max    home
    {"base 300",   10,  LEDC_CHANNEL_0,  300.0f, 500,   2500,   0.0f,  300.0f, 250.0f},
    {"180 #1",      9,  LEDC_CHANNEL_1,  180.0f, 500,   2500,   0.0f,  180.0f, 45.0f},
    {"180 #2",     11,  LEDC_CHANNEL_2,  180.0f, 500,   2500,   0.0f,  180.0f, 27.0f},
};
const uint8_t kNumJoints = sizeof(kJoints) / sizeof(kJoints[0]);

// ============================================
// MOTION TUNING (can also be changed at run time with "speed" / "accel")
// ============================================
float maxSpeed_degps = 30.0f;    // cruise speed of the slowest-finishing joint
float accel_degps2 = 60.0f;      // ramp up/down rate; lower = gentler starts and stops

// After boot, hold the assumed pose for kBootHold_ms, then move smoothly to the home pose.
const bool kHomeOnBoot = true;
const uint32_t kBootHold_ms = 500;

// ============================================
// HARDWARE
// ============================================
#define LED_PIN           21
#define LED_ON            LOW     // onboard LED is active low

#define LEDC_MODE         LEDC_LOW_SPEED_MODE
#define LEDC_TIMER        LEDC_TIMER_0
#define LEDC_RESOLUTION   LEDC_TIMER_14_BIT   // 14 bits at 50 Hz: 1.22 us per count (~0.18 deg)
const uint32_t kPwmFreq_hz = 50;
const uint32_t kPwmPeriod_us = 1000000 / kPwmFreq_hz;
const uint32_t kDutyMax = (1u << 14) - 1;

// One motion update per PWM frame; updating faster than the servo reads pulses does nothing.
const uint32_t kUpdatePeriod_us = kPwmPeriod_us;

// Positions are saved after motion stops and the arm has been still this long.
const uint32_t kSaveDelay_ms = 500;

// Flash key for the saved pose. Changed from "pose" when the base and 180 #2 pins were
// swapped, so a pose saved in the old joint order is never applied to the wrong servo.
const char* const kPoseKey = "pose2";

const uint8_t kInputBufferSize = 64;

// ============================================
// STATE
// ============================================
struct JointState {
    float position_deg;     // where the servo is being commanded right now
    float velocity_degps;   // signed
    float target_deg;
    float moveSpeed_degps;  // this move's speed/accel limits (scaled so joints finish together)
    float moveAccel_degps2;
    bool moving;
};

JointState joints[kNumJoints];
bool outputsEnabled = true;
bool poseDirty = false;       // position changed since last save
uint32_t lastMotion_ms = 0;
Preferences prefs;

// Queue of poses for "test", run one after the other.
const uint8_t kMaxQueuedPoses = 3 * kNumJoints + 1;
float poseQueue[kMaxQueuedPoses][kNumJoints];
uint8_t queueHead = 0, queueCount = 0;

// ============================================
// SERVO OUTPUT
// ============================================
static uint32_t angleToDuty(uint8_t jointIdx, float angle_deg) {
    const JointConfig& cfg = kJoints[jointIdx];
    float pulse_us = cfg.minPulse_us +
                     (angle_deg / cfg.servoTravel_deg) * (cfg.maxPulse_us - cfg.minPulse_us);
    return (uint32_t)lroundf(pulse_us * (float)(kDutyMax + 1) / (float)kPwmPeriod_us);
}

static void writeServo(uint8_t jointIdx) {
    uint32_t duty = outputsEnabled ? angleToDuty(jointIdx, joints[jointIdx].position_deg) : 0;
    ledc_set_duty(LEDC_MODE, kJoints[jointIdx].channel, duty);
    ledc_update_duty(LEDC_MODE, kJoints[jointIdx].channel);
}

static void setupPwm() {
    ledc_timer_config_t timerCfg = {};
    timerCfg.speed_mode = LEDC_MODE;
    timerCfg.duty_resolution = LEDC_RESOLUTION;
    timerCfg.timer_num = LEDC_TIMER;
    timerCfg.freq_hz = kPwmFreq_hz;
    timerCfg.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timerCfg);

    for (uint8_t i = 0; i < kNumJoints; i++) {
        ledc_channel_config_t channelCfg = {};
        channelCfg.gpio_num = kJoints[i].pin;
        channelCfg.speed_mode = LEDC_MODE;
        channelCfg.channel = kJoints[i].channel;
        channelCfg.timer_sel = LEDC_TIMER;
        // The very first pulse is already the assumed pose, so the servo is not yanked anywhere.
        channelCfg.duty = angleToDuty(i, joints[i].position_deg);
        channelCfg.hpoint = 0;
        ledc_channel_config(&channelCfg);
    }
}

// ============================================
// SAVED POSE
// ============================================
static void loadPose() {
    prefs.begin("arm", true);
    bool havePose = prefs.isKey(kPoseKey);
    float saved_deg[kNumJoints];
    if (havePose) {
        havePose = prefs.getBytes(kPoseKey, saved_deg, sizeof(saved_deg)) == sizeof(saved_deg);
    }
    prefs.end();

    for (uint8_t i = 0; i < kNumJoints; i++) {
        float angle_deg = havePose ? saved_deg[i] : kJoints[i].homeAngle_deg;
        angle_deg = constrain(angle_deg, kJoints[i].minAngle_deg, kJoints[i].maxAngle_deg);
        joints[i] = {angle_deg, 0.0f, angle_deg, maxSpeed_degps, accel_degps2, false};
    }
    Serial.println(havePose ? "Assuming last saved pose (no homing)."
                            : "No saved pose: assuming home pose (no homing).");
}

static void savePose() {
    float pose_deg[kNumJoints];
    for (uint8_t i = 0; i < kNumJoints; i++) pose_deg[i] = joints[i].position_deg;
    prefs.begin("arm", false);
    prefs.putBytes(kPoseKey, pose_deg, sizeof(pose_deg));
    prefs.end();
    poseDirty = false;
}

// ============================================
// MOTION
// ============================================
static bool anyMoving() {
    for (uint8_t i = 0; i < kNumJoints; i++) {
        if (joints[i].moving) return true;
    }
    return false;
}

// Time for a trapezoidal (or triangular, if too short to reach full speed) move from rest.
static float moveTime_s(float distance_deg, float speed_degps, float accel_degps2) {
    if (distance_deg <= 0.0f) return 0.0f;
    if (distance_deg >= speed_degps * speed_degps / accel_degps2) {
        return distance_deg / speed_degps + speed_degps / accel_degps2;
    }
    return 2.0f * sqrtf(distance_deg / accel_degps2);
}

static bool inRange(uint8_t jointIdx, float angle_deg) {
    return angle_deg >= kJoints[jointIdx].minAngle_deg && angle_deg <= kJoints[jointIdx].maxAngle_deg;
}

static void printRangeError(uint8_t jointIdx, float angle_deg) {
    Serial.printf("Rejected: joint %d target %.1f deg is outside [%.1f, %.1f]\n", jointIdx + 1,
                  angle_deg, kJoints[jointIdx].minAngle_deg, kJoints[jointIdx].maxAngle_deg);
}

// Start a coordinated move. Joints with move[i] == false keep doing what they were doing.
// Each moving joint's speed and acceleration are scaled by its share of the longest
// distance, which gives every joint the same profile shape and the same arrival time.
static bool startMove(const float target_deg[], const bool move[]) {
    for (uint8_t i = 0; i < kNumJoints; i++) {
        if (move[i] && !inRange(i, target_deg[i])) {
            printRangeError(i, target_deg[i]);
            return false;
        }
    }
    if (!outputsEnabled) {
        Serial.println("Rejected: servos are relaxed. Use 'here a b c' or 'hold' first.");
        return false;
    }

    float longest_deg = 0.0f;
    for (uint8_t i = 0; i < kNumJoints; i++) {
        if (move[i]) longest_deg = fmaxf(longest_deg, fabsf(target_deg[i] - joints[i].position_deg));
    }

    for (uint8_t i = 0; i < kNumJoints; i++) {
        if (!move[i]) continue;
        JointState& j = joints[i];
        float distance_deg = fabsf(target_deg[i] - j.position_deg);
        float scale = longest_deg > 0.0f ? distance_deg / longest_deg : 1.0f;
        j.target_deg = target_deg[i];
        j.moveSpeed_degps = fmaxf(maxSpeed_degps * scale, 0.01f);
        j.moveAccel_degps2 = fmaxf(accel_degps2 * scale, 0.01f);
        j.moving = j.target_deg != j.position_deg || j.velocity_degps != 0.0f;
        Serial.printf("Joint %d: %.1f -> %.1f deg\n", i + 1, j.position_deg, j.target_deg);
    }
    Serial.printf("Move time ~%.1f s\n", moveTime_s(longest_deg, maxSpeed_degps, accel_degps2));
    return true;
}

static void moveAll(const float target_deg[]) {
    bool move[kNumJoints];
    for (uint8_t i = 0; i < kNumJoints; i++) move[i] = true;
    startMove(target_deg, move);
}

static void moveHome() {
    float pose_deg[kNumJoints];
    for (uint8_t i = 0; i < kNumJoints; i++) pose_deg[i] = kJoints[i].homeAngle_deg;
    moveAll(pose_deg);
}

// Advance one joint by dt along its profile.
static void stepJoint(uint8_t jointIdx, float dt_s) {
    JointState& j = joints[jointIdx];
    if (!j.moving) return;

    float remaining_deg = j.target_deg - j.position_deg;
    float direction = remaining_deg >= 0.0f ? 1.0f : -1.0f;
    float speed_degps = j.velocity_degps * direction;   // positive when heading toward target
    float accel = j.moveAccel_degps2;

    if (speed_degps < 0.0f) {
        // Retargeted while heading the other way: brake first.
        speed_degps = fminf(speed_degps + accel * dt_s, 0.0f);
    } else {
        // Fastest speed from which we can still stop at the target, capped at cruise speed.
        float stopLimit_degps = sqrtf(2.0f * accel * fabsf(remaining_deg));
        float cap_degps = fminf(j.moveSpeed_degps, stopLimit_degps);
        speed_degps = fminf(speed_degps + accel * dt_s, cap_degps);
        if (speed_degps * dt_s >= fabsf(remaining_deg)) {
            j.position_deg = j.target_deg;
            j.velocity_degps = 0.0f;
            j.moving = false;
            Serial.printf("Joint %d reached %.1f deg\n", jointIdx + 1, j.position_deg);
            return;
        }
    }

    j.velocity_degps = speed_degps * direction;
    j.position_deg += j.velocity_degps * dt_s;
}

static void updateMotion() {
    static uint32_t lastUpdate_us = micros();
    uint32_t now_us = micros();
    uint32_t elapsed_us = now_us - lastUpdate_us;
    if (elapsed_us < kUpdatePeriod_us) return;
    lastUpdate_us = now_us;

    // Use the real elapsed time so a late loop doesn't slow the motion down,
    // but cap it so a long stall (e.g. flash write) doesn't cause a jump.
    float dt_s = fminf(elapsed_us, 4 * kUpdatePeriod_us) * 1e-6f;

    bool wasMoving = anyMoving();
    for (uint8_t i = 0; i < kNumJoints; i++) {
        if (!joints[i].moving) continue;
        stepJoint(i, dt_s);
        writeServo(i);
        poseDirty = true;
    }
    if (wasMoving) lastMotion_ms = millis();

    // Start the next queued pose once everything has stopped.
    if (!anyMoving() && queueCount > 0) {
        moveAll(poseQueue[queueHead]);
        queueHead = (queueHead + 1) % kMaxQueuedPoses;
        queueCount--;
    }

    // Remember where the arm is, so the next power-up can trust it.
    if (poseDirty && !anyMoving() && millis() - lastMotion_ms > kSaveDelay_ms) {
        savePose();
    }
}

static void stopAll() {
    queueCount = 0;
    for (uint8_t i = 0; i < kNumJoints; i++) {
        joints[i].target_deg = joints[i].position_deg;
        joints[i].velocity_degps = 0.0f;
        joints[i].moving = false;
    }
}

static void queuePose(const float pose_deg[]) {
    if (queueCount >= kMaxQueuedPoses) return;
    uint8_t slot = (queueHead + queueCount) % kMaxQueuedPoses;
    memcpy(poseQueue[slot], pose_deg, sizeof(poseQueue[slot]));
    queueCount++;
}

// ============================================
// COMMANDS
// ============================================
static void printHelp() {
    Serial.println("Commands:");
    Serial.println("  <joint> <angle>    e.g. '1 150' or '2 90'");
    Serial.println("  goto <a> <b> <c>   move all joints together");
    Serial.println("  home               move to the home pose");
    Serial.println("  stop               freeze where the arm is now");
    Serial.println("  speed <deg/s>      max speed");
    Serial.println("  accel <deg/s^2>    acceleration");
    Serial.println("  here <a> <b> <c>   set the believed pose without moving");
    Serial.println("  relax / hold       stop / restart servo pulses");
    Serial.println("  status             show positions");
    Serial.println("  test               sweep each joint");
}

static void printStatus() {
    Serial.println("-- Status --");
    for (uint8_t i = 0; i < kNumJoints; i++) {
        Serial.printf("Joint %d (%s): %.1f -> %.1f deg  [range %.0f..%.0f, home %.0f]\n", i + 1,
                      kJoints[i].name, joints[i].position_deg, joints[i].target_deg,
                      kJoints[i].minAngle_deg, kJoints[i].maxAngle_deg, kJoints[i].homeAngle_deg);
    }
    Serial.printf("Speed %.1f deg/s, accel %.1f deg/s^2, servos %s\n", maxSpeed_degps,
                  accel_degps2, outputsEnabled ? "holding" : "relaxed");
}

static void handleCommand(const char* line) {
    float a, b, c;
    int jointNum;

    if (strcmp(line, "status") == 0) {
        printStatus();
    } else if (strcmp(line, "help") == 0) {
        printHelp();
    } else if (strcmp(line, "stop") == 0) {
        stopAll();
        Serial.println("STOP");
    } else if (strcmp(line, "home") == 0) {
        moveHome();
    } else if (strcmp(line, "relax") == 0) {
        stopAll();
        outputsEnabled = false;
        for (uint8_t i = 0; i < kNumJoints; i++) writeServo(i);
        Serial.println("Servos relaxed. If the arm is moved by hand, use 'here a b c' before 'hold'.");
    } else if (strcmp(line, "hold") == 0) {
        outputsEnabled = true;
        for (uint8_t i = 0; i < kNumJoints; i++) writeServo(i);
        Serial.println("Servos holding.");
    } else if (strcmp(line, "test") == 0) {
        if (anyMoving()) {
            Serial.println("Rejected: wait for the current move to finish.");
            return;
        }
        float start_deg[kNumJoints], pose_deg[kNumJoints];
        for (uint8_t i = 0; i < kNumJoints; i++) start_deg[i] = joints[i].position_deg;
        memcpy(pose_deg, start_deg, sizeof(pose_deg));
        for (uint8_t i = 0; i < kNumJoints; i++) {
            pose_deg[i] = kJoints[i].maxAngle_deg; queuePose(pose_deg);
            pose_deg[i] = kJoints[i].minAngle_deg; queuePose(pose_deg);
            pose_deg[i] = start_deg[i];            queuePose(pose_deg);
        }
        Serial.println("Test sweep queued ('stop' to abort).");
    } else if (sscanf(line, "speed %f", &a) == 1) {
        if (a > 0.0f) maxSpeed_degps = a;
        Serial.printf("Speed %.1f deg/s (applies to the next move)\n", maxSpeed_degps);
    } else if (sscanf(line, "accel %f", &a) == 1) {
        if (a > 0.0f) accel_degps2 = a;
        Serial.printf("Accel %.1f deg/s^2 (applies to the next move)\n", accel_degps2);
    } else if (sscanf(line, "here %f %f %f", &a, &b, &c) == 3) {
        float pose_deg[kNumJoints] = {a, b, c};
        for (uint8_t i = 0; i < kNumJoints; i++) {
            if (!inRange(i, pose_deg[i])) { printRangeError(i, pose_deg[i]); return; }
        }
        stopAll();
        for (uint8_t i = 0; i < kNumJoints; i++) {
            joints[i].position_deg = joints[i].target_deg = pose_deg[i];
            writeServo(i);
        }
        savePose();
        Serial.println("Pose set.");
    } else if (sscanf(line, "goto %f %f %f", &a, &b, &c) == 3) {
        float pose_deg[kNumJoints] = {a, b, c};
        queueCount = 0;
        moveAll(pose_deg);
    } else if (sscanf(line, "%d %f", &jointNum, &a) == 2 && jointNum >= 1 && jointNum <= kNumJoints) {
        float target_deg[kNumJoints] = {};
        bool move[kNumJoints] = {};
        target_deg[jointNum - 1] = a;
        move[jointNum - 1] = true;
        queueCount = 0;
        startMove(target_deg, move);
    } else {
        Serial.println("Unknown command. Type 'help'.");
    }
}

static void readSerial() {
    static char buf[kInputBufferSize];
    static uint8_t len = 0;
    static char prevChar = 0;

    while (Serial.available()) {
        char ch = Serial.read();

        if (ch == '\r' || ch == '\n') {
            bool crlf = (ch == '\n' && prevChar == '\r');
            prevChar = ch;
            if (crlf) continue;   // second half of a CR LF line ending
            Serial.println();
            if (len > 0) {
                buf[len] = '\0';
                handleCommand(buf);
                len = 0;
            }
            Serial.print("> ");
            continue;
        }
        prevChar = ch;

        if ((ch == '\b' || ch == 127) && len > 0) {
            len--;
            Serial.print("\b \b");
        } else if (ch >= 32 && ch < 127 && len < kInputBufferSize - 1) {
            buf[len++] = ch;
            Serial.write(ch);
        }
    }
}

static void updateLed() {
    static uint32_t lastBlink_ms = 0;
    static bool ledOn = false;
    if (anyMoving()) {
        ledOn = true;                                  // solid while moving
    } else if (millis() - lastBlink_ms > 1000) {
        lastBlink_ms = millis();
        ledOn = !ledOn;                                // slow heartbeat when idle
    }
    digitalWrite(LED_PIN, ledOn ? LED_ON : !LED_ON);
}

// ============================================
// MAIN
// ============================================
void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, !LED_ON);

    Serial.begin(115200);
    // Give the USB serial a moment to connect, but run without a host too.
    uint32_t start_ms = millis();
    while (!Serial && millis() - start_ms < 2000) delay(10);

    Serial.println("\nRASTICrane servo controller (ESP32-S3)");
    Serial.println("======================================");

    loadPose();
    setupPwm();

    printStatus();
    Serial.println();
    printHelp();

    if (kHomeOnBoot) {
        delay(kBootHold_ms);
        Serial.println("\nMoving to home pose...");
        moveHome();
    }
    Serial.print("\n> ");
}

void loop() {
    updateMotion();
    readSerial();
    updateLed();
}
