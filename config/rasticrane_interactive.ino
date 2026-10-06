#include <Arduino.h>

// ============== CONFIGURATION ==============
// Servo pins
#define SERVO1_PIN 9    // 180° servo
#define SERVO2_PIN 10   // 180° servo
#define SERVO3_PIN 11   // 300° servo

// PWM settings
#define PWM_FREQ 50
#define PWM_RESOLUTION 12

// ============== SERVO SPECIFICATIONS ==============
// Timing limits (microseconds)
#define SERVO1_MIN_US 500
#define SERVO1_MAX_US 2500
#define SERVO2_MIN_US 500
#define SERVO2_MAX_US 2500
#define SERVO3_MIN_US 500
#define SERVO3_MAX_US 2500

// Degree ranges
#define SERVO1_MAX_DEG 180
#define SERVO2_MAX_DEG 180
#define SERVO3_MAX_DEG 300

// ============== SPEED CONTROL ==============
// Default: degrees per second (adjustable via serial)
float servoSpeedDPS = 30.0;  // Default 30 degrees/second

// ============== CURRENT STATE ==============
struct ServoState {
  int pin;
  int minUs;
  int maxUs;
  int maxDeg;
  const char* name;
  float currentUs;
  float targetUs;
  bool isMoving;
};

ServoState servos[3] = {
  {SERVO1_PIN, SERVO1_MIN_US, SERVO1_MAX_US, SERVO1_MAX_DEG, "Servo 1 (180°, pin 9)", 1500.0, 1500.0, false},
  {SERVO2_PIN, SERVO2_MIN_US, SERVO2_MAX_US, SERVO2_MAX_DEG, "Servo 2 (180°, pin 10)", 1500.0, 1500.0, false},
  {SERVO3_PIN, SERVO3_MIN_US, SERVO3_MAX_US, SERVO3_MAX_DEG, "Servo 3 (300°, pin 11)", 1500.0, 1500.0, false}
};

// ============== FUNCTIONS ==============

int dutyFromUs(float microseconds) {
  return (int)((microseconds * 4095.0) / 20000.0);
}

// Convert degrees to microseconds for each servo
float degToUs(ServoState& s, float degrees) {
  // Clamp to valid range
  if (degrees < 0) degrees = 0;
  if (degrees > s.maxDeg) degrees = s.maxDeg;
  
  // Linear mapping: 0° -> minUs, maxDeg -> maxUs
  return s.minUs + (degrees / s.maxDeg) * (s.maxUs - s.minUs);
}

// Convert microseconds to degrees
float usToDeg(ServoState& s, float us) {
  return (us - s.minUs) / (s.maxUs - s.minUs) * s.maxDeg;
}

void printStatus() {
  Serial.println("\n--- CURRENT STATUS ---");
  Serial.print("Speed: "); Serial.print(servoSpeedDPS); Serial.println(" deg/sec");
  for (int i = 0; i < 3; i++) {
    Serial.print(servos[i].name);
    Serial.print(" | Pos: "); Serial.print(usToDeg(servos[i], servos[i].currentUs), 1);
    Serial.print("° ("); Serial.print((int)servos[i].currentUs); Serial.print("us)");
    Serial.print(" | "); Serial.println(servos[i].isMoving ? "MOVING" : "STOPPED");
  }
  Serial.println("----------------------\n");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("========================================");
  Serial.println("    ESP32 SERVO CONTROL SYSTEM");
  Serial.println("========================================");
  Serial.println();
  
  // Attach servos and initialize to center (90°, 150°, 150° respectively)
  for (int i = 0; i < 3; i++) {
    ledcAttach(servos[i].pin, PWM_FREQ, PWM_RESOLUTION);
    
    // Center positions: 90° for 180° servos, 150° for 300° servo, changed to 90
    float centerDeg = (servos[i].maxDeg == 180) ? 90.0 : 150.0;
    servos[i].currentUs = degToUs(servos[i], centerDeg);
    servos[i].targetUs = servos[i].currentUs;
    
    ledcWrite(servos[i].pin, dutyFromUs(servos[i].currentUs));
  }
  
  printStatus();
  
  Serial.println("COMMANDS:");
  Serial.println("  1 <degrees>  - Move Servo 1 (0-180)");
  Serial.println("  2 <degrees>  - Move Servo 2 (0-180)");
  Serial.println("  3 <degrees>  - Move Servo 3 (0-300)");
  Serial.println("  s <speed>    - Set speed (degrees/sec)");
  Serial.println("  p            - Print status");
  Serial.println("  h            - Help");
  Serial.println();
}

void processSerial() {
  if (!Serial.available()) return;
  
  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;
  
  char cmd = input.charAt(0);
  
  // Help
  if (cmd == 'h' || cmd == 'H') {
    Serial.println("Commands: 1/2/3 <deg> | s <speed> | p | h");
    return;
  }
  
  // Print status
  if (cmd == 'p' || cmd == 'P') {
    printStatus();
    return;
  }
  
  // Set speed
  if (cmd == 's' || cmd == 'S') {
    float newSpeed = input.substring(1).toFloat();
    if (newSpeed > 0 && newSpeed <= 500) {
      servoSpeedDPS = newSpeed;
      Serial.print("Speed set to "); Serial.print(servoSpeedDPS); Serial.println(" deg/sec");
    } else {
      Serial.println("ERROR: Speed must be 0-500 deg/sec");
    }
    return;
  }
  
  // Servo commands (1, 2, or 3)
  int servoIdx = -1;
  if (cmd == '1') servoIdx = 0;
  else if (cmd == '2') servoIdx = 1;
  else if (cmd == '3') servoIdx = 2;
  
  if (servoIdx == -1) {
    Serial.println("ERROR: Unknown command. Use h for help.");
    return;
  }
  
  // Parse target degrees
  float targetDeg = input.substring(1).toFloat();
  
  // Validate range
  if (targetDeg < 0 || targetDeg > servos[servoIdx].maxDeg) {
    Serial.print("ERROR: "); 
    Serial.print(servos[servoIdx].name);
    Serial.print(" range is 0-"); 
    Serial.println(servos[servoIdx].maxDeg);
    return;
  }
  
  // Set target (convert to microseconds)
  float targetUs = degToUs(servos[servoIdx], targetDeg);
  servos[servoIdx].targetUs = targetUs;
  
  // Debug: Report movement start
  Serial.println();
  Serial.print(">>> STARTING: ");
  Serial.print(servos[servoIdx].name);
  Serial.print(" | FROM: "); Serial.print(usToDeg(servos[servoIdx], servos[servoIdx].currentUs), 1);
  Serial.print("° ("); Serial.print((int)servos[servoIdx].currentUs); Serial.print("us)");
  Serial.print(" | TO: "); Serial.print(targetDeg, 1);
  Serial.print("° ("); Serial.print((int)targetUs); Serial.println("us)");
  
  servos[servoIdx].isMoving = true;
}

void updateServos() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0;  // seconds
  lastUpdate = now;
  
  if (dt == 0) return;
  
  // Calculate step size based on speed (convert deg/sec to us/sec, then to us per frame)
  for (int i = 0; i < 3; i++) {
    if (!servos[i].isMoving) continue;
    
    // us per degree for this servo
    float usPerDeg = (float)(servos[i].maxUs - servos[i].minUs) / servos[i].maxDeg;
    
    // Maximum change this frame
    float maxDeltaUs = servoSpeedDPS * usPerDeg * dt;
    
    float error = servos[i].targetUs - servos[i].currentUs;
    
    if (abs(error) <= maxDeltaUs) {
      // Arrived at target
      servos[i].currentUs = servos[i].targetUs;
      ledcWrite(servos[i].pin, dutyFromUs(servos[i].currentUs));
      servos[i].isMoving = false;
      
      // Debug: Report completion
      Serial.print("<<< COMPLETE: ");
      Serial.print(servos[i].name);
      Serial.print(" at "); Serial.print(usToDeg(servos[i], servos[i].currentUs), 1);
      Serial.print("° ("); Serial.print((int)servos[i].currentUs); Serial.println("us)");
      Serial.println();
      
    } else {
      // Still moving
      float direction = (error > 0) ? 1.0 : -1.0;
      servos[i].currentUs += direction * maxDeltaUs;
      ledcWrite(servos[i].pin, dutyFromUs(servos[i].currentUs));
    }
  }
}

void loop() {
  processSerial();
  updateServos();
}