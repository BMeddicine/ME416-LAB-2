#include <Arduino.h>

// ============== CONFIGURATION ==============
// Servo pins
#define SERVO1_PIN 9    // 180° servo
#define SERVO2_PIN 10   // 180° servo
#define SERVO3_PIN 11   // 300° servo

// PWM settings
#define PWM_FREQ 50
#define PWM_RESOLUTION 12

// Servo specifications
#define SERVO1_MIN_US 500
#define SERVO1_MAX_US 2500
#define SERVO1_MAX_DEG 180

#define SERVO2_MIN_US 500
#define SERVO2_MAX_US 2500
#define SERVO2_MAX_DEG 180

#define SERVO3_MIN_US 500
#define SERVO3_MAX_US 2500
#define SERVO3_MAX_DEG 300

// Default speed (degrees per second)
#define DEFAULT_SPEED_DPS 30.0

// ============== SERVO DATA STRUCTURE ==============
struct ServoConfig {
  int pin;
  int minUs;
  int maxUs;
  int maxDeg;
  const char* name;
  float currentUs;
  float targetUs;
  bool isMoving;
};

ServoConfig servo1 = {SERVO1_PIN, SERVO1_MIN_US, SERVO1_MAX_US, SERVO1_MAX_DEG, "Servo 1 (180°)", 1500, 1500, false};
ServoConfig servo2 = {SERVO2_PIN, SERVO2_MIN_US, SERVO2_MAX_US, SERVO2_MAX_DEG, "Servo 2 (180°)", 1500, 1500, false};
ServoConfig servo3 = {SERVO3_PIN, SERVO3_MIN_US, SERVO3_MAX_US, SERVO3_MAX_DEG, "Servo 3 (300°)", 1500, 1500, false};

ServoConfig* servos[] = {&servo1, &servo2, &servo3};
const int NUM_SERVOS = 3;

// Global speed setting
float servoSpeedDPS = DEFAULT_SPEED_DPS;

// ============== HELPER FUNCTIONS ==============

int dutyFromUs(float microseconds) {
  return (int)((microseconds * 4095.0) / 20000.0);
}

// Convert degrees to microseconds for a specific servo
float degToUs(ServoConfig* s, float degrees) {
  if (degrees < 0) degrees = 0;
  if (degrees > s->maxDeg) degrees = s->maxDeg;
  return s->minUs + (degrees / s->maxDeg) * (s->maxUs - s->minUs);
}

// Convert microseconds to degrees
float usToDeg(ServoConfig* s, float us) {
  return (us - s->minUs) / (s->maxUs - s->minUs) * s->maxDeg;
}

// ============== UNIVERSAL MOVE FUNCTION ==============

/**
 * Move servo(s) to target position(s)
 * 
 * Parameters:
 *   s1, s2, s3: Target degrees for each servo, or -1 to skip
 *   speedDPS: Movement speed in degrees/sec (use negative for default)
 * 
 * Returns: true if all requested moves were valid
 * 
 * Examples:
 *   moveServos(45, -1, -1);        // Move only servo 1 to 45°
 *   moveServos(90, 90, 150);       // Move all servos to center
 *   moveServos(0, 180, 300, 60);   // Move all at 60°/sec
 *   moveServos(-1, 45, -1);        // Move only servo 2
 */
bool moveServos(float s1, float s2, float s3 = -1, float speedDPS = -1) {
  float targets[3] = {s1, s2, s3};
  float speed = (speedDPS > 0) ? speedDPS : servoSpeedDPS;
  bool allValid = true;
  
  // Validate and set targets
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (targets[i] < 0) continue;  // Skip this servo
    
    ServoConfig* s = servos[i];
    
    if (targets[i] > s->maxDeg) {
      Serial.print("ERROR: "); 
      Serial.print(s->name);
      Serial.print(" max is "); 
      Serial.println(s->maxDeg);
      allValid = false;
      continue;
    }
    
    s->targetUs = degToUs(s, targets[i]);
    s->isMoving = true;
  }
  
  // Report movement start
  Serial.println();
  Serial.println(">>> MOVE COMMAND <<<");
  
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (targets[i] < 0) {
      Serial.print("  "); 
      Serial.print(servos[i]->name);
      Serial.println(": [UNCHANGED]");
      continue;
    }
    
    Serial.print("  "); 
    Serial.print(servos[i]->name);
    Serial.print(": ");
    Serial.print(usToDeg(servos[i], servos[i]->currentUs), 1);
    Serial.print("° → ");
    Serial.print(targets[i], 1);
    Serial.print("° (");
    Serial.print((int)servos[i]->targetUs);
    Serial.print("us) | SPEED: ");
    Serial.print(speed);
    Serial.println("°/sec");
  }
  
  // Blocking movement (wait for completion)
  unsigned long maxStartTime = millis();
  
  while (true) {
    bool anyMoving = false;
    unsigned long now = millis();
    static unsigned long lastUpdate = 0;
    float dt = (now - lastUpdate) / 1000.0;
    lastUpdate = now;
    
    for (int i = 0; i < NUM_SERVOS; i++) {
      if (!servos[i]->isMoving) continue;
      anyMoving = true;
      
      float usPerDeg = (float)(servos[i]->maxUs - servos[i]->minUs) / servos[i]->maxDeg;
      float maxDeltaUs = speed * usPerDeg * dt;
      float error = servos[i]->targetUs - servos[i]->currentUs;
      
      if (abs(error) <= maxDeltaUs) {
        // Arrived
        servos[i]->currentUs = servos[i]->targetUs;
        servos[i]->isMoving = false;
        
        Serial.print("    ✓ "); 
        Serial.print(servos[i]->name);
        Serial.print(" DONE at ");
        Serial.print(usToDeg(servos[i], servos[i]->currentUs), 1);
        Serial.println("°");
        
      } else {
        // Keep moving
        float direction = (error > 0) ? 1.0 : -1.0;
        servos[i]->currentUs += direction * maxDeltaUs;
      }
      
      ledcWrite(servos[i]->pin, dutyFromUs(servos[i]->currentUs));
    }
    
    if (!anyMoving) break;
    
    delay(1);
  }
  
  Serial.println("<<< COMPLETE >>>");
  Serial.println();
  
  return allValid;
}

// ============== NON-BLOCKING VERSION ==============

/**
 * Non-blocking version: Set targets and return immediately.
 * Call updateServos() in loop() to execute movement.
 */
void setServoTargets(float s1, float s2, float s3, float speedDPS = -1) {
  const char* names[] = {"Servo 1", "Servo 2", "Servo 3"};
  float targets[] = {s1, s2, s3};
  
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (targets[i] < 0) continue;
    if (targets[i] > servos[i]->maxDeg) {
      Serial.print("Invalid: "); Serial.println(names[i]);
      continue;
    }
    servos[i]->targetUs = degToUs(servos[i], targets[i]);
    servos[i]->isMoving = true;
  }
  
  if (speedDPS > 0) servoSpeedDPS = speedDPS;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize PWM
  for (int i = 0; i < NUM_SERVOS; i++) {
    ledcAttach(servos[i]->pin, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(servos[i]->pin, dutyFromUs(servos[i]->currentUs));
  }
  
  Serial.println("========================================");
  Serial.println("  UNIVERSAL SERVO CONTROLLER");
  Serial.println("========================================");
  Serial.println();
  Serial.println("FUNCTION: moveServos(s1, s2, s3, speed)");
  Serial.println("  s1,s2,s3: degrees, or -1 to skip");
  Serial.println("  speed: optional °/sec");
  Serial.println();
  Serial.println("Examples in code:");
  Serial.println("  moveServos(90, -1, -1, 30);     // Servo 1 only");
  Serial.println("  moveServos(0, 180, 150, 30);    // All to positions");
  Serial.println("  moveServos(45, 45, 75, 60); // All at 60°/sec");
  Serial.println();
  
  // Demo sequence using the universal function
  Serial.println("--- RUNNING DEMO SEQUENCE ---");
  Serial.println();
  
  delay(1000);
  
  // Demo: Move all to center
  moveServos(90, 90, 150);
  
  delay(5000);
  
  // Demo: Sweep servo 1 only
  moveServos(0, -1, -1);
  moveServos(180, -1, -1);
  moveServos(90, -1, -1);
  
  delay(5000);
  
  // Demo: Move all three to different positions
  moveServos(45, 135, 75, 30);

  delay(1000);
  
  // Demo: Complex pattern
  for (int i = 0; i <= 180; i += 45) {
    moveServos(i, 180-i, (i * 300) / 180, 30);
    delay(1000);
  }
  
  // Return to center
  moveServos(90, 90, 150, 30);
  
  Serial.println("--- DEMO COMPLETE ---");
  Serial.println("Use Serial commands or modify code to move servos");
  Serial.println();
  
  // Interactive mode
  interactiveMode();
}

void interactiveMode() {
  Serial.println("INTERACTIVE MODE:");
  Serial.println("Enter: 1 <deg> | 2 <deg> | 3 <deg> | a <s1> <s2> <s3> | s <speed>");
  Serial.println();
  
  while (true) {
    if (!Serial.available()) {
      delay(10);
      continue;
    }
    
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.length() == 0) continue;
    
    char type = cmd.charAt(0);
    
    if (type == 'a') {
      // All servos: a <s1> <s2> <s3>
      float s1 = cmd.substring(1).toFloat();
      // Simple parsing - assumes space-separated
      int firstSpace = cmd.indexOf(' ');
      int secondSpace = cmd.indexOf(' ', firstSpace + 1);
      int thirdSpace = cmd.indexOf(' ', secondSpace + 1);
      
      if (firstSpace > 0 && secondSpace > firstSpace) {
        String v1 = cmd.substring(firstSpace + 1, secondSpace);
        String rest = cmd.substring(secondSpace + 1);
        
        secondSpace = rest.indexOf(' ');
        String v2 = (secondSpace > 0) ? rest.substring(0, secondSpace) : rest;
        String v3 = (secondSpace > 0) ? rest.substring(secondSpace + 1) : "";
        
        float f1 = v1.toFloat();
        float f2 = v2.toFloat();
        float f3 = (v3.length() > 0) ? v3.toFloat() : -1;
        
        moveServos(f1, f2, f3);
      }
      
    } else if (type >= '1' && type <= '3') {
      // Single servo: 1 <deg>
      int servoIdx = type - '1';
      float deg = cmd.substring(1).toFloat();
      
      float targets[3] = {-1, -1, -1};
      targets[servoIdx] = deg;
      moveServos(targets[0], targets[1], targets[2]);
      
    } else if (type == 's') {
      // Set speed
      float newSpeed = cmd.substring(1).toFloat();
      if (newSpeed > 0) {
        servoSpeedDPS = newSpeed;
        Serial.print("Speed set to "); Serial.print(servoSpeedDPS); Serial.println("°/sec");
      }
      
    } else if (type == 'p') {
      // Print positions
      Serial.println("\n--- POSITIONS ---");
      for (int i = 0; i < 3; i++) {
        Serial.print(servos[i]->name);
        Serial.print(": "); 
        Serial.print(usToDeg(servos[i], servos[i]->currentUs), 2);
        Serial.print("° ("); 
        Serial.print((int)servos[i]->currentUs);
        Serial.println("us)");
      }
      Serial.println();
      
    } else {
      Serial.println("Unknown. Use: 1/2/3 <deg> | a <s1> <s2> <s3> | s <speed> | p");
    }
  }
}

void loop() {
  // Nothing here - moveServos is blocking
  // Or use setServoTargets() + updateServos() for non-blocking
}