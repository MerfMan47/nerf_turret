/**
 * ════════════════════════════════════════════════════════════════
 * COMPLETE TURRET TRACKING SYSTEM
 * ════════════════════════════════════════════════════════════════
 * Features:
 * - FastAccelStepper for smooth motor control
 * - FreeRTOS dual-core (Serial2 on Core 0, Motors on Core 1)
 * - Kalman filtering for noise reduction
 * - Ultrasonic fusion for precision when close
 * ════════════════════════════════════════════════════════════════
 */

#include "FastAccelStepper.h"
#include <math.h>

// ══════════════════════════════════════════════════════════════════
// PIN DEFINITIONS
// ══════════════════════════════════════════════════════════════════
#define YAW_STEP_PIN      16
#define YAW_DIR_PIN       15
#define PITCH_STEP_PIN    7
#define PITCH_DIR_PIN     6
#define TRIGGER_PIN       1
#define RADAR_RX          5
#define RADAR_TX          4
#define ULTRASONIC_TRIG   10
#define ULTRASONIC_ECHO   11

// ══════════════════════════════════════════════════════════════════
// MOTOR CONFIGURATION
// ══════════════════════════════════════════════════════════════════
const uint16_t MICROSTEPPING          = 8;
const uint16_t STEPS_PER_REV          = 200;
const float    YAW_GEAR_RATIO         = 3.0;           // 180/60
const float    PITCH_GEAR_RATIO       = 64.0 / 21.0;   // 3.048

const uint32_t STEPS_PER_MOTOR_REV    = STEPS_PER_REV * MICROSTEPPING;  // 1600
const uint32_t YAW_STEPS_PER_TABLE_REV   = STEPS_PER_MOTOR_REV * YAW_GEAR_RATIO;    // 4800
const uint32_t PITCH_STEPS_PER_TABLE_REV = STEPS_PER_MOTOR_REV * PITCH_GEAR_RATIO;  // ~4877

const float    YAW_STEPS_PER_DEGREE   = YAW_STEPS_PER_TABLE_REV / 360.0f;    // ~13.33
const float    PITCH_STEPS_PER_DEGREE = PITCH_STEPS_PER_TABLE_REV / 360.0f;  // ~13.55

// Startup pitch angle
const float    STARTUP_PITCH_ANGLE = 35.0f;  // Degrees to pitch up at startup
const int32_t  STARTUP_PITCH_STEPS = round(STARTUP_PITCH_ANGLE * PITCH_STEPS_PER_DEGREE);  // ~474 steps

// ══════════════════════════════════════════════════════════════════
// TRACKING SETTINGS
// ══════════════════════════════════════════════════════════════════
const float    CENTERED_THRESHOLD = 5.0f;
const unsigned long MIN_HIGH_TIME = 2000;

// ══════════════════════════════════════════════════════════════════
// BALLISTICS SETTINGS
// ══════════════════════════════════════════════════════════════════
const float TURRET_HEIGHT_CM = 0.0f;      // Height of turret above target plane (adjust for your setup)
const float TARGET_HEIGHT_CM = 0.0f;      // Estimated target center height (adjust as needed)
const float PROJECTILE_VELOCITY_FPS = 150.0f;  // Projectile velocity in feet per second
const float PROJECTILE_VELOCITY_CMPS = PROJECTILE_VELOCITY_FPS * 30.48f;  // Convert to cm/s (4572 cm/s)
const float GRAVITY_CMPS2 = 980.0f;        // Gravity acceleration in cm/s²

// ══════════════════════════════════════════════════════════════════
// FASTACCELSTEPPER OBJECTS
// ══════════════════════════════════════════════════════════════════
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *yaw_stepper = NULL;
FastAccelStepper *pitch_stepper = NULL;

// ══════════════════════════════════════════════════════════════════
// POSITION TRACKING
// ══════════════════════════════════════════════════════════════════
volatile int32_t current_yaw_position = 0;     // Current yaw position in steps from default
volatile int32_t current_pitch_position = 0;   // Current pitch position relative to startup (35°)

// ══════════════════════════════════════════════════════════════════
// FREERTOS SHARED DATA (Protected by Mutex)
// ══════════════════════════════════════════════════════════════════
SemaphoreHandle_t dataMutex;
volatile float shared_yaw_error = 0;
volatile float shared_pitch_error = 0;
volatile bool target_valid = false;
volatile float shared_raw_x = 0;
volatile float shared_raw_y = 0;
volatile bool fusion_active = false;
volatile float shared_target_distance_cm = 0;  // Distance for pitch calculation

// ══════════════════════════════════════════════════════════════════
// KALMAN FILTER STATE (YAW)
// ══════════════════════════════════════════════════════════════════
float kalman_yaw = 0;
float kalman_yaw_velocity = 0;
float kalman_yaw_error = 1.0;
const float PROCESS_NOISE = 0.01;
const float MEASUREMENT_NOISE = 3.0;
unsigned long last_kalman_time = 0;

// ══════════════════════════════════════════════════════════════════
// TRIGGER STATE
// ══════════════════════════════════════════════════════════════════
bool trigger_active = false;
unsigned long trigger_start = 0;
unsigned long last_target_seen = 0;
const unsigned long TARGET_TIMEOUT_MS = 3000;  // Return to default after 3s without target

// ══════════════════════════════════════════════════════════════════
// LD2450 FRAME PARSER
// ══════════════════════════════════════════════════════════════════
const byte HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};
byte buffer[24];
int idx = 0;
bool receiving = false;

// ══════════════════════════════════════════════════════════════════
// FUNCTION: Return to Default Position
// ══════════════════════════════════════════════════════════════════
void returnToDefault() {
  Serial.println(">>> RETURNING TO DEFAULT POSITION");
  
  // Calculate steps needed to return to default (center yaw, 35° pitch)
  int32_t yaw_return_steps = -current_yaw_position;
  int32_t pitch_return_steps = -current_pitch_position;
  
  if (yaw_return_steps != 0 || pitch_return_steps != 0) {
    Serial.printf("    Yaw: %ld steps | Pitch: %ld steps\n", yaw_return_steps, pitch_return_steps);
    
    yaw_stepper->move(yaw_return_steps);
    pitch_stepper->move(pitch_return_steps);
    
    while (yaw_stepper->isRunning() || pitch_stepper->isRunning()) {
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    
    // Reset position tracking
    current_yaw_position = 0;
    current_pitch_position = 0;
    
    Serial.println("✓ Default position restored");
  }
}

// ══════════════════════════════════════════════════════════════════
// FUNCTION: Kalman Filter
// ══════════════════════════════════════════════════════════════════
float applyKalmanFilter(float measured_angle, float dt) {
  // Predict
  float predicted_angle = kalman_yaw + (kalman_yaw_velocity * dt);
  float predicted_error = kalman_yaw_error + PROCESS_NOISE;
  
  // Update
  float kalman_gain = predicted_error / (predicted_error + MEASUREMENT_NOISE);
  kalman_yaw = predicted_angle + kalman_gain * (measured_angle - predicted_angle);
  kalman_yaw_velocity = (kalman_yaw - predicted_angle) / dt;
  kalman_yaw_error = (1 - kalman_gain) * predicted_error;
  
  return kalman_yaw;
}

// ══════════════════════════════════════════════════════════════════
// FUNCTION: Calculate Firing Pitch Angle
// ══════════════════════════════════════════════════════════════════
float calculateFiringPitch() {
  // Fire ultrasonic to get precise distance
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  // Measure echo
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000);
  if (duration == 0) {
    Serial.println("⚠️ Ultrasonic read failed - using startup pitch");
    return STARTUP_PITCH_ANGLE;
  }
  
  // Calculate distance
  float distance_cm = (duration * 0.0343) / 2.0;
  
  // Validate range
  if (distance_cm < 20 || distance_cm > 400) {
    Serial.printf("⚠️ Distance %.1fcm out of range - using startup pitch\n", distance_cm);
    return STARTUP_PITCH_ANGLE;
  }
  
  // Calculate ballistic drop compensation
  // Time of flight: t = distance / velocity
  float time_of_flight = distance_cm / PROJECTILE_VELOCITY_CMPS;
  
  // Vertical drop due to gravity: drop = 0.5 * g * t²
  float drop_cm = 0.5 * GRAVITY_CMPS2 * time_of_flight * time_of_flight;
  
  // Total height difference (turret height + drop - target height)
  float total_height_diff = TURRET_HEIGHT_CM + drop_cm - TARGET_HEIGHT_CM;
  
  // Calculate pitch angle to compensate
  // Negative pitch = aim down, Positive pitch = aim up
  float pitch_angle = -atan2(total_height_diff, distance_cm) * (180.0 / PI);
  
  Serial.printf("🎯 BALLISTICS: Dist=%.1fcm | TOF=%.3fs | Drop=%.1fcm | Total Height=%.1fcm | Pitch=%.2f°\n",
                distance_cm, time_of_flight, drop_cm, total_height_diff, pitch_angle);
  
  return pitch_angle;
}
float applyUltrasonicFusion(float radar_x, float radar_y, float radar_angle) {
  // Only fuse when nearly centered
  if (fabs(radar_x) > 100) {
    fusion_active = false;
    return radar_angle;
  }
  
  // Fire ultrasonic pulse
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  // Measure echo
  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000);
  if (duration == 0) {
    fusion_active = false;
    return radar_angle;
  }
  
  // Calculate precise distance
  float ultrasonic_y_cm = (duration * 0.0343) / 2.0;
  float ultrasonic_y_mm = ultrasonic_y_cm * 10.0;
  
  // Validate range
  if (ultrasonic_y_cm < 20 || ultrasonic_y_cm > 400) {
    fusion_active = false;
    return radar_angle;
  }
  
  // Recalculate X using precise Y
  float fused_x_mm = ultrasonic_y_mm * tan(radar_angle * PI / 180.0);
  float fused_angle = atan2(fused_x_mm, ultrasonic_y_mm) * (180.0 / PI);
  
  fusion_active = true;
  Serial.printf("FUSION: Y %.0f→%.0fmm | X %.0f→%.0fmm | Angle %.2f°→%.2f°\n",
                radar_y, ultrasonic_y_mm, radar_x, fused_x_mm, radar_angle, fused_angle);
  
  return fused_angle;
}

// ══════════════════════════════════════════════════════════════════
// FREERTOS TASK: Radar Parsing (Core 0)
// ══════════════════════════════════════════════════════════════════
void radarTask(void *parameter) {
  Serial.println("Radar task on Core " + String(xPortGetCoreID()));
  
  while (true) {
    while (Serial2.available()) {
      byte b = Serial2.read();
      
      if (!receiving) {
        if (b == HEADER[idx]) {
          idx++;
          if (idx == 4) { receiving = true; idx = 0; }
        } else idx = 0;
      } else {
        if (idx < 24) buffer[idx++] = b;
        else if (idx == 24 && b == 0x55) idx = 25;
        else if (idx == 25 && b == 0xCC) {
          parseLD2450Frame();
          receiving = false;
          idx = 0;
        } else {
          receiving = false;
          idx = 0;
        }
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

// ══════════════════════════════════════════════════════════════════
// FUNCTION: Parse LD2450 Frame
// ══════════════════════════════════════════════════════════════════
void parseLD2450Frame() {
  float best_x_mm = 0, best_y_mm = 0;
  float best_dist = 999999.0f;
  bool found = false;
  
  for (int i = 0; i < 3; i++) {
    byte* g = buffer + i * 8;
    
    uint16_t raw_x = g[0] | (g[1] << 8);
    uint16_t raw_y = g[2] | (g[3] << 8);
    uint16_t raw_speed = g[4] | (g[5] << 8);
    uint16_t gate = g[6] | (g[7] << 8);
    
    if (raw_x == 0 && raw_y == 0 && raw_speed == 0 && gate == 0) continue;
    
    int16_t x_mm = (g[1] & 0x80) ? (raw_x & 0x7FFF) : -(raw_x & 0x7FFF);
    int16_t y_mm = (g[3] & 0x80) ? (raw_y & 0x7FFF) : -(raw_y & 0x7FFF);
    
    float dist = sqrtf(x_mm * x_mm + y_mm * y_mm);
    if (dist > 300 && dist < best_dist) {
      best_dist = dist;
      best_x_mm = x_mm;
      best_y_mm = y_mm;
      found = true;
    }
  }
  
  // Update shared data with Kalman + Fusion
  if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
    if (found && best_dist < 6000) {
      float x_cm = best_x_mm / 10.0f;
      float y_cm = best_y_mm / 10.0f;
      float raw_yaw = atan2f(x_cm, y_cm) * (180.0f / PI);
      
      // STEP 1: Kalman filter
      unsigned long now = millis();
      float dt = (now - last_kalman_time) / 1000.0;
      float filtered_yaw = (dt > 0 && dt < 1.0) ? applyKalmanFilter(raw_yaw, dt) : raw_yaw;
      if (dt >= 1.0) { kalman_yaw = raw_yaw; kalman_yaw_velocity = 0; }
      last_kalman_time = now;
      
      // STEP 2: Ultrasonic fusion
      float final_yaw = applyUltrasonicFusion(best_x_mm, best_y_mm, filtered_yaw);
      
      // Store distance for firing pitch calculation
      float distance_cm = sqrtf(x_cm * x_cm + y_cm * y_cm);
      
      shared_yaw_error = final_yaw;
      shared_pitch_error = 0;  // Pitch will be calculated just before firing
      shared_raw_x = best_x_mm;
      shared_raw_y = best_y_mm;
      shared_target_distance_cm = distance_cm;
      target_valid = true;
      last_target_seen = millis();  // Update last seen time
      
      if (fusion_active) {
        Serial.printf("Target: X=%.1fcm Y=%.1fcm | Raw=%.2f° | Kalman=%.2f° | FUSED=%.2f° 🎯\n",
                      x_cm, y_cm, raw_yaw, filtered_yaw, final_yaw);
      } else {
        Serial.printf("Target: X=%.1fcm Y=%.1fcm | Raw=%.2f° | Kalman=%.2f°\n",
                      x_cm, y_cm, raw_yaw, filtered_yaw);
      }
    } else {
      shared_yaw_error = 0;
      shared_pitch_error = 0;
      shared_target_distance_cm = 0;
      target_valid = false;
      fusion_active = false;
    }
    xSemaphoreGive(dataMutex);
  }
}

// ══════════════════════════════════════════════════════════════════
// FREERTOS TASK: Motion Control (Core 1)
// ══════════════════════════════════════════════════════════════════
void motionTask(void *parameter) {
  Serial.println("Motion task on Core " + String(xPortGetCoreID()));
  
  bool pitch_adjusted = false;  // Track if we've adjusted pitch for firing
  bool at_default_position = true;  // Track if we're at default position
  
  while (true) {
    // Read shared data
    float yaw_error, pitch_error;
    bool valid;
    unsigned long last_seen;
    
    if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
      yaw_error = shared_yaw_error;
      pitch_error = shared_pitch_error;
      valid = target_valid;
      last_seen = last_target_seen;
      xSemaphoreGive(dataMutex);
    }
    
    float abs_yaw_error = fabs(yaw_error);
    unsigned long time_since_target = millis() - last_seen;
    
    // Check if we should return to default position
    if (!valid || time_since_target > TARGET_TIMEOUT_MS) {
      if (!at_default_position) {
        returnToDefault();
        at_default_position = true;
        pitch_adjusted = false;
        trigger_active = false;
        digitalWrite(TRIGGER_PIN, LOW);
      }
      vTaskDelay(100 / portTICK_PERIOD_MS);
      continue;
    }
    
    if (valid && abs_yaw_error > CENTERED_THRESHOLD) {
      // Move toward target (yaw only during tracking)
      int32_t yaw_steps = round(-yaw_error * YAW_STEPS_PER_DEGREE);
      
      Serial.printf(">>> MOVING Yaw: %.1f° (%ld steps)\n", -yaw_error, yaw_steps);
      
      yaw_stepper->move(yaw_steps);
      current_yaw_position += yaw_steps;  // Track position
      at_default_position = false;
      
      while (yaw_stepper->isRunning()) {
        vTaskDelay(1 / portTICK_PERIOD_MS);
      }
      
      trigger_active = false;
      pitch_adjusted = false;  // Reset pitch adjustment flag
      digitalWrite(TRIGGER_PIN, LOW);
      
    } else if (valid && abs_yaw_error <= CENTERED_THRESHOLD) {
      // Centered on target - calculate and adjust pitch before firing
      if (!pitch_adjusted) {
        Serial.println(">>> YAW LOCKED - Calculating firing pitch...");
        
        // Calculate pitch angle based on ultrasonic distance
        float pitch_adjustment = calculateFiringPitch();
        
        int32_t pitch_steps = round(pitch_adjustment * PITCH_STEPS_PER_DEGREE);
        
        if (fabs(pitch_adjustment) > 0.5) {  // Only adjust if significant
          Serial.printf(">>> Adjusting pitch by %.2f° (%ld steps)\n", pitch_adjustment, pitch_steps);
          pitch_stepper->move(pitch_steps);
          current_pitch_position += pitch_steps;  // Track position
          at_default_position = false;
          
          while (pitch_stepper->isRunning()) {
            vTaskDelay(1 / portTICK_PERIOD_MS);
          }
        }
        
        pitch_adjusted = true;
        Serial.println(">>> PITCH ADJUSTED - Ready to fire");
      }
      
      // Now fire
      digitalWrite(TRIGGER_PIN, HIGH);
      if (!trigger_active) {
        trigger_active = true;
        trigger_start = millis();
        Serial.println(">>> FIRING - PIN 1 HIGH");
      }
    }
    
    // Turn off after timeout
    if (trigger_active && (millis() - trigger_start >= MIN_HIGH_TIME)) {
      if (!valid || abs_yaw_error > 30.0f) {
        trigger_active = false;
        pitch_adjusted = false;  // Reset for next target
        digitalWrite(TRIGGER_PIN, LOW);
        Serial.println(">>> Target lost - PIN 1 LOW");
      }
    }
    
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// ══════════════════════════════════════════════════════════════════
// LD2450 CONFIGURATION COMMANDS
// ══════════════════════════════════════════════════════════════════
void sendLD2450Command(const byte* cmd, size_t len) {
  Serial2.write(cmd, len);
  delay(50);  // Wait for ACK
  
  // Read and display ACK
  if (Serial2.available()) {
    Serial.print("LD2450 ACK: ");
    while (Serial2.available()) {
      Serial.printf("%02X ", Serial2.read());
    }
    Serial.println();
  }
}

void configureLD2450() {
  Serial.println("Configuring LD2450 radar...");
  delay(1000);  // Let radar stabilize
  
  // 1. Enable configuration mode
  const byte enableConfig[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
  Serial.println("→ Enable configuration mode");
  sendLD2450Command(enableConfig, sizeof(enableConfig));
  delay(100);
  
  // 2. Set to SINGLE target tracking (best for turret)
  const byte singleTarget[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x80, 0x00, 0x04, 0x03, 0x02, 0x01};
  Serial.println("→ Set single target tracking");
  sendLD2450Command(singleTarget, sizeof(singleTarget));
  delay(100);
  
  // 3. Bluetooth control (choose one - comment out the other)
  // OPTION A: Disable Bluetooth (reduce interference)
  // const byte disableBT[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xA4, 0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01};
  // Serial.println("→ Disable Bluetooth");
  // sendLD2450Command(disableBT, sizeof(disableBT));
  // delay(100);
  
  // OPTION B: Enable Bluetooth (for wireless monitoring)
  const byte enableBT[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xA4, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
  Serial.println("→ Enable Bluetooth");
  sendLD2450Command(enableBT, sizeof(enableBT));
  delay(100);
  
  // 4. OPTIONAL: Set region filtering (example: detect only front 120° arc, 0.5m-6m range)
  // Coordinates in mm: X1=-3000 Y1=500 X2=3000 Y2=6000 (modify as needed)
  // Type: 0x0001 = detect only in region
  const byte regionFilter[] = {
    0xFD, 0xFC, 0xFB, 0xFA,  // Header
    0x1C, 0x00,              // Length (28 bytes)
    0xC2, 0x00,              // Command 0x00C2
    0x01, 0x00,              // Type: 0x0001 (detect only)
    0x48, 0xF4,              // X1: -3000mm (0xF448 little-endian signed)
    0xF4, 0x01,              // Y1: 500mm
    0xB8, 0x0B,              // X2: 3000mm
    0x70, 0x17,              // Y2: 6000mm
    0x00, 0x00, 0x00, 0x00,  // Region 2: disabled
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,  // Region 3: disabled
    0x00, 0x00, 0x00, 0x00,
    0x04, 0x03, 0x02, 0x01   // End
  };
  Serial.println("→ Set region filter (120° arc, 0.5m-6m)");
  sendLD2450Command(regionFilter, sizeof(regionFilter));
  delay(100);
  
  // 5. End configuration mode
  const byte endConfig[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};
  Serial.println("→ End configuration mode");
  sendLD2450Command(endConfig, sizeof(endConfig));
  delay(100);
  
  // 6. Read firmware version for verification
  delay(500);
  const byte readVersion[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
  sendLD2450Command(readVersion, sizeof(readVersion));
  delay(100);
  const byte getVersion[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xA0, 0x00, 0x04, 0x03, 0x02, 0x01};
  Serial.println("→ Read firmware version");
  sendLD2450Command(getVersion, sizeof(getVersion));
  delay(100);
  const byte endConfig2[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};
  sendLD2450Command(endConfig2, sizeof(endConfig2));
  
  Serial.println("✓ LD2450 configuration complete");
}

// ══════════════════════════════════════════════════════════════════
// SETUP
// ══════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);
  digitalWrite(TRIGGER_PIN, LOW);
  digitalWrite(ULTRASONIC_TRIG, LOW);
  
  // Create mutex
  dataMutex = xSemaphoreCreateMutex();
  if (!dataMutex) {
    Serial.println("Mutex FAILED!");
    while(1);
  }
  
  // Initialize FastAccelStepper
  engine.init();
  
  yaw_stepper = engine.stepperConnectToPin(YAW_STEP_PIN);
  if (yaw_stepper) {
    yaw_stepper->setDirectionPin(YAW_DIR_PIN);
    yaw_stepper->setEnablePin(-1);
    yaw_stepper->setAutoEnable(false);
    yaw_stepper->setSpeedInHz(4000);
    yaw_stepper->setAcceleration(5000);
    Serial.println("✓ Yaw stepper ready");
  } else {
    Serial.println("✗ Yaw stepper FAILED!");
    while(1);
  }
  
  pitch_stepper = engine.stepperConnectToPin(PITCH_STEP_PIN);
  if (pitch_stepper) {
    pitch_stepper->setDirectionPin(PITCH_DIR_PIN);
    pitch_stepper->setEnablePin(-1);
    pitch_stepper->setAutoEnable(false);
    pitch_stepper->setSpeedInHz(2000);      // Slower for pitch
    pitch_stepper->setAcceleration(2500);   // Gentler for pitch
    Serial.println("✓ Pitch stepper ready");
  } else {
    Serial.println("✗ Pitch stepper FAILED!");
    while(1);
  }
  
  // Pitch up 35 degrees at startup
  Serial.printf(">>> Pitching up %.1f degrees (%ld steps)...\n", STARTUP_PITCH_ANGLE, STARTUP_PITCH_STEPS);
  pitch_stepper->move(STARTUP_PITCH_STEPS);
  while (pitch_stepper->isRunning()) {
    delay(10);
  }
  Serial.println("✓ Startup pitch complete");
  
  // Start radar and configure it
  Serial2.begin(256000, SERIAL_8N1, RADAR_RX, RADAR_TX);
  delay(1000);
  
  // Configure LD2450 for optimal turret performance
  configureLD2450();
  delay(1000);
  
  Serial.println("════════════════════════════════════════════════════════");
  Serial.println("  TURRET: FastAccel + Kalman + Ultrasonic Fusion");
  Serial.printf("  Yaw: %.1f steps/degree | Pitch: %.1f steps/degree\n", 
                YAW_STEPS_PER_DEGREE, PITCH_STEPS_PER_DEGREE);
  Serial.println("════════════════════════════════════════════════════════");
  
  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(radarTask, "Radar", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(motionTask, "Motion", 4096, NULL, 1, NULL, 1);
}

// ══════════════════════════════════════════════════════════════════
// LOOP (Idle)
// ══════════════════════════════════════════════════════════════════
void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}