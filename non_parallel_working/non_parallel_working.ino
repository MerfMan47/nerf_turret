#include "FastAccelStepper.h"
#include <math.h>

// ──────────────────────────────
// YOUR EXACT PINS
// ──────────────────────────────
#define STEP_PIN      2
#define DIR_PIN       7

#define TRIGGER_PIN   9
#define RADAR_RX      6   // ESP32 RX ← LD2450 TX (Yellow)
#define RADAR_TX      5   // ESP32 TX → LD2450 RX (Blue)

// ──────────────────────────────
// Motor: 200 steps/rev × 256 µsteps × 3:1 gear = 153600 steps per 360°
// ──────────────────────────────
// const uint32_t STEPS_PER_TABLE_REV = 200UL * 256 * 1;           // 153600 was * 3
// const float    STEPS_PER_DEGREE    = STEPS_PER_TABLE_REV / 360.0f;  // ~426.67
const float STEPS_PER_DEGREE = 13.34;

// ──────────────────────────────
// Tracking Settings
// ──────────────────────────────
const float    CENTERED_THRESHOLD = 5.0f;   // If |angle| ≤ 5° → facing
const unsigned long MIN_HIGH_TIME = 2000;   // Keep pin 9 HIGH at least 2s

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

bool trigger_active = false;
unsigned long trigger_start = 0;

// LD2450 frame parser
const byte HEADER[4] = {0xAA, 0xFF, 0x03, 0x00};
byte buffer[24];
int idx = 0;
bool receiving = false;

float current_angle_error = 0;  // degrees, positive = target to the right

void setup() {
  Serial.begin(115200);
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW);

  // Initialize FastAccelStepper
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (stepper) {
    stepper->setDirectionPin(DIR_PIN);
    stepper->setEnablePin(-1);           // Not used (EN tied to GND)
    stepper->setAutoEnable(false);
    stepper->setSpeedInHz(1200);
    stepper->setAcceleration(3000);
    Serial.println("Stepper ready");
  } else {
    Serial.println("Stepper FAILED!");
    while (1);
  }

  Serial2.begin(256000, SERIAL_8N1, RADAR_RX, RADAR_TX);
  delay(1000);
  Serial.println("=== LD2450 TURRET TRACKER ===");
}

void loop() {
  // ───── Parse incoming LD2450 frames ─────
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

  // ───── Main tracking logic ─────
  float abs_error = fabs(current_angle_error);

  if (abs_error > CENTERED_THRESHOLD) {
    // Need to move toward target
    int32_t steps = round(-current_angle_error * STEPS_PER_DEGREE);  // negative = correct direction
    Serial.printf("Moving %.1f° (%ld steps)\n", -current_angle_error, steps);

    stepper->move(steps);
    while (stepper->isRunning()) delay(1);

    // We moved → reset trigger
    trigger_active = false;
    digitalWrite(TRIGGER_PIN, LOW);

  } else if (abs_error <= CENTERED_THRESHOLD) {
    // Facing target → Pin 9 HIGH
    digitalWrite(TRIGGER_PIN, HIGH);
    if (!trigger_active) {
      trigger_active = true;
      trigger_start = millis();
      Serial.println("TARGET CENTERED → PIN 9 HIGH");
    }
  }

  // Turn off only after min time if target disappears
  if (trigger_active && (millis() - trigger_start >= MIN_HIGH_TIME)) {
    if (abs_error > 30.0f) {  // target gone
      trigger_active = false;
      digitalWrite(TRIGGER_PIN, LOW);
      Serial.println("Target lost → PIN 9 LOW");
    }
  }

  delay(20);
}

// ───── CORRECT LD2450 FRAME PARSING (OFFICIAL PROTOCOL) ─────
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

    // Skip empty targets
    if (raw_x == 0 && raw_y == 0 && raw_speed == 0 && gate == 0) continue;

    // === X COORDINATE (signed int16, MSB = sign) ===
    int16_t x_mm;
    if (g[1] & 0x80) {
      x_mm = raw_x & 0x7FFF;           // positive
    } else {
      x_mm = -(raw_x & 0x7FFF);        // negative
    }

    // === Y COORDINATE (signed int16, MSB = sign) ===
    int16_t y_mm;
    if (g[3] & 0x80) {
      y_mm = raw_y & 0x7FFF;
    } else {
      y_mm = -(raw_y & 0x7FFF);
    }

    float dist = sqrtf(x_mm * x_mm + y_mm * y_mm);
    if (dist > 300 && dist < best_dist) {  // >30cm, closest
      best_dist = dist;
      best_x_mm = x_mm;
      best_y_mm = y_mm;
      found = true;
    }
  }

  if (found && best_dist < 6000) {  // <6 meters
    float x_cm = best_x_mm / 10.0f;
    float y_cm = best_y_mm / 10.0f;
    current_angle_error = atan2f(x_cm, y_cm) * (180.0f / PI);

    Serial.printf("Target: X=%.1fcm Y=%.1fcm → Angle=%.2f°\n", x_cm, y_cm, current_angle_error);
  } else {
    current_angle_error = 0;
    Serial.println("No valid target");
  }
}