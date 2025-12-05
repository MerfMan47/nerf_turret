/*
 * FastAccelStepper with 3:1 Gear Reduction
 * Stepper: 60 teeth
 * Turntable: 180 teeth
 * Ratio: 3:1 (stepper turns 3x for every 1x turntable rotation)
 */

#include "FastAccelStepper.h"

// Pin Configuration
#define STEP_PIN        2
#define DIR_PIN         7
#define ENABLE_PIN      6

// Motor and Gear Configuration
const uint16_t MICROSTEPPING      = 256;          // Your actual setting
const uint16_t STEPS_PER_REV      = 200;          // Motor native steps
const float    GEAR_RATIO         = 0.1;          // 180/60 = 3:1 reduction

// Calculate steps for turntable movement
const uint32_t STEPS_PER_MOTOR_REV = STEPS_PER_REV * MICROSTEPPING;  // 51200 steps per motor revolution
const uint32_t STEPS_PER_TABLE_REV = STEPS_PER_MOTOR_REV * GEAR_RATIO; // 153600 steps per turntable revolution

// For 90° turntable rotation (1/4 turn of turntable)
const uint32_t STEPS_90_DEGREES_TABLE = STEPS_PER_TABLE_REV / 4;  // 38400 steps

const uint32_t SPEED_HZ           = 1200;
const uint32_t ACCELERATION       = 3000;
const uint32_t PAUSE_MS           = 500;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  
  if (stepper) {
    stepper->setDirectionPin(DIR_PIN);
    stepper->setEnablePin(ENABLE_PIN);
    stepper->setAutoEnable(false);
    
    stepper->setSpeedInHz(SPEED_HZ);
    stepper->setAcceleration(ACCELERATION);
    
    Serial.println("FastAccelStepper with 3:1 Gear Reduction");
    Serial.printf("Gear Ratio: %.1f:1\n", GEAR_RATIO);
    Serial.printf("Steps per motor rev: %lu\n", STEPS_PER_MOTOR_REV);
    Serial.printf("Steps per table rev: %lu\n", STEPS_PER_TABLE_REV);
    Serial.printf("Steps for 90° table rotation: %lu\n", STEPS_90_DEGREES_TABLE);
  } else {
    Serial.println("ERROR: Stepper init failed!");
    while(1) delay(1000);
  }
  
  delay(1000);
  Serial.println("Starting 90° back-and-forth...");
}

void loop() {
  // Move turntable forward 90 degrees
  Serial.println("Table rotating +90°...");
  stepper->move(STEPS_90_DEGREES_TABLE);
  while (stepper->isRunning()) { 
    delay(10); 
  }
  
  delay(PAUSE_MS);
  
  // Move turntable backward 90 degrees
  Serial.println("Table rotating -90°...");
  stepper->move(-STEPS_90_DEGREES_TABLE);
  while (stepper->isRunning()) { 
    delay(10); 
  }
  
  delay(PAUSE_MS);
}