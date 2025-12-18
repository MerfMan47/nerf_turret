#include <Arduino.h>

#define RX_PIN 5
#define TX_PIN 4

HardwareSerial radarSerial(1);

// Frame header and footer
const uint8_t FRAME_HEADER[] = {0xAA, 0xFF, 0x03, 0x00};
const uint8_t FRAME_FOOTER[] = {0x55, 0xCC};

struct Target {
  int16_t x;
  int16_t y;
  int16_t speed;
  uint16_t resolution;
};

void setup() {
  Serial.begin(115200);
  radarSerial.begin(256000, SERIAL_8N1, RX_PIN, TX_PIN);
  
  Serial.println("LD2450 Radar Tracker");
  Serial.println("Waiting for data...");
}

void loop() {
  static uint8_t buf[256];
  static int bufIdx = 0;
  
  while (radarSerial.available()) {
    uint8_t b = radarSerial.read();
    buf[bufIdx++] = b;
    
    // Prevent overflow
    if (bufIdx >= 256) bufIdx = 0;
    
    // Check for frame header
    if (bufIdx >= 4) {
      bool headerFound = true;
      for (int i = 0; i < 4; i++) {
        if (buf[bufIdx - 4 + i] != FRAME_HEADER[i]) {
          headerFound = false;
          break;
        }
      }
      
      if (headerFound) {
        // Wait for complete frame (4 header + 29 data + 2 footer = 35 bytes)
        if (bufIdx >= 35) {
          // Check footer
          if (buf[bufIdx - 2] == FRAME_FOOTER[0] && buf[bufIdx - 1] == FRAME_FOOTER[1]) {
            parseFrame(&buf[bufIdx - 35]);
            bufIdx = 0;
          }
        }
      }
    }
  }
}

void parseFrame(uint8_t* frame) {
  // Skip header (4 bytes)
  uint8_t* data = frame + 4;
  
  // Get target count
  uint8_t targetCount = data[0];
  
  Serial.println("\n--- Radar Data ---");
  Serial.print("Targets: ");
  Serial.println(targetCount);
  
  // Parse up to 3 targets
  for (int i = 0; i < 3 && i < targetCount; i++) {
    Target t;
    int offset = 1 + (i * 8);
    
    t.x = (int16_t)(data[offset] | (data[offset + 1] << 8));
    t.y = (int16_t)(data[offset + 2] | (data[offset + 3] << 8));
    t.speed = (int16_t)(data[offset + 4] | (data[offset + 5] << 8));
    t.resolution = (uint16_t)(data[offset + 6] | (data[offset + 7] << 8));
    
    Serial.print("Target ");
    Serial.print(i + 1);
    Serial.print(": X=");
    Serial.print(t.x);
    Serial.print("mm, Y=");
    Serial.print(t.y);
    Serial.print("mm, Speed=");
    Serial.print(t.speed);
    Serial.print("mm/s, Res=");
    Serial.println(t.resolution);
  }
}