#define PIN_SERIAL_TX (5) // Blue Wire conencts to RX
#define PIN_SERIAL_RX (6) // Yellow wirevconnects to TX (need to be reversed for some reason)

const byte HEADER[] = {0xAA, 0xFF, 0x03, 0x00}; // Frame header
const byte FOOTER[] = {0x55, 0xCC}; // Frame footer
const size_t DATA_LENGTH = 24;
const size_t GROUP_LENGTH = 8;

byte buffer[DATA_LENGTH];
byte currentIndex = 0;
bool frameStarted = false; // To track if header has been found

void setup() {
// Serial for debug output (USB-CDC or built-in UART)
Serial.begin(9600); 

// Serial2 for sensor communication on specified pins
// Syntax: SerialX.begin(baud, config, rxPin, txPin);
// The third and fourth parameters assign the hardware pins.
Serial2.begin(256000, SERIAL_8N1, PIN_SERIAL_RX, PIN_SERIAL_TX);
}

void loop() {
// Use Serial2 instead of Serial1
while (Serial2.available()) { // Check if Serial2 has data
byte inputByte = Serial2.read();

// Check if the current index is at the start of a new frame
if (!frameStarted) {
// Check for the start of the header
if (inputByte == HEADER[currentIndex]) {
currentIndex++;
if (currentIndex == sizeof(HEADER)) {
frameStarted = true; // Header matched, start receiving data
currentIndex = 0; // Reset index for data collection
}
} else {
// Reset if the header does not match
currentIndex = 0;
}
} else {
// Reading the 24 data bytes
if (currentIndex < DATA_LENGTH) {
buffer[currentIndex++] = inputByte;

} else if (currentIndex == DATA_LENGTH) {
// Check for footer bytes
if (inputByte == FOOTER[0]) {
currentIndex++; // Move to the next step
} else {
// Reset if footer does not match
frameStarted = false;
currentIndex = 0;
}
} else if (currentIndex == DATA_LENGTH + 1) {
if (inputByte == FOOTER[1]) {
// Complete frame detected, process data
processFrame(buffer, DATA_LENGTH);
}
// Reset the state for the next frame
frameStarted = false;
currentIndex = 0;
}
}
}
}

// --- Frame Processing Functions (Unchanged) ---
// Note: You must ensure all original functions (processFrameRaw, processFrame,
// repeatSpaces, and unpackSensorData) are included below this point.

void processFrameRaw(byte* data, size_t length) {
// Process the data as needed
Serial.println("Frame received:");
for (size_t i = 0; i < length; i++) {
Serial.print(data[i], HEX);
Serial.print(" ");
}
Serial.println();
}

void processFrame(byte* data, size_t length) {
// Validate data length
if (length != DATA_LENGTH) return;

// Split the data into three groups and unpack each
for (int i = 0; i < 3; i++) {
byte group[GROUP_LENGTH];
memcpy(group, &data[i * GROUP_LENGTH], GROUP_LENGTH);
unpackSensorData(group); // Call the function to unpack sensor data
}
}

String repeatSpaces(int count) {
String spaces = "";
for (int i = 0; i < count; i++) {
spaces += " ";
}
return spaces;
}

void unpackSensorData(byte* group) {
// Unpack 4 signed and unsigned integers from the 8-byte group
uint16_t int1 = ((group[1] & 0x7F) << 8) | group[0]; // First signed integer
uint16_t int2 = ((group[3] & 0xFF) << 8) | group[2]; // Second signed integer 
uint16_t int3 = ((group[5] & 0x7F) << 8) | group[4]; // Third signed integer
uint16_t uintInt = (group[7] << 8) | group[6]; // Fourth unsigned integer

if (int1 != 0 || int2 != 0 || int3 != 0 || uintInt != 0) {
float xdist = (int1) / 10.0;
if ((group[1] & 0x80) == 0) {
xdist = -xdist;
}
float ydist = (int2);
ydist = ydist - 0x8000;
ydist /= 10.0;
float speed = (int3);
if ((group[5] & 0x80) == 0) {
speed = -speed;
}

const int WIDTH_X = 8; // Width for X distance
const int WIDTH_Y = 8; // Width for Y distance
const int WIDTH_SPD = 8; // Width for Speed
const int WIDTH_GATE = 5; // Width for Gate

// Print the unpacked integers
Serial.print("| X: ");
Serial.print(repeatSpaces(WIDTH_X - String(xdist).length())); // Padding spaces
Serial.print(xdist);
Serial.print("cm | Y:");
Serial.print(repeatSpaces(WIDTH_Y - String(ydist).length())); // Padding spaces
Serial.print(ydist);
Serial.print("cm | Spd:");
Serial.print(repeatSpaces(WIDTH_SPD - String(speed).length())); // Padding spaces
Serial.print(speed);
Serial.print("cm/sec | Gate: ");
Serial.print(repeatSpaces(WIDTH_GATE - String(uintInt).length())); // Padding spaces
Serial.print(uintInt);
Serial.println("mm |");
}
}