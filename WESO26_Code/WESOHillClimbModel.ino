#include <Servo.h>

// === PINS ===
const int voltagePin = A0;      // Simulated voltage (potentiometer)
const int currentPin = A1;      // Simulated current (potentiometer)
const int estopPin = 4;         // E-stop button
const int actuatorPin = 5;      // Servo for pitch control

// === ACTUATOR ===
Servo actuator;
int actuatorPosition = 56;      // Start at mid-range
const int actuatorMin = 47;     // Fully loaded (max power extraction)
const int actuatorMax = 71;     // Fully feathered (min load)

// === HILL CLIMB PARAMETERS ===
const int stepSize = 1;
const unsigned long updateInterval = 300; // Faster updates
unsigned long lastUpdateTime = 0;

float lastPower = 0.0;
bool increasingPitch = true;

const float powerThreshold = 0.05;  // Smaller threshold = more sensitive

// === SMOOTHING ===
const int numReadings = 3;
float voltageReadings[numReadings] = {0};
float currentReadings[numReadings] = {0};
int readIndex = 0;

// === SAFETY ===
const float maxVoltage = 15.0;
bool estopEngaged = false;

// === FUNCTION DECLARATIONS ===
float readVoltage();
float readCurrent();

void setup() {
  Serial.begin(115200);
  
  pinMode(estopPin, INPUT_PULLUP);
  
  actuator.attach(actuatorPin);
  actuator.write(actuatorPosition);
  
  Serial.println("=== Wind Turbine Power Optimization Simulator ===");
  Serial.println("Adjust potentiometers to simulate changing conditions");
  Serial.println();
}

void loop() {
  unsigned long currentTime = millis();
  
  // === READ SENSORS (Simulated) ===
  float voltage = readVoltage();
  float current = readCurrent();
  float power = voltage * current;
  
  // === SAFETY CHECKS ===
  if (digitalRead(estopPin) == LOW || voltage > maxVoltage) {
    if (!estopEngaged) {
      estopEngaged = true;
      actuatorPosition = actuatorMax; // Feather blades
      actuator.write(actuatorPosition);
      Serial.println("*** E-STOP ENGAGED ***");
    }
    delay(100);
    return;
  } else {
    estopEngaged = false;
  }
  
  // === HILL CLIMB ALGORITHM ===
  if (currentTime - lastUpdateTime >= updateInterval) {
    lastUpdateTime = currentTime;
    
    // Print current state
    Serial.print("V: "); Serial.print(voltage, 2); Serial.print("V");
    Serial.print(" | I: "); Serial.print(current, 3); Serial.print("A");
    Serial.print(" | P: "); Serial.print(power, 2); Serial.print("W");
    Serial.print(" | Pitch: "); Serial.print(actuatorPosition);
    
    // Calculate power difference
    float powerDiff = power - lastPower;
    
    // Hill climbing logic with dead zone
    if (powerDiff > powerThreshold) {
      // Power INCREASED significantly - keep going same direction
      if (increasingPitch) {
        actuatorPosition += stepSize;
      } else {
        actuatorPosition -= stepSize;
      }
      Serial.print(" → UP (+"); Serial.print(powerDiff, 2); Serial.println("W)");
      
    } else if (powerDiff < -powerThreshold) {
      // Power DECREASED significantly - reverse direction
      increasingPitch = !increasingPitch;
      if (increasingPitch) {
        actuatorPosition += stepSize;
      } else {
        actuatorPosition -= stepSize;
      }
      Serial.print(" → DOWN ("); Serial.print(powerDiff, 2); Serial.println("W) REV");
      
    } else {
      // Power change too small - don't move
      Serial.print(" → HOLD ("); Serial.print(powerDiff, 2); Serial.println("W)");
    }
    
    // Constrain actuator
    actuatorPosition = constrain(actuatorPosition, actuatorMin, actuatorMax);
    actuator.write(actuatorPosition);
    
    lastPower = power;
  }
  
  delay(50);
}

// === SENSOR READING FUNCTIONS ===

float readVoltage() {
  int raw = analogRead(voltagePin);
  float voltage = (raw / 1023.0) * 15.0;
  
  // Store reading and calculate average
  voltageReadings[readIndex % numReadings] = voltage;
  
  float sum = 0;
  for (int i = 0; i < numReadings; i++) {
    sum += voltageReadings[i];
  }
  
  return sum / numReadings;
}

float readCurrent() {
  int raw = analogRead(currentPin);
  float current = (raw / 1023.0) * 3.0;
  
  // Store reading and calculate average
  currentReadings[readIndex % numReadings] = current;
  
  float sum = 0;
  for (int i = 0; i < numReadings; i++) {
    sum += currentReadings[i];
  }
  
  readIndex++; // Increment once at end
  
  return sum / numReadings;
}