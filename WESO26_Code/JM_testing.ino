// ============================================================
// RPM Tachometer - Teensy 4.1 + IR Slotted Optocoupler
// Logs CSV data to SD card with auto-incremented filename
// ============================================================

#include <SD.h>
#include <SPI.h>

const int IR_PIN = 2;
const int SD_CS_PIN = BUILTIN_SDCARD;   // Teensy 4.1 built-in SD slot
const float PULSES_PER_REV = 1.0;

// Timing
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;
volatile bool newPulse = false;

// RPM calculation
float rpm = 0.0;
float filteredRPM = 0.0;
const float ALPHA = 0.2;

// Timeout
const unsigned long RPM_TIMEOUT_MS = 2000;

// Display/log interval
const unsigned long LOG_INTERVAL_MS = 250;
unsigned long lastLogTime = 0;

// SD file
File logFile;
char fileName[20];

// ============================================================
// Find next available filename: RUN_001.csv, RUN_002.csv ...
// ============================================================
void getNextFileName() {
  int runNumber = 1;
  while (runNumber < 1000) {
    snprintf(fileName, sizeof(fileName), "RUN_%03d.CSV", runNumber);
    if (!SD.exists(fileName)) break;  // Found an unused name
    runNumber++;
  }
}

// ============================================================
// INTERRUPT SERVICE ROUTINE
// ============================================================
void IRAM_ATTR onPulse() {
  unsigned long now = micros();
  if (lastPulseTime > 0) {
    pulseInterval = now - lastPulseTime;
    newPulse = true;
  }
  lastPulseTime = now;
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  // --- Init SD card ---
  Serial.print("Initializing SD card... ");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("FAILED. Check SD card and try again.");
    while (true);   // Halt — no point running without SD
  }
  Serial.println("OK.");

  // --- Find a fresh filename ---
  getNextFileName();
  Serial.print("Logging to: ");
  Serial.println(fileName);

  // --- Open file and write CSV header ---
  logFile = SD.open(fileName, FILE_WRITE);
  if (!logFile) {
    Serial.println("Error opening file! Halting.");
    while (true);
  }
  logFile.println("Timestamp_ms,RPM,Filtered_RPM");
  logFile.flush();

  // --- IR sensor ---
  pinMode(IR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IR_PIN), onPulse, FALLING);

  Serial.println("Timestamp_ms,RPM,Filtered_RPM");  // Mirror header to Serial
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  unsigned long now = millis();

  // --- Calculate RPM ---
  if (newPulse) {
    noInterrupts();
    unsigned long interval = pulseInterval;
    newPulse = false;
    interrupts();

    if (interval > 0) {
      rpm = 60000000.0 / (interval * PULSES_PER_REV);
      filteredRPM = (ALPHA * rpm) + ((1.0 - ALPHA) * filteredRPM);
    }
  }

  // --- Zero RPM if shaft stopped ---
  noInterrupts();
  unsigned long lastPulse = lastPulseTime;
  interrupts();

  if (lastPulse > 0 && (micros() - lastPulse) > (RPM_TIMEOUT_MS * 1000UL)) {
    rpm = 0.0;
    filteredRPM = 0.0;
    lastPulseTime = 0;
  }

  // --- Log at set interval ---
  if (now - lastLogTime >= LOG_INTERVAL_MS) {
    lastLogTime = now;

    // Build the CSV row once, write to both destinations
    char row[40];
    snprintf(row, sizeof(row), "%lu,%.1f,%.1f", now, rpm, filteredRPM);

    // Write to SD
    if (logFile) {
      logFile.println(row);
      logFile.flush();      // Flush every write — safe but slower.
                            // Move flush() outside this block and call
                            // every ~1s if you need higher throughput.
    }

    // Mirror to Serial Monitor
    Serial.println(row);
  }
}