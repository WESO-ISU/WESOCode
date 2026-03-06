/*
 * WESO Wind Turbine Controller
 * Teensy 4.1
 * Main state machine file
 *
 * Pinout (https://www.pjrc.com/teensy/pinout.html):
 *   Pin 0  - PWM output for Linear Actuator (LA)
 *   Pin 1  - IR sensor input (RPM)
 *   Pin 2-9- Load resistance digital outputs
 *   Pin 10 - SPI CS
 *   Pin 11 - SPI MOSI
 *   Pin 12 - SPI MISO
 *   Pin 13 - SPI SCK
 *   Pin 22 - E-Stop input A
 *   Pin 23 - E-Stop input B
 *   Pin 37 - SPI extra
 *   I2C    - Wind speed sensor (via optical isolation)
 */

#include <SD.h>
#include <Servo.h>
#include <Wire.h>
#include "data_logger.h"

// ─── Pin Definitions ────────────────────────────────────────────────────────
#define LA_PWM_PIN       0
#define RPM_PIN          1
#define LOAD_PIN_START   2
#define LOAD_PIN_END     9
#define SPI_CS_PIN       10
#define ESTOP_PIN_A      22
#define ESTOP_PIN_B      23

// ─── Thresholds & Tuning ────────────────────────────────────────────────────
#define RPM_MIN_SAFE         260    // Do NOT decrease load R below this RPM
#define RPM_MAX_SAFE        1660    // DO decrease load R (add load) at this RPM
#define RPM_HIGH_TARGET     1600    // High wind target RPM
#define RPM_HIGH_APPROACH   1500    // Increase load when we hit this
#define RPM_LOW_TARGET       950    // Low wind target RPM
#define RPM_LOW_APPROACH     900    // Increase load when we hit this
#define RPM_DROP_THRESHOLD   200    // RPM drop that triggers Hill Climber
#define RPM_WAY_TOO_LOW      150    // RPM drop that triggers E-Stop from Startup
#define WIND_CHANGE_THRESHOLD 0.5f  // m/s delta that triggers state change
#define LOAD_CHANGE_WAIT_MS  2000   // ms to wait after a load change
#define HILL_CLIMBER_WAIT_MS 3000   // ms to give turbine time to recover
#define ACTIVE_READ_INTERVAL  500   // ms between active readings
#define IDLE_READ_INTERVAL   1000   // ms between idle readings
#define LA_ESTOP_ANGLE         0    // LA angle for E-Stop (feathered)
#define LA_DEFAULT_ANGLE      90    // LA starting angle
#define LA_STEP                5    // degrees to adjust LA per step
#define LA_MIN_ANGLE           0
#define LA_MAX_ANGLE         180
#define NUM_LOAD_PINS          8    // Pins 2-9
#define MAX_LOAD_STATE       255    // 8 bits

// ─── State Enum ─────────────────────────────────────────────────────────────
typedef enum {
  STATE_ESTOP,
  STATE_STARTUP,
  STATE_ACTIVE_READING,
  STATE_LOAD_CHANGE,
  STATE_HILL_CLIMBER,
  STATE_IDLE
} SystemState;

// ─── Globals ────────────────────────────────────────────────────────────────
SystemState currentState = STATE_ESTOP;
Servo linearActuator;

volatile unsigned long rpmPulseCount = 0;
volatile unsigned long lastPulseTime = 0;
int    laAngle        = LA_DEFAULT_ANGLE;
int    loadState      = 0;          // bitmask across pins 2-9
float  lastWindSpeed  = 0.0f;
float  lastRPM        = 0.0f;
unsigned long stateEntryTime = 0;

// ─── ISR: RPM counter ───────────────────────────────────────────────────────
void IRAM_ATTR rpmISR() {
  rpmPulseCount++;
  lastPulseTime = millis();
}

// ═══════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}   // wait up to 3s for Serial

  // Load pins
  for (int p = LOAD_PIN_START; p <= LOAD_PIN_END; p++) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }

  // E-Stop pins
  pinMode(ESTOP_PIN_A, INPUT_PULLUP);
  pinMode(ESTOP_PIN_B, INPUT_PULLUP);

  // RPM pin
  pinMode(RPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpmISR, RISING);

  // Linear actuator
  linearActuator.attach(LA_PWM_PIN);
  linearActuator.write(LA_DEFAULT_ANGLE);

  // I2C for wind sensor
  Wire.begin();

  // SD card & logger
  logger_init(SPI_CS_PIN);

  // Print CSV header to serial & SD
  logger_print_header();

  // Enter E-Stop on boot until we confirm safe
  enter_estop();
}

// ═══════════════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════
void loop() {
  // Always check E-Stop first
  if (is_estop_triggered()) {
    if (currentState != STATE_ESTOP) enter_estop();
  }

  switch (currentState) {
    case STATE_ESTOP:         run_estop();          break;
    case STATE_STARTUP:       run_startup();        break;
    case STATE_ACTIVE_READING:run_active_reading(); break;
    case STATE_LOAD_CHANGE:   run_load_change();    break;
    case STATE_HILL_CLIMBER:  run_hill_climber();   break;
    case STATE_IDLE:          run_idle();           break;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  E-STOP STATE
// ═══════════════════════════════════════════════════════════════════════════
void enter_estop() {
  currentState = STATE_ESTOP;
  stateEntryTime = millis();
  set_la_angle(LA_ESTOP_ANGLE);
  set_load(0);   // disconnect load
  Serial.println("# [STATE] E-STOP entered");
}

void run_estop() {
  float rpm       = get_rpm();
  float windspeed = get_windspeed();
  float voltage   = get_voltage();
  float current   = get_current();
  bool  load      = detect_load();

  logger_record(rpm, windspeed, voltage, current, loadState, laAngle, "ESTOP");

  // Stay in E-Stop — operator must reset or conditions must clear
  // (Extend: add a reset button check here)
}

// ═══════════════════════════════════════════════════════════════════════════
//  STARTUP STATE
// ═══════════════════════════════════════════════════════════════════════════
void enter_startup() {
  currentState = STATE_STARTUP;
  stateEntryTime = millis();
  set_load(0);   // start with max resistance (no load pins active)
  set_la_angle(LA_DEFAULT_ANGLE);
  Serial.println("# [STATE] STARTUP entered");
}

void run_startup() {
  float rpm       = get_rpm();
  float windspeed = get_windspeed();
  float voltage   = get_voltage();
  float current   = get_current();
  bool  load      = detect_load();

  logger_record(rpm, windspeed, voltage, current, loadState, laAngle, "STARTUP");

  // No load detected → go to E-Stop
  if (!load) {
    enter_estop();
    return;
  }

  // RPM way too low → E-Stop
  if (rpm > 0 && rpm < RPM_WAY_TOO_LOW) {
    enter_estop();
    return;
  }

  // Wind speed present → move to Active Reading
  if (windspeed > 0.5f && rpm > RPM_MIN_SAFE) {
    lastWindSpeed = windspeed;
    enter_active_reading();
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  ACTIVE READING STATE
// ═══════════════════════════════════════════════════════════════════════════
void enter_active_reading() {
  currentState = STATE_ACTIVE_READING;
  stateEntryTime = millis();
  Serial.println("# [STATE] ACTIVE READING entered");
}

void run_active_reading() {
  static unsigned long lastReadTime = 0;
  if (millis() - lastReadTime < ACTIVE_READ_INTERVAL) return;
  lastReadTime = millis();

  float rpm       = get_rpm();
  float windspeed = get_windspeed();
  float voltage   = get_voltage();
  float current   = get_current();
  bool  load      = detect_load();

  logger_record(rpm, windspeed, voltage, current, loadState, laAngle, "ACTIVE");

  if (!load) { enter_estop(); return; }

  // Significant wind speed change → Load Change State
  if (fabs(windspeed - lastWindSpeed) > WIND_CHANGE_THRESHOLD) {
    lastWindSpeed = windspeed;
    enter_load_change();
    return;
  }

  // RPM dropped too low → Hill Climber
  if (rpm < (lastRPM - RPM_DROP_THRESHOLD) && rpm < RPM_LOW_TARGET) {
    enter_hill_climber();
    return;
  }

  // RPM approaching high target → bump load
  if (rpm >= RPM_HIGH_APPROACH) {
    enter_load_change();
    return;
  }

  // RPM approaching low target → bump load
  if (rpm >= RPM_LOW_APPROACH && rpm < RPM_HIGH_APPROACH) {
    enter_load_change();
    return;
  }

  // Adjust LA toward target
  adjust_la_for_rpm(rpm, windspeed);

  lastRPM = rpm;
  lastWindSpeed = windspeed;
}

// ═══════════════════════════════════════════════════════════════════════════
//  LOAD CHANGE STATE
// ═══════════════════════════════════════════════════════════════════════════
void enter_load_change() {
  currentState = STATE_LOAD_CHANGE;
  stateEntryTime = millis();
  Serial.println("# [STATE] LOAD CHANGE entered");
}

void run_load_change() {
  float rpm       = get_rpm();
  float windspeed = get_windspeed();
  float voltage   = get_voltage();
  float current   = get_current();
  bool  load      = detect_load();

  logger_record(rpm, windspeed, voltage, current, loadState, laAngle, "LOAD_CHG");

  if (!load) { enter_estop(); return; }

  // Adjust load based on RPM and windspeed
  change_load_resistance(rpm, windspeed);

  // Wait for system to settle
  if (millis() - stateEntryTime < LOAD_CHANGE_WAIT_MS) return;

  // Re-read after wait
  rpm = get_rpm();

  // RPM at or above maximum target → Idle
  if (rpm >= RPM_HIGH_TARGET) {
    enter_idle();
    return;
  }

  // Wind speed changed again → stay in Load Change (reset timer)
  if (fabs(windspeed - lastWindSpeed) > WIND_CHANGE_THRESHOLD) {
    lastWindSpeed = windspeed;
    stateEntryTime = millis();
    return;
  }

  // RPMs met target → Active Reading
  if (rpm >= RPM_LOW_TARGET && rpm < RPM_HIGH_TARGET) {
    enter_active_reading();
    return;
  }

  lastWindSpeed = windspeed;
  lastRPM = rpm;
}

// ═══════════════════════════════════════════════════════════════════════════
//  HILL CLIMBER STATE
// ═══════════════════════════════════════════════════════════════════════════
void enter_hill_climber() {
  currentState = STATE_HILL_CLIMBER;
  stateEntryTime = millis();
  Serial.println("# [STATE] HILL CLIMBER entered");

  // Back LA off slightly to let turbine recover
  set_la_angle(laAngle + LA_STEP);
}

void run_hill_climber() {
  float rpm       = get_rpm();
  float windspeed = get_windspeed();
  float voltage   = get_voltage();
  float current   = get_current();
  bool  load      = detect_load();

  logger_record(rpm, windspeed, voltage, current, loadState, laAngle, "HILL_CLB");

  if (!load) { enter_estop(); return; }

  // Give turbine time to recover
  if (millis() - stateEntryTime < HILL_CLIMBER_WAIT_MS) return;

  // Wind speed changed → Load Change
  if (fabs(windspeed - lastWindSpeed) > WIND_CHANGE_THRESHOLD) {
    lastWindSpeed = windspeed;
    enter_load_change();
    return;
  }

  // Timer expired, check RPM against target
  if (rpm >= RPM_LOW_TARGET) {
    enter_active_reading();
  } else {
    // Still struggling — back off LA more and reset timer
    set_la_angle(laAngle + LA_STEP);
    stateEntryTime = millis();
  }

  lastRPM = rpm;
  lastWindSpeed = windspeed;
}

// ═══════════════════════════════════════════════════════════════════════════
//  IDLE STATE
// ═══════════════════════════════════════════════════════════════════════════
void enter_idle() {
  currentState = STATE_IDLE;
  stateEntryTime = millis();
  Serial.println("# [STATE] IDLE entered");
}

void run_idle() {
  static unsigned long lastReadTime = 0;
  if (millis() - lastReadTime < IDLE_READ_INTERVAL) return;
  lastReadTime = millis();

  float rpm       = get_rpm();
  float windspeed = get_windspeed();
  float voltage   = get_voltage();
  float current   = get_current();
  bool  load      = detect_load();

  logger_record(rpm, windspeed, voltage, current, loadState, laAngle, "IDLE");

  if (!load) { enter_estop(); return; }

  // Wind speed changed with threshold → Active Reading
  if (fabs(windspeed - lastWindSpeed) > WIND_CHANGE_THRESHOLD) {
    lastWindSpeed = windspeed;
    enter_active_reading();
    return;
  }

  lastWindSpeed = windspeed;
  lastRPM = rpm;
}

// ═══════════════════════════════════════════════════════════════════════════
//  SENSOR FUNCTIONS
// ═══════════════════════════════════════════════════════════════════════════

/*
 * get_rpm()
 * Reads pulse count from RPM ISR over a 100ms window.
 * Assumes 1 pulse per revolution (adjust PULSES_PER_REV if needed).
 */
float get_rpm() {
  const float PULSES_PER_REV = 1.0f;
  const unsigned long SAMPLE_MS = 100;

  noInterrupts();
  unsigned long count = rpmPulseCount;
  rpmPulseCount = 0;
  interrupts();

  delay(SAMPLE_MS);

  noInterrupts();
  unsigned long newCount = rpmPulseCount;
  rpmPulseCount = 0;
  interrupts();

  float revs = (float)(newCount) / PULSES_PER_REV;
  float rpm  = revs * (60000.0f / SAMPLE_MS);
  return rpm;
}

/*
 * get_windspeed()
 * Reads pitot tube pressure differential via I2C (after optical isolation).
 * For March 12-13 bench testing, reads analog voltage directly.
 * Equation: v = sqrt(2 * deltaP / rho_air)
 *   deltaP = (Vout / Vref) * P_max   [sensor-specific, adjust P_max]
 *   rho_air ~ 1.225 kg/m^3
 */
float get_windspeed() {
  // ── Bench test mode: direct analog read ──────────────────────────────
  // Uncomment below for March 12-13 testing, comment out I2C block
  /*
  int raw = analogRead(A0);           // Replace A0 with your actual analog pin
  float vout = raw * (3.3f / 1023.0f);
  float vref = 3.3f;
  float P_max = 1000.0f;             // Pa — adjust to your sensor's spec
  float rho = 1.225f;
  float deltaP = (vout / vref) * P_max;
  if (deltaP <= 0) return 0.0f;
  return sqrt(2.0f * deltaP / rho);
  */

  // ── I2C mode (optical isolation path) ────────────────────────────────
  // TODO: Replace 0x28 with your sensor's actual I2C address
  Wire.requestFrom(0x28, 2);
  if (Wire.available() < 2) return lastWindSpeed; // return last known if no data

  uint16_t raw = ((uint16_t)Wire.read() << 8) | Wire.read();
  // Mask status bits if needed (sensor-dependent)
  raw &= 0x3FFF;

  float vref  = 3.3f;
  float P_max = 1000.0f;  // Pa — adjust to your sensor's spec
  float rho   = 1.225f;

  float deltaP = ((float)raw / 16383.0f) * P_max;
  if (deltaP <= 0.0f) return 0.0f;
  return sqrt(2.0f * deltaP / rho);
}

/*
 * get_voltage()
 * Reads turbine output voltage via analog pin.
 * Adjust VOLTAGE_DIVIDER_RATIO to match your resistor divider on the ADC input.
 */
float get_voltage() {
  const float VOLTAGE_DIVIDER_RATIO = 11.0f;  // e.g. 10k+1k divider
  int raw = analogRead(A1);                    // Replace with actual pin
  float vADC = raw * (3.3f / 1023.0f);
  return vADC * VOLTAGE_DIVIDER_RATIO;
}

/*
 * get_current()
 * Reads current via a current sensor (e.g. ACS712) on an analog pin.
 * Adjust sensitivity and offset to match your sensor.
 */
float get_current() {
  const float SENSITIVITY  = 0.185f;   // V/A for ACS712 5A version
  const float ZERO_CURRENT = 1.65f;    // Vout at 0A (Vcc/2 for 3.3V)
  int raw = analogRead(A2);            // Replace with actual pin
  float vSensor = raw * (3.3f / 1023.0f);
  return (vSensor - ZERO_CURRENT) / SENSITIVITY;
}

/*
 * detect_load()
 * Compares turbine voltage to load voltage to determine if a load is present.
 * Returns true if load is detected.
 */
bool detect_load() {
  float turbineV = get_voltage();
  // TODO: Add second ADC read for load-side voltage once PCB is available
  // For now, use a simple threshold
  return (turbineV > 0.5f);
}

/*
 * is_estop_triggered()
 * Returns true if either E-Stop pin is pulled LOW (active-low with pullup).
 */
bool is_estop_triggered() {
  return (digitalRead(ESTOP_PIN_A) == LOW || digitalRead(ESTOP_PIN_B) == LOW);
}

// ═══════════════════════════════════════════════════════════════════════════
//  ACTUATOR & LOAD CONTROL
// ═══════════════════════════════════════════════════════════════════════════

/*
 * set_la_angle()
 * Clamps and writes angle to the linear actuator servo.
 */
void set_la_angle(int angle) {
  laAngle = constrain(angle, LA_MIN_ANGLE, LA_MAX_ANGLE);
  linearActuator.write(laAngle);
}

/*
 * adjust_la_for_rpm()
 * Moves LA in small steps to track the RPM target zone.
 */
void adjust_la_for_rpm(float rpm, float windspeed) {
  int targetRPM = (windspeed > 6.0f) ? RPM_HIGH_TARGET : RPM_LOW_TARGET;

  if (rpm < targetRPM - 50) {
    set_la_angle(laAngle - LA_STEP);   // pitch into wind more
  } else if (rpm > targetRPM + 50) {
    set_la_angle(laAngle + LA_STEP);   // feather slightly
  }
}

/*
 * set_load()
 * Writes a bitmask to pins 2-9 to set load resistance.
 * Each pin switches in a parallel resistor. Higher bitmask = lower resistance.
 */
void set_load(int mask) {
  loadState = constrain(mask, 0, MAX_LOAD_STATE);
  for (int i = 0; i < NUM_LOAD_PINS; i++) {
    digitalWrite(LOAD_PIN_START + i, (loadState >> i) & 0x01);
  }
}

/*
 * change_load_resistance()
 * Decides whether to increase or decrease load based on current RPM.
 * High wind: want lower R (more load pins ON) to approach 1600 RPM.
 * Low wind:  want higher R (fewer load pins ON) to approach 950 RPM.
 */
int change_load_resistance(float rpm, float windspeed) {
  // Safety floor: never lower resistance below 260 RPM
  if (rpm < RPM_MIN_SAFE) {
    set_load(0);  // open circuit / max R
    return loadState;
  }

  // Safety ceiling: must lower resistance at 1660 RPM
  if (rpm >= RPM_MAX_SAFE) {
    set_load(min(loadState + 1, MAX_LOAD_STATE));
    return loadState;
  }

  if (windspeed > 6.0f) {
    // High wind → target 1600 RPM
    if (rpm >= RPM_HIGH_APPROACH) {
      // Close to target, bump load (lower R)
      set_load(min(loadState + 1, MAX_LOAD_STATE));
    } else if (rpm < RPM_HIGH_TARGET - 100) {
      // Too far below, back off load (raise R)
      set_load(max(loadState - 1, 0));
    }
  } else {
    // Low wind → target 950 RPM
    if (rpm >= RPM_LOW_APPROACH) {
      set_load(min(loadState + 1, MAX_LOAD_STATE));
    } else if (rpm < RPM_LOW_TARGET - 100) {
      set_load(max(loadState - 1, 0));
    }
  }

  return loadState;
}
