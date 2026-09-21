// WESO Fallback Firmware - Load Hill Climber
// -----------------------------------------------------------------------
// Fixed pitch (LA parked at home). Load-only hill climber.
// Adds load until RPM drops too low, backs off one step = peak.
// Holds peak. Re-climbs automatically if RPM rises significantly.
// -----------------------------------------------------------------------

#include <stdint.h>

// -----------------------------------------------------------------------
// PINS
// -----------------------------------------------------------------------
const int LOAD_PIN1 = 2;
const int LOAD_PIN2 = 3;
const int LOAD_PIN3 = 4;
const int LOAD_PIN4 = 5;
const int LOAD_PIN5 = 6;
const int LOAD_PIN6 = 7;
const int LOAD_PIN7 = 8;
const int LOAD_PIN8 = 9;

const int IR_PIN = 1;
const int LA_PIN = 0;

const int LOAD_PINS[8] = {
    LOAD_PIN8, LOAD_PIN7, LOAD_PIN6, LOAD_PIN5,
    LOAD_PIN4, LOAD_PIN3, LOAD_PIN2, LOAD_PIN1
};

// -----------------------------------------------------------------------
// LA - fixed, never moves after setup
// -----------------------------------------------------------------------
const int LA_DUTY_HOME = 3815;

// -----------------------------------------------------------------------
// TUNABLE CONSTANTS
// -----------------------------------------------------------------------
const float RPM_MIN_THRESHOLD  = 50.0f;   // Wait for this RPM before climbing
const float RPM_FLOOR          = 200.0f;  //#FLAG RPM floor - back off load if we drop below this
const float RPM_RISE_THRESHOLD = 45.0f;  //#FLAG RPM rise above hold RPM that triggers a re-climb
const int   LOAD_STEP          = 1;       //#FLAG load byte step size per climb iteration
const unsigned long STEP_DELAY_MS = 1000; //#FLAG ms to wait between load steps (let RPM settle)

const bool LOAD_ACTIVE_HIGH = true; //#FLAG confirm with EE team

// -----------------------------------------------------------------------
// GLOBALS
// -----------------------------------------------------------------------
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;

uint8_t current_load = 0;
float   hold_rpm     = 0.0f;

// -----------------------------------------------------------------------
// IR INTERRUPT - FALLING, optically isolated
// -----------------------------------------------------------------------
void ir_interrupt() {
    unsigned long now = micros();
    if (now - lastPulseTime < 10000) return;
    pulseInterval = now - lastPulseTime;
    lastPulseTime = now;
}

// -----------------------------------------------------------------------
// RPM
// -----------------------------------------------------------------------
float get_rpm() {
    noInterrupts();
    unsigned long interval  = pulseInterval;
    unsigned long lastPulse = lastPulseTime;
    interrupts();
    if (interval == 0) return 0.0f;
    if (micros() - lastPulse > 2000000) return 0.0f;
    return 60.0f / (interval / 1000000.0f);
}

// -----------------------------------------------------------------------
// LOAD
// -----------------------------------------------------------------------
void set_load(uint8_t value) {
    current_load = value;
    for (int i = 0; i < 8; i++) {
        bool activate = ((value >> i) & 1) == 1;
        digitalWrite(LOAD_PINS[i], (activate == LOAD_ACTIVE_HIGH) ? HIGH : LOW);
    }
    Serial.print("Load: "); Serial.print(value);
    Serial.print(" RPM: "); Serial.println(get_rpm());
}

// -----------------------------------------------------------------------
// HILL CLIMB
// Adds load one step at a time until RPM drops below RPM_FLOOR.
// Then backs off one step and records that as the hold point.
// -----------------------------------------------------------------------
void run_climb() {
    Serial.println("--- Climb start ---");

    while (true) {
        delay(STEP_DELAY_MS);
        float rpm = get_rpm();

        if (rpm < RPM_FLOOR) {
            // RPM dropped too low - back off one step
            if (current_load >= LOAD_STEP) {
                set_load(current_load - LOAD_STEP);
            }
            delay(STEP_DELAY_MS);
            hold_rpm = get_rpm();
            Serial.print("--- Peak found. Holding load="); Serial.print(current_load);
            Serial.print(" RPM="); Serial.println(hold_rpm);
            return;
        }

        if (current_load + LOAD_STEP <= 255) {
            set_load(current_load + LOAD_STEP);
        } else {
            // Hit max load - hold here
            hold_rpm = get_rpm();
            Serial.print("--- Max load reached. Holding RPM="); Serial.println(hold_rpm);
            return;
        }
    }
}

// -----------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------
void setup() {
    Serial.begin(9600);

    pinMode(IR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IR_PIN), ir_interrupt, FALLING);

    // Park LA at fixed pitch - never touched again
    pinMode(LA_PIN, OUTPUT);
    analogWriteResolution(12);
    analogWriteFrequency(LA_PIN, 50);
    analogWrite(LA_PIN, LA_DUTY_HOME);
    delay(15);

    for (int i = 0; i < 8; i++) {
        pinMode(LOAD_PINS[i], OUTPUT);
        digitalWrite(LOAD_PINS[i], LOAD_ACTIVE_HIGH ? LOW : HIGH);
    }

    // Wait for spin-up
    Serial.println("Waiting for spin-up...");
    while (get_rpm() < RPM_MIN_THRESHOLD) {
        Serial.print("RPM: "); Serial.println(get_rpm());
        delay(500);
    }
    Serial.println("Spinning - starting climb.");

    run_climb();
}

// -----------------------------------------------------------------------
// LOOP
// Holds peak load. Re-climbs if RPM rises significantly (wind picked up).
// -----------------------------------------------------------------------
void loop() {
    delay(500);
    float rpm = get_rpm();

    // Re-climb if RPM has risen well above our hold point
    if (rpm > hold_rpm + RPM_RISE_THRESHOLD) {
        Serial.print("RPM rose to "); Serial.print(rpm);
        Serial.println(" - re-climbing.");
        run_climb();
        return;
    }

    Serial.print("HOLD - Load: "); Serial.print(current_load);
    Serial.print(" RPM: ");        Serial.println(rpm);
}
