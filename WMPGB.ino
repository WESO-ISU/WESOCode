/*
 * WESO Wind Turbine — MINIMAL POWER GENERATION BUILD
 * Teensy 4.1
 *
 * Goal: spin the turbine, dump power into the load resistors, and stop safely
 *       when the E-Stop fires. Nothing else.
 *
 * What this build does:
 *   1. On startup, drives the LA to LA_DUTY_HOME.
 *   2. Sets the load array to a fixed default (so the generator has something
 *      to push current into — without a load, no useful power is generated).
 *   3. Continuously polls the E-Stop line. If triggered:
 *        - Slew the LA to E_STOP_DUTY.
 *        - Zero out the load array (open circuit).
 *        - Latch in E-Stop forever (requires a reset to recover).
 *   4. Reads RPM via interrupt and prints it to Serial for sanity-checking.
 *
 * What this build does NOT do (intentionally):
 *   - State machine, lookup tables, hill climber, load adjustment
 *   - SD logging, threading
 *   - Voltage/current sensing (LMP92064)
 *   - Manual override buttons (extend/retract/go home/record)
 */

// ─── Pin Definitions ────────────────────────────────────────────────────────
const int LOAD_PIN1       = 2;
const int LOAD_PIN2       = 3;
const int LOAD_PIN3       = 4;
const int LOAD_PIN4       = 5;
const int LOAD_PIN5       = 6;
const int LOAD_PIN6       = 7;
const int LOAD_PIN7       = 8;
const int LOAD_PIN8       = 9;

const int IR_PIN          = 15;
const int LA_PIN          = 0;
const int ESTOP_WRITE_PIN = 22;
const int ESTOP_READ_PIN  = 23;

// ─── Linear Actuator Constants ──────────────────────────────────────────────
// LA is driven by direct PWM (analogWrite) at 12-bit resolution.
// Values are post-optical-isolation duty cycles.
// Lower value = more extended, higher value = more retracted.
const int LA_PWM_RESOLUTION = 12;       // bits — gives 0..4095 duty range
const int LA_PWM_FREQUENCY  = 20000;    // Hz — #FLAG confirm with EE team

const int LA_DUTY_EXTEND    = 3835;     // Duty cycle for full extend (lower value)
const int LA_DUTY_RETRACT   = 3891;     // Duty cycle for full retract (higher value)
const int LA_DUTY_HOME      = 3840;     // Home position
const int LA_DUTY_STEP      = 1;        // Duty cycle step size //#FLAG tune after mechanical testing

// #FLAG — confirm safe E-Stop duty with mechanical team before competition.
const int E_STOP_DUTY       = LA_DUTY_HOME;  // Placeholder — should be a duty
                                             // that pitches blades out of the wind.

// ─── Load Array Constants ───────────────────────────────────────────────────
// LOAD_ACTIVE_HIGH: writing HIGH activates a resistor when true, LOW when false.
// #FLAG — confirm polarity with EE team.
const bool LOAD_ACTIVE_HIGH = true;

// Index 0 = MSB = LOAD_PIN8 (largest resistor weight).
const int LOAD_PINS[8] = {
    LOAD_PIN8, LOAD_PIN7, LOAD_PIN6, LOAD_PIN5,
    LOAD_PIN4, LOAD_PIN3, LOAD_PIN2, LOAD_PIN1
};

// #FLAG — pick the right starting load for your generator.
// 0x00 = all resistors off (no load → no useful power, turbine freewheels).
// 0xFF = all resistors on (lowest resistance → max load on the generator).
// Starting value below is a midrange placeholder; tune for your hardware.
const uint8_t DEFAULT_LOAD_BYTE = 0xF0;

// ─── Globals ────────────────────────────────────────────────────────────────
int la_current_duty       = LA_DUTY_HOME;
uint8_t loadArray[8]      = {0};

// RPM interrupt state
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;

// Latch — once E-Stop fires, we stay stopped until reset.
bool eStopLatched = false;

// ─── ISR: IR sensor pulse ───────────────────────────────────────────────────
void ir_interrupt() {
    unsigned long now = micros();
    if (now - lastPulseTime < 10000) return;  // 10ms debounce
    pulseInterval = now - lastPulseTime;
    lastPulseTime = now;
}

// ─── Load Array Helpers ─────────────────────────────────────────────────────
void loadLoadArray(uint8_t loadValue) {
    // Index 0 = MSB
    for (int i = 0; i < 8; i++) {
        loadArray[i] = (loadValue >> (7 - i)) & 1;
    }
}

void loadLoad() {
    for (int i = 0; i < 8; i++) {
        bool activate = (loadArray[i] == 1);
        digitalWrite(LOAD_PINS[i], (activate == LOAD_ACTIVE_HIGH) ? HIGH : LOW);
    }
}

// ─── E-Stop ─────────────────────────────────────────────────────────────────
bool is_estop_triggered() {
    // E-Stop write pin is held HIGH; the read pin reads HIGH when the loop is
    // closed and LOW when the E-Stop button breaks the circuit.
    return digitalRead(ESTOP_READ_PIN) == LOW;
}

void eStop() {
    Serial.println("!!! E-STOP TRIGGERED !!!");

    // 1. Drop the load (open circuit, no current path).
    loadLoadArray(0x00);
    loadLoad();

    // 2. Slew the LA to safe duty.
    if (la_current_duty < E_STOP_DUTY) {
        for (int d = la_current_duty; d <= E_STOP_DUTY; d += LA_DUTY_STEP) {
            analogWrite(LA_PIN, d);
            delay(25);
        }
    } else {
        for (int d = la_current_duty; d >= E_STOP_DUTY; d -= LA_DUTY_STEP) {
            analogWrite(LA_PIN, d);
            delay(25);
        }
    }
    analogWrite(LA_PIN, E_STOP_DUTY);  // ensure exact final value
    la_current_duty = E_STOP_DUTY;

    // 3. Latch — never come back without a reset.
    eStopLatched = true;
    Serial.println("E-Stop latched. Reset board to recover.");
}

// ─── RPM Read (for monitoring only) ─────────────────────────────────────────
float get_rpm() {
    if (pulseInterval == 0) return 0.0f;
    if (micros() - lastPulseTime > 2000000) return 0.0f;  // stale → 0
    float revTime_sec = pulseInterval / 1000000.0f;
    return 60.0f / revTime_sec;
}

// ════════════════════════════════════════════════════════════════════════════
// SETUP
// ════════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(9600);
    while (!Serial && millis() < 3000) {}

    // Load pins — initialize all OFF.
    for (int i = 0; i < 8; i++) {
        pinMode(LOAD_PINS[i], OUTPUT);
        digitalWrite(LOAD_PINS[i], LOAD_ACTIVE_HIGH ? LOW : HIGH);
    }

    // E-Stop pins.
    pinMode(ESTOP_WRITE_PIN, OUTPUT);
    digitalWrite(ESTOP_WRITE_PIN, HIGH);
    pinMode(ESTOP_READ_PIN, INPUT);

    // IR sensor interrupt.
    pinMode(IR_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(IR_PIN), ir_interrupt, FALLING);

    // Linear actuator → home (direct PWM).
    analogWriteResolution(LA_PWM_RESOLUTION);
    analogWriteFrequency(LA_PIN, LA_PWM_FREQUENCY);
    pinMode(LA_PIN, OUTPUT);
    analogWrite(LA_PIN, LA_DUTY_HOME);
    la_current_duty = LA_DUTY_HOME;
    delay(500);  // give it a moment to settle

    // Apply the default load so the generator has something to push into.
    loadLoadArray(DEFAULT_LOAD_BYTE);
    loadLoad();

    Serial.println("WESO minimal build ready. Generating with default load.");
    Serial.print("Default load byte: 0x");
    Serial.println(DEFAULT_LOAD_BYTE, HEX);
}

// ════════════════════════════════════════════════════════════════════════════
// LOOP
// ════════════════════════════════════════════════════════════════════════════
void loop() {
    // E-Stop has highest priority and is checked every iteration.
    if (!eStopLatched && is_estop_triggered()) {
        eStop();
    }

    // Once latched, do nothing else. Board must be reset to recover.
    if (eStopLatched) {
        delay(100);
        return;
    }

    // Heartbeat — print RPM once per second so we can confirm it's alive.
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {
        Serial.print("RPM: ");
        Serial.println(get_rpm(), 1);
        lastPrint = millis();
    }
}
