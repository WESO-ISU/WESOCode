// WESO Fallback Firmware - Sweep & Hold Hill Climber
// -----------------------------------------------------------------------
// No state machine. No e-stop. Helper functions only.
// Sweeps a grid of LA duty + load byte combos, measures stabilized RPM
// at each, locks in the best combo found and holds indefinitely.
// Press RESWEEP_BTN_PIN (pin 26) to trigger a fresh sweep at any time.
// -----------------------------------------------------------------------

#include <stdint.h>
#include <SPI.h>

// -----------------------------------------------------------------------
// PIN DEFINITIONS
// -----------------------------------------------------------------------
const int LOAD_PIN1 = 2;
const int LOAD_PIN2 = 3;
const int LOAD_PIN3 = 4;
const int LOAD_PIN4 = 5;
const int LOAD_PIN5 = 6;
const int LOAD_PIN6 = 7;
const int LOAD_PIN7 = 8;
const int LOAD_PIN8 = 9;

const int IR_PIN             = 15;
const int LA_PIN             = 0;
const int LA_EXTEND_BTN_PIN  = 24;
const int LA_RETRACT_BTN_PIN = 25;
const int RESWEEP_BTN_PIN    = 26;
const int LA_GO_HOME_BTN_PIN = 27;

const int LOAD_PINS[8] = {
    LOAD_PIN8, LOAD_PIN7, LOAD_PIN6, LOAD_PIN5,
    LOAD_PIN4, LOAD_PIN3, LOAD_PIN2, LOAD_PIN1
};

// -----------------------------------------------------------------------
// LA CONSTANTS - constrained by optocouplers
// -----------------------------------------------------------------------
const int LA_DUTY_EXTEND  = 3805;
const int LA_DUTY_RETRACT = 3880;
const int LA_DUTY_HOME    = 3825;
const int LA_DUTY_MIN     = LA_DUTY_EXTEND;
const int LA_DUTY_MAX     = LA_DUTY_RETRACT;
const int LA_DUTY_STEP    = 1;

// -----------------------------------------------------------------------
// SWEEP PARAMETERS
// #FLAG - coarser steps = faster sweep, lower resolution. Tune after first run.
// At RPM_STABLE_TIME_MS=1500, ~15 LA pts x 16 load pts = ~6 min full sweep.
// -----------------------------------------------------------------------
const int LA_SWEEP_STEP   = 5;   // Steps across LA_DUTY_MIN..LA_DUTY_MAX (~15 pts)
const int LOAD_SWEEP_STEP = 16;  // Steps across 0..255 load byte (~16 pts)

// -----------------------------------------------------------------------
// TUNABLE CONSTANTS
// -----------------------------------------------------------------------
const float RPM_MIN_THRESHOLD      = 50.0f;   //#FLAG minimum meaningful RPM to begin sweep
const float RPM_MARGIN             = 25.0f;   //#FLAG RPM stability window
const unsigned long RPM_STABLE_MS  = 1500;    //#FLAG ms RPM must hold before sampling
const unsigned long SETTLE_MS      = 200;     // Extra settle after each LA step
const bool LOAD_ACTIVE_HIGH        = true;    //#FLAG confirm with EE team

// -----------------------------------------------------------------------
// GLOBALS
// -----------------------------------------------------------------------
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;

int     la_current_duty   = LA_DUTY_HOME;
uint8_t current_load_byte = 0;
int     loadArray[8]      = {0, 0, 0, 0, 0, 0, 0, 0};

int     best_la_duty   = LA_DUTY_HOME;
uint8_t best_load_byte = 0;
float   best_rpm       = 0.0f;
bool    sweep_running  = false;

// -----------------------------------------------------------------------
// IR INTERRUPT
// FALLING edge - optically isolated signal is inverted.
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
// LA CONTROL
// -----------------------------------------------------------------------
void la_write(int duty) {
    duty = constrain(duty, LA_DUTY_MIN, LA_DUTY_MAX);
    analogWriteResolution(12);
    analogWriteFrequency(LA_PIN, 50);
    analogWrite(LA_PIN, duty);
    la_current_duty = duty;
}

// Blocking ramp to target - used in setup and between sweep LA steps
void la_move_to(int target) {
    target = constrain(target, LA_DUTY_MIN, LA_DUTY_MAX);
    int step = (target > la_current_duty) ? LA_DUTY_STEP : -LA_DUTY_STEP;
    while (la_current_duty != target) {
        la_write(la_current_duty + step);
        delay(10);
    }
}

// -----------------------------------------------------------------------
// LOAD CONTROL
// -----------------------------------------------------------------------
void set_load(uint8_t value) {
    current_load_byte = value;
    for (int i = 0; i < 8; i++) {
        loadArray[i] = (value >> i) & 1;
        bool activate = (loadArray[i] == 1);
        digitalWrite(LOAD_PINS[i], (activate == LOAD_ACTIVE_HIGH) ? HIGH : LOW);
    }
}

// -----------------------------------------------------------------------
// RPM STABILITY
// -----------------------------------------------------------------------
bool is_rpm_stable() {
    static float         last_rpm     = 0.0f;
    static unsigned long stable_since = 0;
    float rpm = get_rpm();
    if (abs(rpm - last_rpm) > RPM_MARGIN) {
        stable_since = millis();
    }
    last_rpm = rpm;
    return (millis() - stable_since >= RPM_STABLE_MS);
}

// Blocking wait for RPM to stabilize
void wait_for_stable_rpm() {
    while (!is_rpm_stable()) { delay(50); }
}

// -----------------------------------------------------------------------
// SWEEP
// Tries every (LA duty, load byte) combo on a coarse grid.
// Records the combo that produced the highest RPM.
// -----------------------------------------------------------------------
void run_sweep() {
    sweep_running  = true;
    best_rpm       = 0.0f;
    best_la_duty   = LA_DUTY_HOME;
    best_load_byte = 0;

    Serial.println("===== SWEEP START =====");

    for (int la = LA_DUTY_MIN; la <= LA_DUTY_MAX; la += LA_SWEEP_STEP) {
        // Zero load before moving LA to avoid stress at unknown combos
        set_load(0);
        la_move_to(la);
        delay(SETTLE_MS);

        for (int load = 0; load <= 255; load += LOAD_SWEEP_STEP) {
            set_load((uint8_t)load);
            wait_for_stable_rpm();

            float rpm = get_rpm();
            Serial.print("LA: ");    Serial.print(la);
            Serial.print(" Load: "); Serial.print(load);
            Serial.print(" RPM: ");  Serial.println(rpm);

            if (rpm > best_rpm) {
                best_rpm       = rpm;
                best_la_duty   = la;
                best_load_byte = (uint8_t)load;
                Serial.print("  ^ New best! RPM="); Serial.print(best_rpm);
                Serial.print(" LA=");               Serial.print(best_la_duty);
                Serial.print(" Load=");             Serial.println(best_load_byte);
            }
        }
    }

    // Lock in best combo
    set_load(0);
    la_move_to(best_la_duty);
    set_load(best_load_byte);

    Serial.println("===== SWEEP COMPLETE =====");
    Serial.print("Best LA duty:   "); Serial.println(best_la_duty);
    Serial.print("Best load byte: "); Serial.println(best_load_byte);
    Serial.print("Best RPM:       "); Serial.println(best_rpm);
    Serial.println("==========================");

    sweep_running = false;
}

// -----------------------------------------------------------------------
// MANUAL BUTTON HANDLING - only active while holding (not during sweep)
// -----------------------------------------------------------------------
void handle_manual_buttons() {
    if (sweep_running) return;

    if (digitalRead(LA_EXTEND_BTN_PIN) == LOW && la_current_duty - LA_DUTY_STEP >= LA_DUTY_MIN) {
        la_write(la_current_duty - LA_DUTY_STEP);
        Serial.print("LA duty: "); Serial.println(la_current_duty);
        delay(100);
    }
    if (digitalRead(LA_RETRACT_BTN_PIN) == LOW && la_current_duty + LA_DUTY_STEP <= LA_DUTY_MAX) {
        la_write(la_current_duty + LA_DUTY_STEP);
        Serial.print("LA duty: "); Serial.println(la_current_duty);
        delay(100);
    }
    if (digitalRead(LA_GO_HOME_BTN_PIN) == LOW) {
        la_write(LA_DUTY_HOME);
        Serial.println("LA going home.");
        delay(50);
    }
}

// -----------------------------------------------------------------------
// SETUP
// -----------------------------------------------------------------------
void setup() {
    Serial.begin(9600);

    pinMode(IR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IR_PIN), ir_interrupt, FALLING);

    pinMode(LA_PIN, OUTPUT);
    la_write(LA_DUTY_HOME);
    delay(15);

    pinMode(LA_EXTEND_BTN_PIN,  INPUT_PULLUP);
    pinMode(LA_RETRACT_BTN_PIN, INPUT_PULLUP);
    pinMode(LA_GO_HOME_BTN_PIN, INPUT_PULLUP);
    pinMode(RESWEEP_BTN_PIN,    INPUT_PULLUP);

    for (int i = 0; i < 8; i++) {
        pinMode(LOAD_PINS[i], OUTPUT);
        digitalWrite(LOAD_PINS[i], LOAD_ACTIVE_HIGH ? LOW : HIGH);
    }

    // Wait for turbine to spin up before sweeping
    Serial.println("WESO Fallback Ready. Waiting for spin-up...");
    while (get_rpm() < RPM_MIN_THRESHOLD) {
        Serial.print("RPM: "); Serial.println(get_rpm());
        delay(500);
    }
    Serial.println("Turbine spinning - starting initial sweep.");

    run_sweep();
}

// -----------------------------------------------------------------------
// LOOP
// Holds the best combo found. Triggers a fresh sweep on button press.
// -----------------------------------------------------------------------
void loop() {
    // Check resweep button (debounced edge detect)
    static bool last_resweep_btn = HIGH;
    bool resweep_btn = digitalRead(RESWEEP_BTN_PIN);
    if (last_resweep_btn == HIGH && resweep_btn == LOW) {
        Serial.println("Resweep triggered.");
        run_sweep();
    }
    last_resweep_btn = resweep_btn;

    handle_manual_buttons();

    // Hold best combo and print status every 2s
    la_write(best_la_duty);
    set_load(best_load_byte);

    static unsigned long last_print = 0;
    if (millis() - last_print >= 2000) {
        Serial.print("HOLD - LA: ");   Serial.print(best_la_duty);
        Serial.print(" Load: ");        Serial.print(best_load_byte);
        Serial.print(" RPM: ");         Serial.println(get_rpm());
        last_print = millis();
    }
}
