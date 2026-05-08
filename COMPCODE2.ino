#include <Servo.h>
#include <SD.h>
#include <TeensyThreads.h>
#include <stdint.h>
#include <SPI.h>

const int LOAD_PIN1 = 2;
const int LOAD_PIN2 = 3;
const int LOAD_PIN3 = 4;
const int LOAD_PIN4 = 5;
const int LOAD_PIN5 = 6;
const int LOAD_PIN6 = 7;
const int LOAD_PIN7 = 8;
const int LOAD_PIN8 = 9;

const int CS1_PIN         = 10;
const int SCK_PIN         = 13;
const int MOSI_PIN        = 11;
const int MISO_PIN        = 12;

const int IR_PIN          = 1;
const int LA_PIN          = 0;
const int ESTOP_READ_PIN  = 15;

const int LA_EXTEND_BTN_PIN   = 24;
const int LA_RETRACT_BTN_PIN  = 25;
const int RECORD_BTN_PIN      = 26;
const int LA_GO_HOME_BTN_PIN  = 27;

// LA is controlled via analogWrite at 12-bit resolution (0-4095) at 50Hz.
// Working duty cycle range confirmed from hardware testing: 3816 (retract) to 3891 (extend).
// #FLAG - fine tune LA_DUTY_RETRACT and LA_DUTY_EXTEND bounds after mechanical testing.
const int LA_DUTY_EXTEND  = 3835;  // Duty cycle for full retract
const int LA_DUTY_RETRACT   = 3891;  // Duty cycle for full extend
const int LA_DUTY_HOME     = 3835;  // Midpoint
const int LA_DUTY_STEP     = 1;     // Duty cycle step size //#FLAG tune after mechanical testing

// #FLAG - Confirm safe e-stop duty cycle with EE/mechanical team before competition.
const int E_STOP_DUTY      = 3891; // PLACEHOLDER - verify before competition

const int BUFFER_SIZE        = 50;
const int FLUSH_INTERVAL_MS  = 5000;
const int SAMPLE_INTERVAL_MS = 100;

// Set to true if HIGH activates a resistor, false if LOW activates.
// Load is NOT optically isolated, so no inversion needed here.
const bool LOAD_ACTIVE_HIGH = true; //#FLAG confirm with EE team before competition

const int LOAD_PINS[8] = {
    LOAD_PIN8, LOAD_PIN7, LOAD_PIN6, LOAD_PIN5,
    LOAD_PIN4, LOAD_PIN3, LOAD_PIN2, LOAD_PIN1
};

int loadArray[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// LOOKUP TABLE

struct LookupEntry {
    float   wind_speed;       // m/s
    int     la_duty;          // target LA duty cycle for this wind speed
    float   resistance_ohms;  // human-readable resistance value
    uint8_t load_byte;        // raw value to pass to loadLoadArray()
    float   rpm_spike_delta;  // RPM jump threshold for load disconnect detection at this wind speed
};

// Derived from tunnel testing data - best wattage point per wind speed.
// Pitch (la_duty) values mapped from raw pitch (microseconds) into confirmed duty range using:
//   la_duty = map(raw_pitch, 1000, 2000, LA_DUTY_RETRACT, LA_DUTY_EXTEND)
//   5 m/s:  map(1515, 1000, 2000, 3816, 3891) = 3855
//   6 m/s:  map(1485, 1000, 2000, 3816, 3891) = 3852
//   7 m/s:  map(1470, 1000, 2000, 3816, 3891) = 3851
//   8 m/s:  map(1470, 1000, 2000, 3816, 3891) = 3851
//   9 m/s:  map(1435, 1000, 2000, 3816, 3891) = 3849
//  10 m/s:  map(1430, 1000, 2000, 3816, 3891) = 3848
// 11-13 m/s mirror 10 m/s as starting point; state_maintain_wattage() walks load to hold power.
// #FLAG - rpm_spike_delta values of 200.0f are initial estimates. Tune after competition testing.
const int LOOKUP_TABLE_SIZE = 9;
const LookupEntry lookupTable[LOOKUP_TABLE_SIZE] = {
    // wind_speed, la_duty, resistance_ohms, load_byte, rpm_spike_delta
    {  5.0f,       3868,    1280.0f,         0x80,      200.0f }, //#FLAG tune spike delta
    {  6.0f,       3874,    88.0f,           0xF0,      200.0f }, //#FLAG tune spike delta
    {  7.0f,       3877,    15.1f,           0xF8,      200.0f }, //#FLAG tune spike delta
    {  8.0f,       3883,    15.1f,           0xF8,      200.0f }, //#FLAG tune spike delta
    {  9.0f,       3884,    13.29f,          0xD6,      200.0f }, //#FLAG tune spike delta
    { 10.0f,       3889,    11.04f,          0x7F,      200.0f }, //#FLAG tune spike delta
    { 11.0f,       3889,    11.04f,          0x7F,      200.0f }, // mirrors 10 m/s //#FLAG tune spike delta
    { 12.0f,       3889,    11.04f,          0x7F,      200.0f }, // mirrors 10 m/s //#FLAG tune spike delta
    { 13.0f,       3889,    11.04f,          0x7F,      200.0f }, // mirrors 10 m/s //#FLAG tune spike delta
};

// TUNABLE CONSTANTS

const float RPM_MAX_THRESHOLD            = 3000.0f;  //#FLAG verify safe max RPM with mechanical team
const float RPM_MIN_THRESHOLD            = 50.0f;    //#FLAG verify minimum meaningful spinning RPM
const float RPM_MARGIN                   = 15.0f;    //#FLAG tune stability window after testing
const unsigned long RPM_STABLE_TIME_MS   = 1500;     //#FLAG tune - how long RPM must hold before we act
const float HILL_CLIMB_LOAD_RPM_MIN      = 100.0f;   //#FLAG tune - RPM floor during load hill climbing
const int   HILL_CLIMB_LA_STEP           = 1;        //#FLAG tune - smaller = smoother but slower hill climb
const int   HILL_CLIMB_LOAD_STEP         = 1;        //#FLAG tune - smaller = finer load resolution
const int   HILL_CLIMB_ITERATIONS        = 2;        //#FLAG tune - more iterations = more optimization time
const float VOLTAGE_SOFT_LIMIT           = 24.0f;    // NOTE: unused until LMP sensors restored
const float VOLTAGE_HARD_LIMIT           = 30.0f;    // NOTE: unused until LMP sensors restored
const int   VOLTAGE_LA_STEP              = 2;        // NOTE: unused until LMP sensors restored
const float RPM_WIND_INCREMENT_THRESHOLD = 20.0f;    //#FLAG tune - RPM rise that signals 1 m/s wind increase
const float WIND_SPEED_POWER_HOLD        = 10.0f;    // Switch to maintain-wattage mode at this wind speed
const float WIND_SPEED_MAX               = 13.0f;    // Competition maximum wind speed
const float WIND_SPEED_INCREMENT         = 1.0f;     // Competition increases by 1 m/s at a time

// Target wattage at 10 m/s from tunnel data (V²/R = 15.65²/11.04 = 22.18W)
const float TARGET_POWER_WATTS_10MS      = 22.18f;   //#FLAG recalculate if better 10 m/s data point found

int   la_current_duty = LA_DUTY_HOME;

volatile unsigned long lastPulseTime     = 0;
volatile unsigned long pulseInterval     = 0;
volatile unsigned long prevPulseInterval = 0;

// Load disconnect flag - set in ISR, checked and acted on in loop()
volatile bool load_disconnected = false;

// Current lookup index - kept volatile so ir_interrupt() can safely read it
volatile int current_lookup_index = 0;

volatile bool is_recording = false;
char current_filename[20];

float    estimated_wind_speed = 5.0f;
float    target_power_watts   = TARGET_POWER_WATTS_10MS;
uint8_t  current_load_byte    = 0;
int      hill_climb_iteration = 0;

struct DataPoint {
    unsigned long timestamp;
    float rpm;
    float wind_speed;
    int   la_duty;
    uint8_t load_byte;
};

DataPoint buffer_A[BUFFER_SIZE];
DataPoint buffer_B[BUFFER_SIZE];

DataPoint* write_buffer = buffer_A;
DataPoint* flush_buffer = buffer_B;

volatile int  write_index = 0;
Threads::Mutex buffer_mutex;

enum State {
    STATE_ESTOP,
    STATE_STARTUP,
    STATE_LOOKUP,
    STATE_HILL_CLIMB_PITCH,
    STATE_HILL_CLIMB_LOAD,
    STATE_IDLE,
    STATE_MAINTAIN_WATTAGE
};

State current_state = STATE_STARTUP;

// IR interrupt - handles both RPM measurement and load disconnect detection.
// NOTE: Using FALLING edge because IR line is optically isolated (signal is inverted).
void ir_interrupt() {
    unsigned long now = micros();

    if (now - lastPulseTime < 10000) return;

    prevPulseInterval = pulseInterval;
    pulseInterval     = now - lastPulseTime;
    lastPulseTime     = now;

    if (prevPulseInterval > 0 && pulseInterval < prevPulseInterval) {
        float prev_rpm = 60.0f / (prevPulseInterval / 1000000.0f);
        float curr_rpm = 60.0f / (pulseInterval     / 1000000.0f);
        float spike    = curr_rpm - prev_rpm;

        float threshold = lookupTable[current_lookup_index].rpm_spike_delta;
        if (spike > threshold) {
            load_disconnected = true;
        }
    }
}

// Core LA write - all actuator motion goes through here.
void la_write(int duty) {
    duty = constrain(duty, LA_DUTY_RETRACT, LA_DUTY_EXTEND);
    analogWriteResolution(12);
    analogWriteFrequency(LA_PIN, 50);
    analogWrite(LA_PIN, duty);
    la_current_duty = duty;
}

bool is_legal_la_duty(int duty) {
    return (duty >= LA_DUTY_RETRACT && duty <= LA_DUTY_EXTEND);
}

void handle_actuator_write(int duty) {
    if (is_legal_la_duty(duty)) {
        la_write(duty);
    }
}

void handle_actuator_buttons() {
    bool extendPressed  = digitalRead(LA_EXTEND_BTN_PIN) == LOW;
    bool retractPressed = digitalRead(LA_RETRACT_BTN_PIN) == LOW;

    if (extendPressed && la_current_duty + LA_DUTY_STEP <= LA_DUTY_EXTEND) {
        la_write(la_current_duty + LA_DUTY_STEP);
        Serial.print("LA duty: "); Serial.println(la_current_duty);
        delay(100);
    } else if (retractPressed && la_current_duty - LA_DUTY_STEP >= LA_DUTY_RETRACT) {
        la_write(la_current_duty - LA_DUTY_STEP);
        Serial.print("LA duty: "); Serial.println(la_current_duty);
        delay(100);
    }
}

void handle_la_go_home() {
    if (digitalRead(LA_GO_HOME_BTN_PIN) == LOW) {
        la_write(LA_DUTY_HOME);
        delay(50);
        Serial.println("LA going home.");
    }
}

void handle_estop_button() {
    static bool last_btn_state = HIGH;
    bool btn_state = digitalRead(ESTOP_READ_PIN);
    if (last_btn_state == HIGH && btn_state == LOW) {
        Serial.println("E-stop button pressed!");
        current_state = STATE_ESTOP;
    }
    last_btn_state = btn_state;
}

void eStop() {
    // Gradually step LA to e-stop duty cycle to avoid sudden mechanical shock
    if (la_current_duty < E_STOP_DUTY) {
        for (int i = la_current_duty; i < E_STOP_DUTY; i++) {
            la_write(i);
            delay(25);
        }
    } else {
        for (int i = la_current_duty; i > E_STOP_DUTY; i--) {
            la_write(i);
            delay(25);
        }
    }
}

void setup() {
    Serial.begin(9600);

    // IR sensor - FALLING because optical isolation inverts the signal
    pinMode(IR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IR_PIN), ir_interrupt, FALLING);

    // Linear actuator - initialize via analogWrite
    pinMode(LA_PIN, OUTPUT);
    la_write(LA_DUTY_HOME);
    delay(15);

    // Button inputs
    pinMode(LA_EXTEND_BTN_PIN,  INPUT_PULLUP);
    pinMode(LA_RETRACT_BTN_PIN, INPUT_PULLUP);
    pinMode(LA_GO_HOME_BTN_PIN, INPUT_PULLUP);
    pinMode(RECORD_BTN_PIN,     INPUT_PULLUP);
    pinMode(ESTOP_READ_PIN,     INPUT_PULLUP);

    setup_sd();
    setup_loads();

    threads.addThread(sampling_thread);
    threads.addThread(flush_thread);

    Serial.println("WESO Competition Firmware Ready.");
}

void setup_sd() {
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD init failed.");
        return;
    }
    Serial.println("SD ready.");
}

void setup_loads() {
    for (int i = 0; i < 8; i++) {
        pinMode(LOAD_PINS[i], OUTPUT);
        digitalWrite(LOAD_PINS[i], LOAD_ACTIVE_HIGH ? LOW : HIGH);
    }
}

void loadLoadArray(uint8_t loadValue) {
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

float get_rpm() {
    noInterrupts();
    unsigned long interval  = pulseInterval;
    unsigned long lastPulse = lastPulseTime;
    interrupts();
    if (interval == 0) return 0.0f;
    if (micros() - lastPulse > 2000000) return 0.0f;
    return 60.0f / (interval / 1000000.0f);
}

float get_windspeed() {
    return estimated_wind_speed;
}

void update_wind_speed() {
    static float baseline_rpm = 0.0f;
    static bool  baseline_set = false;
    float rpm = get_rpm();
    if (!baseline_set) {
        baseline_rpm = rpm;
        baseline_set = true;
        return;
    }
    if (rpm - baseline_rpm > RPM_WIND_INCREMENT_THRESHOLD) {
        if (estimated_wind_speed < WIND_SPEED_MAX) {
            estimated_wind_speed += WIND_SPEED_INCREMENT;
            Serial.print("Wind speed updated to: ");
            Serial.println(estimated_wind_speed);
        }
        baseline_set = false;
    }
}

void update_lookup_index() {
    for (int i = 0; i < LOOKUP_TABLE_SIZE - 1; i++) {
        if (estimated_wind_speed <= lookupTable[i].wind_speed) {
            current_lookup_index = i;
            return;
        }
    }
    current_lookup_index = LOOKUP_TABLE_SIZE - 1;
}

int lookup_la_duty(float wind_speed) {
    if (wind_speed <= lookupTable[0].wind_speed)
        return lookupTable[0].la_duty;
    if (wind_speed >= lookupTable[LOOKUP_TABLE_SIZE - 1].wind_speed)
        return lookupTable[LOOKUP_TABLE_SIZE - 1].la_duty;
    for (int i = 0; i < LOOKUP_TABLE_SIZE - 1; i++) {
        if (wind_speed >= lookupTable[i].wind_speed &&
            wind_speed <= lookupTable[i+1].wind_speed) {
            float t = (wind_speed - lookupTable[i].wind_speed) /
                      (lookupTable[i+1].wind_speed - lookupTable[i].wind_speed);
            return (int)(lookupTable[i].la_duty +
                         t * (lookupTable[i+1].la_duty - lookupTable[i].la_duty));
        }
    }
    return LA_DUTY_HOME;
}

uint8_t lookup_load_byte(float wind_speed) {
    if (wind_speed <= lookupTable[0].wind_speed)
        return lookupTable[0].load_byte;
    if (wind_speed >= lookupTable[LOOKUP_TABLE_SIZE - 1].wind_speed)
        return lookupTable[LOOKUP_TABLE_SIZE - 1].load_byte;
    for (int i = 0; i < LOOKUP_TABLE_SIZE - 1; i++) {
        if (wind_speed >= lookupTable[i].wind_speed &&
            wind_speed <= lookupTable[i+1].wind_speed) {
            float t = (wind_speed - lookupTable[i].wind_speed) /
                      (lookupTable[i+1].wind_speed - lookupTable[i].wind_speed);
            float interp_resistance = lookupTable[i].resistance_ohms +
                        t * (lookupTable[i+1].resistance_ohms - lookupTable[i].resistance_ohms);
            int   best      = 0;
            float best_diff = abs(lookupTable[0].resistance_ohms - interp_resistance);
            for (int j = 1; j < LOOKUP_TABLE_SIZE; j++) {
                float diff = abs(lookupTable[j].resistance_ohms - interp_resistance);
                if (diff < best_diff) { best_diff = diff; best = j; }
            }
            return lookupTable[best].load_byte;
        }
    }
    return 0;
}

bool should_estop() {
    if (digitalRead(ESTOP_READ_PIN) == LOW) return true;
    if (get_rpm() > RPM_MAX_THRESHOLD)      return true;
    return false;
}

bool is_rpm_stable() {
    static float         last_rpm     = 0.0f;
    static unsigned long stable_since = 0;
    float rpm = get_rpm();
    if (abs(rpm - last_rpm) > RPM_MARGIN) {
        stable_since = millis();
    }
    last_rpm = rpm;
    return (millis() - stable_since >= RPM_STABLE_TIME_MS);
}

void state_estop() {
    eStop();
    Serial.println("E-STOP");
    float rpm = get_rpm();
    if (digitalRead(ESTOP_READ_PIN) == HIGH && rpm < RPM_MAX_THRESHOLD) {
        load_disconnected = false;
        current_state = STATE_STARTUP;
    }
}

void state_startup() {
    if (should_estop()) { current_state = STATE_ESTOP; return; }

    estimated_wind_speed  = 5.0f;
    hill_climb_iteration  = 0;
    current_load_byte     = 0;
    target_power_watts    = TARGET_POWER_WATTS_10MS;
    loadLoadArray(0);
    loadLoad();
    update_lookup_index();

    float rpm = get_rpm();
    Serial.print("STARTUP - RPM: "); Serial.println(rpm);

    if (rpm > RPM_MIN_THRESHOLD) {
        Serial.println("Turbine spinning, transitioning to LOOKUP.");
        current_state = STATE_LOOKUP;
    }
}

void state_lookup() {
    if (should_estop()) { current_state = STATE_ESTOP; return; }

    float   wind        = get_windspeed();
    float   rpm         = get_rpm();
    int     target_duty = lookup_la_duty(wind);
    uint8_t target_load = lookup_load_byte(wind);

    Serial.print("LOOKUP - Wind: "); Serial.print(wind);
    Serial.print(" RPM: ");          Serial.print(rpm);
    Serial.print(" Target duty: ");  Serial.print(target_duty);
    Serial.print(" Target load: ");  Serial.println(target_load);

    if (la_current_duty != target_duty) {
        int step = (target_duty > la_current_duty) ? HILL_CLIMB_LA_STEP : -HILL_CLIMB_LA_STEP;
        handle_actuator_write(la_current_duty + step);
        return;
    }

    if (!is_rpm_stable()) return;

    if (current_load_byte != target_load) {
        if (current_load_byte < target_load) current_load_byte += HILL_CLIMB_LOAD_STEP;
        else                                 current_load_byte -= HILL_CLIMB_LOAD_STEP;
        loadLoadArray(current_load_byte);
        loadLoad();
        return;
    }

    if (!is_rpm_stable()) return;

    Serial.println("Lookup complete, starting hill climb.");
    hill_climb_iteration = 0;
    current_state = STATE_HILL_CLIMB_PITCH;
}

void state_hill_climb_pitch() {
    if (should_estop()) { current_state = STATE_ESTOP; return; }

    float rpm = get_rpm();
    Serial.print("HILL CLIMB PITCH - RPM: "); Serial.println(rpm);

    if (rpm >= RPM_MAX_THRESHOLD) {
        Serial.println("Max RPM reached, transitioning to HILL CLIMB LOAD.");
        current_state = STATE_HILL_CLIMB_LOAD;
        return;
    }

    if (!is_rpm_stable()) return;

    if (la_current_duty + HILL_CLIMB_LA_STEP <= LA_DUTY_EXTEND) {
        handle_actuator_write(la_current_duty + HILL_CLIMB_LA_STEP);
    } else {
        Serial.println("LA at extend limit, transitioning to HILL CLIMB LOAD.");
        current_state = STATE_HILL_CLIMB_LOAD;
    }
}

void state_hill_climb_load() {
    if (should_estop()) { current_state = STATE_ESTOP; return; }

    float rpm = get_rpm();
    Serial.print("HILL CLIMB LOAD - RPM: "); Serial.print(rpm);
    Serial.print(" Load byte: "); Serial.println(current_load_byte);

    if (!is_rpm_stable()) return;

    if (rpm < HILL_CLIMB_LOAD_RPM_MIN) {
        if (current_load_byte > 0) {
            current_load_byte -= HILL_CLIMB_LOAD_STEP;
            loadLoadArray(current_load_byte);
            loadLoad();
            Serial.println("RPM too low, backing off load.");
        } else {
            Serial.println("Load at zero, RPM still low. E-stopping.");
            current_state = STATE_ESTOP;
        }
        return;
    }

    if (current_load_byte < 255) {
        current_load_byte += HILL_CLIMB_LOAD_STEP;
        loadLoadArray(current_load_byte);
        loadLoad();
        return;
    }

    hill_climb_iteration++;
    Serial.print("Hill climb iteration complete: "); Serial.println(hill_climb_iteration);

    if (hill_climb_iteration < HILL_CLIMB_ITERATIONS) {
        current_state = STATE_HILL_CLIMB_PITCH;
    } else {
        Serial.println("All hill climb iterations done. Transitioning to IDLE.");
        current_state = STATE_IDLE;
    }
}

void state_idle() {
    if (should_estop()) { current_state = STATE_ESTOP; return; }

    float wind = get_windspeed();
    Serial.print("IDLE - Estimated wind: "); Serial.println(wind);

    if (wind >= WIND_SPEED_POWER_HOLD) {
        Serial.println("Switching to MAINTAIN WATTAGE.");
        current_state = STATE_MAINTAIN_WATTAGE;
        return;
    }

    float prev_wind = estimated_wind_speed;
    update_wind_speed();
    update_lookup_index();
    if (estimated_wind_speed > prev_wind) {
        Serial.println("Wind speed increased, re-optimizing via LOOKUP.");
        hill_climb_iteration = 0;
        current_state = STATE_LOOKUP;
    }
}

void state_maintain_wattage() {
    if (should_estop()) { current_state = STATE_ESTOP; return; }

    float rpm = get_rpm();
    Serial.print("MAINTAIN - RPM: "); Serial.println(rpm);

    if (!is_rpm_stable()) return;

    // #FLAG - replace TARGET_RPM with average of best 10 m/s RPM readings after competition testing.
    const float TARGET_RPM      = 1741.0f;  //#FLAG verify after competition testing
    const float RPM_HOLD_MARGIN = 20.0f;    //#FLAG tune after testing

    float rpm_error = rpm - TARGET_RPM;

    if (rpm_error > RPM_HOLD_MARGIN) {
        if (current_load_byte < 255) {
            current_load_byte++;
            loadLoadArray(current_load_byte);
            loadLoad();
            Serial.println("RPM high, increasing load.");
        }
    } else if (rpm_error < -RPM_HOLD_MARGIN) {
        if (current_load_byte > 0) {
            current_load_byte--;
            loadLoadArray(current_load_byte);
            loadLoad();
            Serial.println("RPM low, decreasing load.");
        }
    }
}

void sampling_thread() {
    while (1) {
        if (is_recording && write_index < BUFFER_SIZE) {
            DataPoint dp;
            dp.timestamp  = millis();
            dp.rpm        = get_rpm();
            dp.wind_speed = get_windspeed();
            dp.la_duty    = la_current_duty;
            dp.load_byte  = current_load_byte;

            buffer_mutex.lock();
            write_buffer[write_index++] = dp;
            buffer_mutex.unlock();
        }
        threads.delay(SAMPLE_INTERVAL_MS);
    }
}

void flush_thread() {
    while (1) {
        threads.delay(FLUSH_INTERVAL_MS);
        if (is_recording) {
            buffer_mutex.lock();
            DataPoint* temp = write_buffer;
            write_buffer    = flush_buffer;
            flush_buffer    = temp;
            int count       = write_index;
            write_index     = 0;
            buffer_mutex.unlock();

            File f = SD.open(current_filename, FILE_WRITE);
            if (f) {
                for (int i = 0; i < count; i++) {
                    f.print(flush_buffer[i].timestamp);  f.print(",");
                    f.print(flush_buffer[i].rpm);        f.print(",");
                    f.print(flush_buffer[i].wind_speed); f.print(",");
                    f.print(flush_buffer[i].la_duty);    f.print(",");
                    f.println(flush_buffer[i].load_byte);
                }
                f.close();
                Serial.print("Flushed "); Serial.print(count); Serial.println(" records.");
            } else {
                Serial.println("SD write failed.");
            }
        }
    }
}

void handle_record_button() {
    static bool last_btn_state = HIGH;
    bool btn_state = digitalRead(RECORD_BTN_PIN);
    if (last_btn_state == HIGH && btn_state == LOW) {
        if (!is_recording) {
            snprintf(current_filename, sizeof(current_filename), "log_%lu.csv", millis());
            File f = SD.open(current_filename, FILE_WRITE);
            if (f) {
                f.println("timestamp_ms,rpm,wind_speed_ms,la_duty,load_byte");
                f.close();
                is_recording = true;
                Serial.print("Recording started: "); Serial.println(current_filename);
            } else {
                Serial.println("Failed to create log file.");
            }
        } else {
            is_recording = false;
            Serial.print("Recording stopped: "); Serial.println(current_filename);
        }
    }
    last_btn_state = btn_state;
}

void loop() {
    if (load_disconnected) {
        Serial.println("Load disconnect detected via RPM spike! E-stopping.");
        current_state = STATE_ESTOP;
        load_disconnected = false;
    }

    handle_record_button();
    handle_actuator_buttons();
    handle_la_go_home();
    handle_estop_button();

    switch (current_state) {
        case STATE_ESTOP:            state_estop();            break;
        case STATE_STARTUP:          state_startup();          break;
        case STATE_LOOKUP:           state_lookup();           break;
        case STATE_HILL_CLIMB_PITCH: state_hill_climb_pitch(); break;
        case STATE_HILL_CLIMB_LOAD:  state_hill_climb_load();  break;
        case STATE_IDLE:             state_idle();             break;
        case STATE_MAINTAIN_WATTAGE: state_maintain_wattage(); break;
    }
}
