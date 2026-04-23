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
const int CS2_PIN         = 37;  // LMP92064 chip select
const int SCK_PIN         = 13;
const int MOSI_PIN        = 11;
const int MISO_PIN        = 12;

const int IR_PIN          = 15;
const int LA_PIN          = 0;
const int ESTOP_READ_PIN  = 23;

const int LA_EXTEND_BTN_PIN   = 24;
const int LA_RETRACT_BTN_PIN  = 25;
const int RECORD_BTN_PIN      = 26;
const int LA_GO_HOME_BTN_PIN  = 27;

const int LA_EXTEND       = 120;  // Hard extend limit
const int LA_RETRACT      = 80;   // Hard retract limit
const int LA_HOME_POSITION = 102; // Home/default position
const int LA_STEP_SIZE    = 1;    // Default step size
const int E_STOP_POSITION = 83;   // Safe position during e-stop - verify with EE team

const int          BUFFER_SIZE       = 50;
const int          FLUSH_INTERVAL_MS = 5000;
const int          SAMPLE_INTERVAL_MS = 100;

const float LMP_VREF       = 2.048f;
const float LMP_ADC_COUNTS = 4096.0f;
const float R_SENSE        = 0.020f;   // 20mΩ - confirm with EE team (must be < 25mΩ for 3A max)

// Voltage divider - update with actual PCB values from EE team
const float LMP_VDIV_R2     = 1600.0f;
const float LMP_VDIV_R3     = 46400.0f;
const float LMP_VDIV_FACTOR = (LMP_VDIV_R2 + LMP_VDIV_R3) / LMP_VDIV_R2;

const uint16_t LMP_REG_VOUT_LSB = 0x0200;
const uint16_t LMP_REG_VOUT_MSB = 0x0201;
const uint16_t LMP_REG_COUT_LSB = 0x0202;
const uint16_t LMP_REG_COUT_MSB = 0x0203;

// Set to true if HIGH activates a resistor, false if LOW activates
const bool LOAD_ACTIVE_HIGH = true;

const int LOAD_PINS[8] = {
    LOAD_PIN8, LOAD_PIN7, LOAD_PIN6, LOAD_PIN5,
    LOAD_PIN4, LOAD_PIN3, LOAD_PIN2, LOAD_PIN1
};

int loadArray[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // Default all off

// LOOKUP TABLE

struct LookupEntry {
    float   wind_speed;      // m/s
    int     la_position;     // target LA position for this wind speed
    float   resistance_ohms; // human-readable resistance value
    uint8_t load_byte;       // raw value to pass to loadLoadArray()
};

// Mock table - replace ALL values after tunnel testing in Colorado
const int LOOKUP_TABLE_SIZE = 9;
const LookupEntry lookupTable[LOOKUP_TABLE_SIZE] = {
    //la_position, resistance_ohms, load_byte needs to be chosen from data
    // wind_speed, la_position, resistance_ohms, load_byte
    {  5.0f,       88,          10.0f,           0b00000001 }, 
    {  6.0f,       93,          20.0f,           0b00000011 },
    {  7.0f,       98,          40.0f,           0b00000111 },
    {  8.0f,       104,         80.0f,           0b00001111 },
    {  9.0f,       104,         80.0f,           0b00001111 },
    { 10.0f,       104,         80.0f,           0b00001111 },
    { 11.0f,       110,         160.0f,          0b00011111 },
    { 12.0f,       104,         80.0f,           0b00001111 },
    { 13.0f,       104,         80.0f,           0b00001111 }
};

// TUNABLE CONSTANTS

const float RPM_MAX_THRESHOLD          = 3000.0f;  // RPM above this triggers e-stop
const float RPM_MIN_THRESHOLD          = 50.0f;   // RPM below this means turbine not spinning
const float RPM_MARGIN                 = 15.0f;   // Variance allowed to consider RPM stable
const unsigned long RPM_STABLE_TIME_MS = 1500;    // Time RPM must stay within margin to be stable
const float HILL_CLIMB_LOAD_RPM_MIN    = 100.0f;  // Don't drop below this RPM during load hill climb
const int   HILL_CLIMB_LA_STEP         = 1;       // LA step size during pitch hill climbing
const int   HILL_CLIMB_LOAD_STEP       = 1;       // Load byte increment during load hill climbing
const int   HILL_CLIMB_ITERATIONS      = 2;       // How many pitch+load hill climb cycles per wind speed
const float LOAD_DETECT_VOLTAGE_MIN    = 0.5f;    // Min voltage to consider load connected
const float LOAD_DETECT_CURRENT_MIN    = 0.05f;   // Min current to consider load connected
const float POWER_MARGIN_WATTS         = 0.5f;    // Acceptable power error during wattage maintenance
const float VOLTAGE_SOFT_LIMIT         = 24.0f;   // Start reducing RPM above this voltage
const float VOLTAGE_HARD_LIMIT         = 30.0f;   // Capacitor danger zone - never exceed
const int   VOLTAGE_LA_STEP            = 2;       // LA step when voltage limit is hit
const float RPM_WIND_INCREMENT_THRESHOLD = 20.0f; // RPM rise that indicates 1 m/s wind increase
const float WIND_SPEED_POWER_HOLD      = 10.0f;   // Switch to maintain wattage at this wind speed
const float WIND_SPEED_MAX             = 13.0f;   // Competition maximum wind speed
const float WIND_SPEED_INCREMENT       = 1.0f;    // Competition increases by 1 m/s at a time

Servo linearActuator;
int   la_current_position = LA_HOME_POSITION;

volatile unsigned long lastPulseTime  = 0;
volatile unsigned long pulseInterval  = 0;

volatile bool is_recording = false;
char current_filename[20];

float    estimated_wind_speed = 5.0f;
float    target_power_watts   = 0.0f;
uint8_t  current_load_byte    = 0;
int      hill_climb_iteration = 0;

struct DataPoint {
    unsigned long timestamp;
    float rpm;
    float wind_speed;
    float voltage;
    float current;
    int   la_position;
    uint8_t load_byte;
};

DataPoint buffer_A[BUFFER_SIZE];
DataPoint buffer_B[BUFFER_SIZE];

DataPoint* write_buffer = buffer_A;
DataPoint* flush_buffer = buffer_B;

volatile int  write_index    = 0;
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

void ir_interrupt() {
    unsigned long now = micros();
    if (now - lastPulseTime < 10000) return; // debounce - ignore pulses < 10ms apart
    pulseInterval = now - lastPulseTime;
    lastPulseTime = now;
}

void setup() {
    Serial.begin(9600);

    // IR sensor
    pinMode(IR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IR_PIN), ir_interrupt, RISING);

    // Linear actuator
    linearActuator.attach(LA_PIN);
    linearActuator.write(LA_HOME_POSITION);
    delay(15);

    // Button inputs
    pinMode(LA_EXTEND_BTN_PIN,  INPUT_PULLUP);
    pinMode(LA_RETRACT_BTN_PIN, INPUT_PULLUP);
    pinMode(LA_GO_HOME_BTN_PIN, INPUT_PULLUP);
    pinMode(RECORD_BTN_PIN,     INPUT_PULLUP);
    pinMode(ESTOP_READ_PIN,     INPUT_PULLUP);

    setup_sd();
    setup_loads();
    setup_lmp();

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
        digitalWrite(LOAD_PINS[i], LOAD_ACTIVE_HIGH ? LOW : HIGH); // default all off
    }
}

void setup_lmp() {
    pinMode(CS2_PIN, OUTPUT);
    digitalWrite(CS2_PIN, HIGH);
    SPI.begin();
    delay(10);
    Serial.println("LMP92064 SPI ready.");
}

bool is_legal_la_write(int write_value) {
    return !(write_value > LA_EXTEND || write_value < LA_RETRACT);
}

void handle_actuator_write(int write_value) {
    if (is_legal_la_write(write_value)) {
        linearActuator.write(write_value);
        la_current_position = write_value;
    }
}

void handle_actuator_buttons() {
    bool extendPressed  = digitalRead(LA_EXTEND_BTN_PIN) == LOW;
    bool retractPressed = digitalRead(LA_RETRACT_BTN_PIN) == LOW;

    if (extendPressed && la_current_position + LA_STEP_SIZE < LA_EXTEND) {
        la_current_position += LA_STEP_SIZE;
        linearActuator.write(la_current_position);
        Serial.print("LA position: "); Serial.println(la_current_position);
        delay(100);
    } else if (retractPressed && la_current_position - LA_STEP_SIZE > LA_RETRACT) {
        la_current_position -= LA_STEP_SIZE;
        linearActuator.write(la_current_position);
        Serial.print("LA position: "); Serial.println(la_current_position);
        delay(100);
    }
}

void handle_la_go_home() {
    if (digitalRead(LA_GO_HOME_BTN_PIN) == LOW) {
        linearActuator.write(LA_HOME_POSITION);
        la_current_position = LA_HOME_POSITION;
        delay(50);
        Serial.println("LA going home.");
    }
}

void eStop() {
    // Gradually move LA to e-stop position to avoid sudden mechanical shock
    if (la_current_position > E_STOP_POSITION) {
        for (int i = la_current_position; i > E_STOP_POSITION; i--) {
            handle_actuator_write(i);
            delay(25);
        }
    } else {
        for (int i = la_current_position; i < E_STOP_POSITION; i++) {
            handle_actuator_write(i);
            delay(25);
        }
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

uint8_t lmp_read_register(uint16_t reg_addr) {
    uint16_t cmd = 0x8000 | (reg_addr & 0x7FFF);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(CS2_PIN, LOW);
    SPI.transfer16(cmd);
    uint8_t result = SPI.transfer(0x00);
    digitalWrite(CS2_PIN, HIGH);
    SPI.endTransaction();
    return result;
}

float getVoltage() {
    (void)lmp_read_register(LMP_REG_COUT_MSB); // required read - unlocks conversion data
    (void)lmp_read_register(LMP_REG_COUT_LSB); // required read - unlocks conversion data
    uint8_t vout_msb = lmp_read_register(LMP_REG_VOUT_MSB);
    uint8_t vout_lsb = lmp_read_register(LMP_REG_VOUT_LSB);
    uint16_t vout_code = ((uint16_t)(vout_msb & 0x0F) << 8) | vout_lsb;
    float v_sensed = (vout_code / LMP_ADC_COUNTS) * LMP_VREF;
    return v_sensed * LMP_VDIV_FACTOR;
}

float getCurrent() {
    uint8_t cout_msb = lmp_read_register(LMP_REG_COUT_MSB);
    uint8_t cout_lsb = lmp_read_register(LMP_REG_COUT_LSB);
    (void)lmp_read_register(LMP_REG_VOUT_MSB); // required read - unlocks conversion data
    (void)lmp_read_register(LMP_REG_VOUT_LSB); // required read - unlocks conversion data
    uint16_t cout_code = ((uint16_t)(cout_msb & 0x0F) << 8) | cout_lsb;
    float v_diff = (cout_code / LMP_ADC_COUNTS) * LMP_VREF / 25.0f;
    return v_diff / R_SENSE;
}

float get_power() {
    return getVoltage() * getCurrent();
}

float get_rpm() {
    noInterrupts();
    unsigned long interval  = pulseInterval;
    unsigned long lastPulse = lastPulseTime;
    interrupts();
    if (interval == 0) return 0.0f;
    if (micros() - lastPulse > 2000000) return 0.0f; // no pulse in 2s = stopped
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
        baseline_set = false; // reset for next increment
    }
}

int lookup_la_position(float wind_speed) {
    if (wind_speed <= lookupTable[0].wind_speed)
        return lookupTable[0].la_position;
    if (wind_speed >= lookupTable[LOOKUP_TABLE_SIZE - 1].wind_speed)
        return lookupTable[LOOKUP_TABLE_SIZE - 1].la_position;
    for (int i = 0; i < LOOKUP_TABLE_SIZE - 1; i++) {
        if (wind_speed >= lookupTable[i].wind_speed &&
            wind_speed <= lookupTable[i+1].wind_speed) {
            float t = (wind_speed - lookupTable[i].wind_speed) /
                      (lookupTable[i+1].wind_speed - lookupTable[i].wind_speed);
            return (int)(lookupTable[i].la_position +
                         t * (lookupTable[i+1].la_position - lookupTable[i].la_position));
        }
    }
    return LA_HOME_POSITION;
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

bool detect_load() {
    return (getVoltage() > LOAD_DETECT_VOLTAGE_MIN &&
            getCurrent() > LOAD_DETECT_CURRENT_MIN);
}

bool should_estop() {
    if (digitalRead(ESTOP_READ_PIN) == LOW)  return true;
    if (get_rpm() > RPM_MAX_THRESHOLD)       return true;
    if (!detect_load())                      return true;
    return false;
}

void check_voltage_limit() {
    float voltage = getVoltage();
    if (voltage >= VOLTAGE_HARD_LIMIT) {
        handle_actuator_write(LA_EXTEND);
        Serial.println("WARNING: Voltage at hard limit!");
        return;
    }
    if (voltage >= VOLTAGE_SOFT_LIMIT) {
        handle_actuator_write(la_current_position + VOLTAGE_LA_STEP);
        Serial.print("WARNING: Voltage soft limit: ");
        Serial.println(voltage);
    }
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
    float rpm  = get_rpm();
    bool  load = detect_load();
    // Only recover once hardware button released, load present, RPM safe
    if (digitalRead(ESTOP_READ_PIN) == HIGH && load && rpm < RPM_MAX_THRESHOLD) {
        current_state = STATE_STARTUP;
    }
}

void state_startup() {
    if (should_estop()) { current_state = STATE_ESTOP; return; }
    check_voltage_limit();

    // Reset all state variables for a clean run
    estimated_wind_speed  = 5.0f;
    hill_climb_iteration  = 0;
    current_load_byte     = 0;
    target_power_watts    = 0.0f;
    loadLoadArray(0);
    loadLoad();

    float rpm = get_rpm();
    Serial.print("STARTUP - RPM: "); Serial.println(rpm);

    if (!detect_load()) { current_state = STATE_ESTOP; return; }

    // Wait for turbine to be meaningfully spinning before proceeding
    if (rpm > RPM_MIN_THRESHOLD) {
        Serial.println("Turbine spinning, transitioning to LOOKUP.");
        current_state = STATE_LOOKUP;
    }
}

void state_lookup() {
    if (should_estop()) { current_state = STATE_ESTOP; return; }
    check_voltage_limit();

    float   wind      = get_windspeed();
    float   rpm       = get_rpm();
    int     target_la = lookup_la_position(wind);
    uint8_t target_load = lookup_load_byte(wind);

    Serial.print("LOOKUP - Wind: "); Serial.print(wind);
    Serial.print(" RPM: "); Serial.print(rpm);
    Serial.print(" Target LA: "); Serial.print(target_la);
    Serial.print(" Target Load: "); Serial.println(target_load);

    // Step LA toward target one step at a time
    if (la_current_position != target_la) {
        int step = (target_la > la_current_position) ? HILL_CLIMB_LA_STEP : -HILL_CLIMB_LA_STEP;
        handle_actuator_write(la_current_position + step);
        return;
    }

    // LA at target - wait for RPM to stabilize
    if (!is_rpm_stable()) return;

    // RPM stable - step load toward target one step at a time
    if (current_load_byte != target_load) {
        if (current_load_byte < target_load) current_load_byte += HILL_CLIMB_LOAD_STEP;
        else                                 current_load_byte -= HILL_CLIMB_LOAD_STEP;
        loadLoadArray(current_load_byte);
        loadLoad();
        return;
    }

    // Load at target - wait for RPM to stabilize again
    if (!is_rpm_stable()) return;

    // Both at target and stable - begin hill climbing
    Serial.println("Lookup complete, starting hill climb.");
    hill_climb_iteration = 0;
    current_state = STATE_HILL_CLIMB_PITCH;
}

void state_hill_climb_pitch() {
    if (should_estop()) { current_state = STATE_ESTOP; return; }
    check_voltage_limit();

    float rpm = get_rpm();
    Serial.print("HILL CLIMB PITCH - RPM: "); Serial.println(rpm);

    // Max RPM hit - stop and move to load hill climbing
    if (rpm >= RPM_MAX_THRESHOLD) {
        Serial.println("Max RPM reached, transitioning to HILL CLIMB LOAD.");
        current_state = STATE_HILL_CLIMB_LOAD;
        return;
    }

    // Wait for RPM to stabilize before next step
    if (!is_rpm_stable()) return;

    // Take one step toward extend to increase pitch
    if (la_current_position + HILL_CLIMB_LA_STEP <= LA_EXTEND) {
        handle_actuator_write(la_current_position + HILL_CLIMB_LA_STEP);
    } else {
        // LA at physical limit, move on
        Serial.println("LA at extend limit, transitioning to HILL CLIMB LOAD.");
        current_state = STATE_HILL_CLIMB_LOAD;
    }
}

void state_hill_climb_load() {
    if (should_estop()) { current_state = STATE_ESTOP; return; }
    check_voltage_limit();

    float rpm = get_rpm();
    Serial.print("HILL CLIMB LOAD - RPM: "); Serial.print(rpm);
    Serial.print(" Load byte: "); Serial.println(current_load_byte);

    // Wait for RPM to stabilize before evaluating
    if (!is_rpm_stable()) return;

    // RPM dropped too low - back off load until stable
    if (rpm < HILL_CLIMB_LOAD_RPM_MIN) {
        if (current_load_byte > 0) {
            current_load_byte -= HILL_CLIMB_LOAD_STEP;
            loadLoadArray(current_load_byte);
            loadLoad();
            Serial.println("RPM too low, backing off load.");
        } else {
            // Load at zero and RPM still too low - something is wrong
            Serial.println("Load at zero, RPM still low. E-stopping.");
            current_state = STATE_ESTOP;
        }
        return;
    }

    // RPM healthy - try increasing load
    if (current_load_byte < 255) {
        current_load_byte += HILL_CLIMB_LOAD_STEP;
        loadLoadArray(current_load_byte);
        loadLoad();
        return;
    }

    // Reached max load - this iteration is complete
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
    check_voltage_limit();

    float wind = get_windspeed();
    Serial.print("IDLE - Estimated wind: "); Serial.print(wind);
    Serial.print(" Power: "); Serial.println(get_power());

    // At 10 m/s, capture current power as target and switch to maintain mode
    if (wind >= WIND_SPEED_POWER_HOLD) {
        target_power_watts = get_power();
        Serial.print("Switching to MAINTAIN WATTAGE. Target power: ");
        Serial.println(target_power_watts);
        current_state = STATE_MAINTAIN_WATTAGE;
        return;
    }

    // Watch for wind speed increase
    float prev_wind = estimated_wind_speed;
    update_wind_speed();
    if (estimated_wind_speed > prev_wind) {
        Serial.println("Wind speed increased, re-optimizing via LOOKUP.");
        hill_climb_iteration = 0;
        current_state = STATE_LOOKUP;
        return;
    }
}

void state_maintain_wattage() {
    if (should_estop()) { current_state = STATE_ESTOP; return; }
    check_voltage_limit();

    float current_power = get_power();
    float power_error   = current_power - target_power_watts;

    Serial.print("MAINTAIN - Power: "); Serial.print(current_power);
    Serial.print(" Target: "); Serial.println(target_power_watts);

    if (!is_rpm_stable()) return;

    if (power_error > POWER_MARGIN_WATTS) {
        // Too much power - increase load to reduce current
        if (current_load_byte < 255) {
            current_load_byte++;
            loadLoadArray(current_load_byte);
            loadLoad();
        }
    } else if (power_error < -POWER_MARGIN_WATTS) {
        // Too little power - decrease load to allow more current
        if (current_load_byte > 0) {
            current_load_byte--;
            loadLoadArray(current_load_byte);
            loadLoad();
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
            dp.voltage    = getVoltage();
            dp.current    = getCurrent();
            dp.la_position = la_current_position;
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
                    f.print(flush_buffer[i].voltage);    f.print(",");
                    f.print(flush_buffer[i].current);    f.print(",");
                    f.print(flush_buffer[i].la_position); f.print(",");
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
                f.println("timestamp_ms,rpm,wind_speed_ms,voltage,current,la_position,load_byte");
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
    handle_record_button();
    handle_actuator_buttons();
    handle_la_go_home();

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
