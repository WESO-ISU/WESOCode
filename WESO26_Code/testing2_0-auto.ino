//NEED TO KNOW SPECIFIC LIBRARIES TO USE FOR EACH COMPONENT.
#include <Servo.h>
#include <SD.h>
#include <TeensyThreads.h>
#include <stdint.h>
#include <SPI.h>


int LOAD_PIN1 = 2;
int LOAD_PIN2 = 3;
int LOAD_PIN3 = 4;
int LOAD_PIN4 = 5;
int LOAD_PIN5 = 6;
int LOAD_PIN6 = 7;
int LOAD_PIN7 = 8;
int LOAD_PIN8 = 9;

int CS1_PIN = 10;
int CS2_PIN = 37;
int SCK_PIN = 13;
int MOSI_PIN = 11;
int MISO_PIN = 12;

int IR_PIN = 15;
int LA_PIN = 0;
int ESTOP_WRITE_PIN = 22;
int ESTOP_READ_PIN = 23;

int LA_EXTEND_BTN_PIN = 24;
int LA_RETRACT_BTN_PIN = 25;

int RECORD_BTN_PIN = 26;
int LA_GO_HOME_BTN_PIN = 27;

const int LA_EXTEND = 120; //This should be the furthest out the linear actuator can move. Adjust as needed. 
const int LA_RETRACT = 80; //This should be the furthest in the linear actuator can move. Adjust as needed.
//All linear actuator movement needs to be bounded by these numbers. This is to avoid strain on linear actuator.
const int LA_STEP_SIZE = 1; //Common step size for moving linear actuator. Look into changing this to decimal values to give us more places to move to.
const int LA_HOME_POSITION = 102; //This is the home position of the linear actuator. Subject to change.
int la_current_position = 102; //This also acts as the starting position for la. Remeber to change this value whenever the linear actuator is moved to ensure position stays in bounds.

const int E_STOP_POSITION = 83; // Change this value to the correct e-stop value, 83 is likely incorrect. 

const int BUFFER_SIZE = 50;
const int FLUSH_INTERVAL_MS = 5000;
const int SAMPLE_INTERVAL_MS = 100;

volatile bool is_recording = false;
char current_filename[20];

Servo linearActuator;

// Wind speed estimation
float estimated_wind_speed = 5.0f;  // Always starts at 5 m/s per competition rules
const float WIND_SPEED_START = 5.0f;
const float WIND_SPEED_INCREMENT = 1.0f;   // Competition increases by 1 m/s at a time
const float WIND_SPEED_MAX = 13.0f;        // Competition max
const float WIND_SPEED_POWER_HOLD = 10.0f; // At this speed, hold power output constant
const float RPM_WIND_INCREMENT_THRESHOLD = 20.0f; // RPM increase that indicates a wind speed step - tune after testing
//const float AIR_DENSITY_IA = 1.197f; //Magic number: this is the air density in Ames Iowa, or places with ~292m of elevation
//const float AIR_DENSITY_CO = 1.045f; //Magic number: this is the air density in Boulder Colorado, or places with ~1655m of elevation
const float AIR_DENSITY = 1.197f;      //^ 
const int NUM_SAMPLES = 10; //Amount of samples to average the reading

float target_power_watts = 0.0f; 

int loadArray[8] = {1,1,1,1,1,1,1,1};
// Set to true if HIGH activates a resistor, false if LOW activates
const bool LOAD_ACTIVE_HIGH = true;

const int LOAD_PINS[8] = {
    LOAD_PIN8, LOAD_PIN7, LOAD_PIN6, LOAD_PIN5,
    LOAD_PIN4, LOAD_PIN3, LOAD_PIN2, LOAD_PIN1
};


struct DataPoint {
    unsigned long timestamp;
    float rpm;
    float wind_speed;
    float voltage;
    float current;
    int la_position;
};

DataPoint buffer_A[BUFFER_SIZE];
DataPoint buffer_B[BUFFER_SIZE];

DataPoint* write_buffer = buffer_A;
DataPoint* flush_buffer = buffer_B;

volatile int write_index = 0;
volatile bool flush_requested = false;

Threads::Mutex buffer_mutex;

volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;

//Voltage and current sensor stuff 
const float LMP_VREF = 2.048f;           // Internal reference voltage
const float LMP_ADC_COUNTS = 4096.0f;    // 12-bit ADC

// Current channel: code = (V_diff * gain * ADC_counts) / VREF
// gain = 25, so current = (code / 50000) / R_SENSE
const float R_SENSE = 0.020f;            // 20m ohmplaceholder - update when confirmed

// Voltage divider: R2=1.6k ohm, R3=46.4k ohm from datasheet example - update when confirmed
const float LMP_VDIV_R2 = 1600.0f;
const float LMP_VDIV_R3 = 46400.0f;
const float LMP_VDIV_FACTOR = (LMP_VDIV_R2 + LMP_VDIV_R3) / LMP_VDIV_R2;

// Register addresses
const uint16_t LMP_REG_STATUS   = 0x0103;
const uint16_t LMP_REG_VOUT_LSB = 0x0200;
const uint16_t LMP_REG_VOUT_MSB = 0x0201;
const uint16_t LMP_REG_COUT_LSB = 0x0202;
const uint16_t LMP_REG_COUT_MSB = 0x0203;


//Random tunable consts, I am loosing the plot pretty hard...
const float WIND_STABLE_DELTA = 0.3f;    // m/s — max change between samples to be considered stable
const unsigned long WIND_STABLE_TIME_MS = 750; // how long wind must stay within delta to be considered stable


//Lookup table bullshit
struct LookupEntry {
    float wind_speed;        // m/s
    int la_position;         // target LA position for this wind speed
    float resistance_ohms;   // human-readable resistance value
    uint8_t load_byte;       // raw value to pass to loadLoadArray()
};

// Mock table - replace all values after tunnel testing
const int LOOKUP_TABLE_SIZE = 5;
const LookupEntry lookupTable[LOOKUP_TABLE_SIZE] = {
    // wind_speed, la_position, resistance_ohms, load_byte
    { 4.0f,  88,  10.0f, 0b00000001 },
    { 6.0f,  93,  20.0f, 0b00000011 },
    { 8.0f,  98,  40.0f, 0b00000111 },
    { 10.0f, 104, 80.0f, 0b00001111 },
    { 12.0f, 110, 160.0f, 0b00011111 } //This shi should be a bit longer. we NEED to get this jawn looking nice during testind monday 4/13/2026
};


void ir_interrupt() {
    unsigned long now = micros();
    if (now - lastPulseTime < 10000) return;  // ignore if less than 10ms since last pulse
    pulseInterval = now - lastPulseTime;
    lastPulseTime = now;
}

//Consider making setup methods like setup_sd() for each component that needs to be setup for readability and scalability.
void setup() {
  pinMode(IR_PIN, INPUT);
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(IR_PIN), ir_interrupt, RISING);

  linearActuator.attach(LA_PIN);
  linearActuator.write(LA_HOME_POSITION);
  delay(15);
  pinMode(IR_PIN, INPUT_PULLUP);
  pinMode(LA_EXTEND_BTN_PIN, INPUT_PULLUP);
  pinMode(LA_RETRACT_BTN_PIN, INPUT_PULLUP);
  pinMode(LA_GO_HOME_BTN_PIN, INPUT_PULLUP);
  
  setup_sd();  
  threads.addThread(sampling_thread);
  threads.addThread(flush_thread);
  pinMode(RECORD_BTN_PIN, INPUT_PULLUP);
  setup_loads();
  setup_lmp();
}



void setup_sd(){
    if(!SD.begin(BUILTIN_SDCARD)){
        Serial.println("SD init has failed.");
        return;
    }
    Serial.println("SD setup was successful.");
}

void setup_loads() {
    for (int i = 0; i < 8; i++) {
        pinMode(LOAD_PINS[i], OUTPUT);
        digitalWrite(LOAD_PINS[i], LOAD_ACTIVE_HIGH ? LOW : HIGH); // default all off
    }
}

void setup_lmp(){
    pinMode(CS2_PIN, OUTPUT);
    digitalWrite(CS2_PIN, HIGH);
    SPI.begin();
    delay(10);
    Serial.println("LMP SPI ready");
}


//E-stop states methods 
void eStop(){
    if(la_current_position - E_STOP_POSITION > 0){
      for(int i = la_current_position; i > E_STOP_POSITION; i--){
          handle_actuator_write(i);
          delay(25);
      }
    } else {
        for(int i = la_current_position; i < E_STOP_POSITION; i++){
            handle_actuator_write(i);
            delay(25);
        }
    }
   
    //Should e-stop do anything after this?
    //Should it wait a while and restart, or kill everthing completely?
}
  

  

//Start-up state methods 

  

//Active reading state methods 


//Load change state methods 
  

//Hill slimber state methods  

  

//Idle state methods 

  

//Data collection methods 

//Linear actuator methods.
void handle_actuator_buttons(){
    bool extendPressed = digitalRead(LA_EXTEND_BTN_PIN) == LOW;
    bool retractPressed = digitalRead(LA_RETRACT_BTN_PIN) == LOW;

    if(extendPressed && la_current_position + LA_STEP_SIZE < LA_EXTEND){
        la_current_position += LA_STEP_SIZE;
        linearActuator.write(la_current_position);
        Serial.print("Current LA position ");
        Serial.println(la_current_position);
        delay(100); //Allows actuator to catch up
    } else if(retractPressed && la_current_position - LA_STEP_SIZE > LA_RETRACT){
        la_current_position -= LA_STEP_SIZE;
        linearActuator.write(la_current_position);
        Serial.print("Current LA position ");
        Serial.println(la_current_position);
        delay(100);
    }
}

void handle_la_go_home(){
  bool laGoHome = digitalRead(LA_GO_HOME_BTN_PIN) == LOW;

  if(laGoHome){
    linearActuator.write(LA_HOME_POSITION);
    la_current_position = LA_HOME_POSITION;
    delay(50);
    Serial.println("LA going home");
  }
}

void handle_actuator_write(int write_value){
    if(is_legal_la_write(write_value)){
        linearActuator.write(write_value);
        la_current_position = write_value;
    }
}


bool is_legal_la_step(int step_size){
      //return true;
    return !(la_current_position + step_size > LA_EXTEND || la_current_position - step_size < LA_RETRACT);
}

bool is_legal_la_write(int write_value){
    return !(write_value > LA_EXTEND || write_value < LA_RETRACT);
}


  
/* These record methods will actually exist in 
sampling_thread(). The method will make a call to some 
get method. This will allow us to format the data in the 
file better. We will keep the method stubs to remind us 
what to record. 

void record_rpm(); 
/ write to sd card  

void record_voltage(); 

void record_current(); 

void record_windspeed(); //if pitot tube is here n all set up.  

void detect_load();
*/

void sampling_thread(){
    while(1) {
        if(is_recording && write_index < BUFFER_SIZE){
            DataPoint dp;
            dp.timestamp = millis();
            dp.rpm = get_rpm();
            dp.wind_speed = get_windspeed();
            dp.voltage = getVoltage();
            dp.current = getCurrent();
            dp.la_position = la_current_position;

            buffer_mutex.lock();
            write_buffer[write_index++] = dp;
            buffer_mutex.unlock();
        }
        threads.delay(SAMPLE_INTERVAL_MS);
    }
}

void flush_thread(){
    while(1){
        threads.delay(FLUSH_INTERVAL_MS);
        if(is_recording){
            buffer_mutex.lock();

            DataPoint* temp = write_buffer;
            write_buffer = flush_buffer;
            flush_buffer = temp;
            int count = write_index;
            write_index = 0;
            buffer_mutex.unlock();

            File f = SD.open(current_filename, FILE_WRITE);
                if(f){
                    for (int i = 0; i < count; i++){
                        f.print(flush_buffer[i].timestamp);
                        f.print(",");
                        f.print(flush_buffer[i].rpm);
                        f.print(",");
                        f.print(flush_buffer[i].wind_speed);
                        f.print(",");
                        f.print(flush_buffer[i].voltage);
                        f.print(",");
                        f.print(flush_buffer[i].current);
                        f.print(",");
                        f.println(flush_buffer[i].la_position);
                    }
                f.close();
                Serial.print("Flushed ");
                Serial.print(count);
                Serial.println( "records to sd");
            } else {
                Serial.println("SD write failed");
            }
            
        }
    }
}

void handle_record_button(){
    static bool last_btn_state = HIGH;
    bool btn_state = digitalRead(RECORD_BTN_PIN);

    if(last_btn_state == HIGH && btn_state == LOW){
        if(!is_recording){
            start_recording();
        } else {
            stop_recording();
        }
    }
    last_btn_state = btn_state;
}

void start_recording(){
    snprintf(current_filename, sizeof(current_filename), "log_%lu.csv", millis());

    File f = SD.open(current_filename, FILE_WRITE);
    if (f){
        f.println("timestamp_ms,rpm,wind_speed_ms,voltage,current,la_position");
        f.close();
        is_recording = true;
        Serial.print("Recording started: ");
        Serial.println(current_filename);
    } else {
        Serial.println("Failed to create file!");
    }
}

void stop_recording(){
    is_recording = false;
    Serial.print("Recording stopped: ");
    Serial.println(current_filename);
}
  



float get_rpm() {
    noInterrupts();
    unsigned long interval = pulseInterval;
    unsigned long lastPulse = lastPulseTime;
    interrupts();

    if (interval == 0) return 0.0;
    if (micros() - lastPulse > 2000000) return 0.0;
    return 60.0 / (interval / 1000000.0);
}



float get_windspeed() {
    return estimated_wind_speed;
}

void update_wind_speed() {
    static float baseline_rpm = 0.0f;
    static bool baseline_set = false;

    float rpm = get_rpm();

    // Set baseline RPM when we first enter idle at this wind speed
    if (!baseline_set) {
        baseline_rpm = rpm;
        baseline_set = true;
        return;
    }

    // If RPM has increased significantly above baseline, wind speed has stepped up
    if (rpm - baseline_rpm > RPM_WIND_INCREMENT_THRESHOLD) {
        if (estimated_wind_speed < WIND_SPEED_MAX) {
            estimated_wind_speed += WIND_SPEED_INCREMENT;
            Serial.print("Wind speed updated to: ");
            Serial.println(estimated_wind_speed);
        }
        // Reset baseline for next increment
        baseline_set = false;
    }
}

  
uint8_t lmp_read_register(uint16_t reg_addr){
    uint16_t cmd = 0x8000 | (reg_addr & 0x7FFF);

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(CS2_PIN, LOW);
    SPI.transfer16(cmd);
    uint8_t result = SPI.transfer(0x00);
    digitalWrite(CS2_PIN, HIGH);
    SPI.endTransaction();
   
    return result;
}

float getVoltage(){
    uint8_t cout_msb = lmp_read_register(LMP_REG_COUT_MSB);
    uint8_t cout_lsb = lmp_read_register(LMP_REG_COUT_LSB);
    uint8_t vout_msb = lmp_read_register(LMP_REG_VOUT_MSB);
    uint8_t vout_lsb = lmp_read_register(LMP_REG_VOUT_LSB);

    uint16_t vout_code = ((uint16_t)(vout_msb & 0x0f) << 8) | vout_lsb;

    float v_sensed = (vout_code / LMP_ADC_COUNTS) * LMP_VREF;
    return v_sensed * LMP_VDIV_FACTOR;
}

float getCurrent(){
    uint8_t cout_msb = lmp_read_register(LMP_REG_COUT_MSB);
    uint8_t cout_lsb = lmp_read_register(LMP_REG_COUT_LSB);
    uint8_t vout_msb = lmp_read_register(LMP_REG_VOUT_MSB);
    uint8_t vout_lsb = lmp_read_register(LMP_REG_VOUT_LSB);

    uint16_t cout_code = ((uint16_t)(cout_msb & 0x0F) << 8) | cout_lsb;

    float v_diff = (cout_code / LMP_ADC_COUNTS) * LMP_VREF / 25.0f;
    return v_diff / R_SENSE;
}

float get_power() {
    return getVoltage() * getCurrent();
}

  

//LOAD METHODS HERE
//loads the load array with the correct binary values from the given uint8_t
void loadLoadArray(uint8_t loadValue){ 
    for (int i = 0; i < 8; i++){
        loadArray[7-i] = (loadValue >> (7 - i)) & 1; 
    }
}

void printLoadArray(){
    for(int i = 0; i < 7; i++){
        Serial.print(loadArray[i]);    
    }
    Serial.println("");
}

void loadLoad() {
    for (int i = 0; i < 8; i++) {
        bool activate = (loadArray[i] == 1);
        digitalWrite(LOAD_PINS[i], (activate == LOAD_ACTIVE_HIGH) ? HIGH : LOW);
    }
}


//State machine stuff


bool is_wind_stable(float current_wind) {
    static float last_wind = 0.0f;
    static unsigned long stable_since = 0;

    if (abs(current_wind - last_wind) > WIND_STABLE_DELTA) {
        // Wind is still changing, reset the timer
        stable_since = millis();
    }

    last_wind = current_wind;
    return (millis() - stable_since >= WIND_STABLE_TIME_MS);
}

int lookup_la_position(float wind_speed) {
    // Below table minimum
    if (wind_speed <= lookupTable[0].wind_speed)
        return lookupTable[0].la_position;
    // Above table maximum
    if (wind_speed >= lookupTable[LOOKUP_TABLE_SIZE - 1].wind_speed)
        return lookupTable[LOOKUP_TABLE_SIZE - 1].la_position;

    // Find surrounding entries and interpolate
    for (int i = 0; i < LOOKUP_TABLE_SIZE - 1; i++) {
        if (wind_speed >= lookupTable[i].wind_speed && 
            wind_speed <= lookupTable[i+1].wind_speed) {
            float t = (wind_speed - lookupTable[i].wind_speed) /
                      (lookupTable[i+1].wind_speed - lookupTable[i].wind_speed);
            return (int)(lookupTable[i].la_position + 
                        t * (lookupTable[i+1].la_position - lookupTable[i].la_position));
        }
    }
    return LA_HOME_POSITION; // fallback
}

uint8_t lookup_load_byte(float wind_speed) {
    // Below table minimum
    if (wind_speed <= lookupTable[0].wind_speed)
        return lookupTable[0].load_byte;
    // Above table maximum
    if (wind_speed >= lookupTable[LOOKUP_TABLE_SIZE - 1].wind_speed)
        return lookupTable[LOOKUP_TABLE_SIZE - 1].load_byte;

    // Find surrounding entries and interpolate the resistance, then find nearest load_byte
    for (int i = 0; i < LOOKUP_TABLE_SIZE - 1; i++) {
        if (wind_speed >= lookupTable[i].wind_speed && 
            wind_speed <= lookupTable[i+1].wind_speed) {
            float t = (wind_speed - lookupTable[i].wind_speed) /
                      (lookupTable[i+1].wind_speed - lookupTable[i].wind_speed);
            // Interpolate resistance then round to nearest load_byte
            float interp_resistance = lookupTable[i].resistance_ohms +
                        t * (lookupTable[i+1].resistance_ohms - lookupTable[i].resistance_ohms);
            // Find the entry whose resistance is closest to interpolated value
            int best = 0;
            float best_diff = abs(lookupTable[0].resistance_ohms - interp_resistance);
            for (int j = 1; j < LOOKUP_TABLE_SIZE; j++) {
                float diff = abs(lookupTable[j].resistance_ohms - interp_resistance);
                if (diff < best_diff) { best_diff = diff; best = j; }
            }
            return lookupTable[best].load_byte;
        }
    }
    return 0; // fallback - no load
}



// ---- Tunable Constants ----
const float RPM_MAX_THRESHOLD = 300.0f;
const float RPM_MIN_THRESHOLD = 50.0f;
const float RPM_TARGET = 200.0f;              // Update after tunnel testing
const float RPM_MARGIN = 10.0f;
const float RPM_DROP_THRESHOLD = 80.0f;
const int   HILL_CLIMBER_WAIT_MS = 2000;
const int   HILL_CLIMBER_LA_STEP = 2;
const int   LOAD_CHANGE_WAIT_MS = 3000;
const float LOAD_DETECT_VOLTAGE_MIN = 0.5f;
const float LOAD_DETECT_CURRENT_MIN = 0.05f;
const float POWER_MARGIN_WATTS = 0.5f;  // Acceptable power error in watts - tune after testing
const float VOLTAGE_SOFT_LIMIT = 24.0f;  // Start reducing RPM above this
const float VOLTAGE_HARD_LIMIT = 30.0f;  // Capacitor danger zone, never exceed
const int   VOLTAGE_LA_STEP = 2;         // How much to extend LA when voltage is too high
// ---- State Machine ----
enum State {
    STATE_ESTOP,
    STATE_STARTUP,
    STATE_ACTIVE_READING,
    STATE_LOAD_CHANGE,
    STATE_HILL_CLIMBER,
    STATE_IDLE
};

State current_state = STATE_STARTUP;

// ---- Helpers ----
bool detect_load() {
    return (getVoltage() > LOAD_DETECT_VOLTAGE_MIN && 
            getCurrent() > LOAD_DETECT_CURRENT_MIN);
}

bool should_estop() {
    if (digitalRead(ESTOP_READ_PIN) == LOW) return true;
    if (get_rpm() > RPM_MAX_THRESHOLD) return true;
    if (!detect_load()) return true;
    return false;
}

void check_voltage_limit() {
    float voltage = getVoltage();

    if (voltage >= VOLTAGE_HARD_LIMIT) {
        // Emergency - extend LA as far as safely possible to reduce RPM fast
        handle_actuator_write(LA_EXTEND);
        Serial.println("WARNING: Voltage at hard limit!");
        return;
    }

    if (voltage >= VOLTAGE_SOFT_LIMIT) {
        // Soft limit - nudge LA outward to reduce RPM gradually
        handle_actuator_write(la_current_position + VOLTAGE_LA_STEP);
        Serial.print("WARNING: Voltage soft limit hit: ");
        Serial.println(voltage);
    }
}

// ---- State Functions ----
void state_estop() {
    eStop();
    float rpm = get_rpm();
    bool load = detect_load();

    Serial.println("E-STOP");

    if (digitalRead(ESTOP_READ_PIN) == HIGH && load && rpm < RPM_MAX_THRESHOLD) {
        current_state = STATE_STARTUP;
    }
}

void state_startup() {
    if (should_estop()) { current_state = STATE_ESTOP; return; }
    check_voltage_limit();

    float rpm = get_rpm();
    bool load = detect_load();

    Serial.print("STARTUP - RPM: "); Serial.println(rpm);

    if (!load) { current_state = STATE_ESTOP; return; }

    if (rpm > RPM_MIN_THRESHOLD) {
        current_state = STATE_ACTIVE_READING;
    }
}

void state_load_change() {
    if (should_estop()) { current_state = STATE_ESTOP; return; }
    check_voltage_limit();

    float wind = get_windspeed();
    float rpm = get_rpm();

    Serial.print("LOAD CHANGE - RPM: "); Serial.print(rpm);
    Serial.print(" Wind: "); Serial.println(wind);

    // Load adjustment stubbed - implement after tunnel testing
    // loadLoadArray(some_value);
    // loadLoad();

    delay(LOAD_CHANGE_WAIT_MS);

    if (rpm < RPM_MIN_THRESHOLD) { current_state = STATE_STARTUP; return; }
    if (abs(rpm - RPM_TARGET) <= RPM_MARGIN) { current_state = STATE_ACTIVE_READING; return; }
    if (rpm >= RPM_MAX_THRESHOLD) { current_state = STATE_IDLE; return; }
}

void state_hill_climber() {
    if (should_estop()) { current_state = STATE_ESTOP; return; }
    check_voltage_limit();

    bool load = detect_load();
    if (!load) { current_state = STATE_ESTOP; return; }

    Serial.println("HILL CLIMBER");

    handle_actuator_write(la_current_position - HILL_CLIMBER_LA_STEP);
    delay(HILL_CLIMBER_WAIT_MS);

    float rpm = get_rpm();
    if (abs(rpm - RPM_TARGET) <= RPM_MARGIN) {
        current_state = STATE_ACTIVE_READING;
    }
}

void state_idle() {
    if (should_estop()) { current_state = STATE_ESTOP; return; }
    check_voltage_limit();

    float wind = get_windspeed();
    bool load = detect_load();

    Serial.print("IDLE - Estimated wind: "); Serial.println(wind);

    if (!load) { current_state = STATE_LOAD_CHANGE; return; }

    // Check if wind speed has incremented
    update_wind_speed();

    // If wind changed, go back to active reading to re-optimize
    if (estimated_wind_speed > wind) {
        current_state = STATE_ACTIVE_READING;
        return;
    }

    // At or above power hold threshold - maintain power output instead of maximizing
    if (estimated_wind_speed >= WIND_SPEED_POWER_HOLD) {
    // Record target power the first time we hit 10 m/s
    if (target_power_watts == 0.0f) {
        target_power_watts = get_power();
        Serial.print("Target power set: ");
        Serial.println(target_power_watts);
        return;
    }

    float current_power = get_power();
    float power_error = current_power - target_power_watts;

    // If producing too much power, increase load (more resistance = less current = less power)
    // If producing too little power, decrease load
    if (power_error > POWER_MARGIN_WATTS) {
        uint8_t current_load = loadArray[0]; // rough proxy for current load byte
        loadLoadArray(current_load + 1);
        loadLoad();
    } else if (power_error < -POWER_MARGIN_WATTS) {
        uint8_t current_load = loadArray[0];
        loadLoadArray(current_load - 1);
        loadLoad();
    }
    return;
}
}


void state_active_reading() {
    if (should_estop()) { current_state = STATE_ESTOP; return; }
    check_voltage_limit();
    float rpm = get_rpm();
    float wind = get_windspeed();
    bool load = detect_load();

    Serial.print("ACTIVE - RPM: "); Serial.print(rpm);
    Serial.print(" Wind: "); Serial.println(wind);

    if (!load) { current_state = STATE_ESTOP; return; }
    if (rpm < RPM_DROP_THRESHOLD) { current_state = STATE_HILL_CLIMBER; return; }

    if (!is_wind_stable(wind)) {
        // Wind is still changing, just wait
        return;
    }

    // Wind is stable - look up and step toward target LA position
    int target_la = lookup_la_position(wind);
    if (la_current_position != target_la) {
        int step = (target_la > la_current_position) ? LA_STEP_SIZE : -LA_STEP_SIZE;
        handle_actuator_write(la_current_position + step);
        return; // Step once per loop iteration, come back next iteration
    }

    // LA is at target - now check if RPM has stabilized
    static float last_rpm = 0.0f;
    static unsigned long rpm_stable_since = 0;
    if (abs(rpm - last_rpm) > RPM_MARGIN) {
        rpm_stable_since = millis();
    }
    last_rpm = rpm;

    if (millis() - rpm_stable_since < WIND_STABLE_TIME_MS) {
        // RPM still settling, wait
        return;
    }

    // RPM is stable - apply optimal load from lookup table
    uint8_t target_load = lookup_load_byte(wind);
    loadLoadArray(target_load);
    loadLoad();

    Serial.print("Load applied: 0b");
    Serial.println(target_load, BIN);
}



void loop() {
    handle_record_button();
    handle_actuator_buttons();
    handle_la_go_home();

    switch (current_state) {
        case STATE_ESTOP:          state_estop();          break;
        case STATE_STARTUP:        state_startup();        break;
        case STATE_ACTIVE_READING: state_active_reading(); break;
        case STATE_LOAD_CHANGE:    state_load_change();    break;
        case STATE_HILL_CLIMBER:   state_hill_climber();   break;
        case STATE_IDLE:           state_idle();           break;
    }
}



  

  

//How will we represent the data recorded during our testing time? 

// Table | RPM | voltage | current | 

//PRINT OUT ALL FUNCTIONS IN A CSV FORMAT.
