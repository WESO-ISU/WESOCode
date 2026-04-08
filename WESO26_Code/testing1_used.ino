//NEED TO KNOW SPECIFIC LIBRARIES TO USE FOR EACH COMPONENT.
#include <Servo.h>
#include <SD.h>
#include <TeensyThreads.h>
#include <stdint.h>


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

//Differential Pressor Sensor (DPS) setup stuff here
const int WIND_SENSOR_PIN = 14;
const int ADC_RESOLUTION = 4096;
const float ADC_VREF = 3.3f;
const float R1 = 6800.0f; // Top resistor value - subject to change - not currently accurate
const float R2 = 10000.0f; // Bottom resistor value - subject to change - not currently accurate
const float DIVIDER_FACTOR = (R1 + R2) / R2; 
const float VS = 5.0f;
//const float AIR_DENSITY_IA = 1.197f; //Magic number: this is the air density in Ames Iowa, or places with ~292m of elevation
//const float AIR_DENSITY_CO = 1.045f; //Magic number: this is the air density in Boulder Colorado, or places with ~1655m of elevation
const float AIR_DENSITY = 1.197f;      //^ 
const int NUM_SAMPLES = 10; //Amount of samples to average the reading

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
  setup_dps();
  setup_loads();
}

void setup_dps(){
    analogReadResolution(12);
    pinMode(WIND_SENSOR_PIN, INPUT);
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


//E-stop states methods 
void eStop(){
    handle_actuator_write(E_STOP_POSITION);
    //Should e-stop do anything after this?
    //Should it wait a while and restart, or kill everthing completely?
}
  

void e_stop(); //change linear actuator to the stopping angle.  

  

//Start-up state methods 

  

//Active reading state methods 
void active_reading();

void set_pitch_angle(float angle); //Requires analogWrite()

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
    return !(la_current_position + step_size > LA_EXTEND && la_current_position - step_size < LA_RETRACT);
}

bool is_legal_la_write(int write_value){
    return !(write_value > LA_EXTEND && write_value < LA_RETRACT);
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
  

//Methods used in multiple different states 

  



float get_rpm() {
    noInterrupts();
    unsigned long interval = pulseInterval;
    unsigned long lastPulse = lastPulseTime;
    interrupts();

    if (interval == 0) return 0.0;
    if (micros() - lastPulse > 2000000) return 0.0;
    return 60.0 / (interval / 1000000.0);
}



float get_windspeed(){
    long adcSum = 0;
    for(int i = 0; i < NUM_SAMPLES; i++){
        adcSum += analogRead(WIND_SENSOR_PIN);
        delayMicroseconds(200);
    }
    float adcAvg = (float)adcSum / NUM_SAMPLES;

    float vMeasured = (adcAvg / (ADC_RESOLUTION - 1)) * ADC_VREF;
    float vOut = vMeasured * DIVIDER_FACTOR;

    float pressureKPa = (vOut / VS - 0.5f) / 0.2f;
    float pressurePa = pressureKPa * 1000.0f;

    if(pressurePa <= 0.0f) return 0.0f;

    //Bernoulli's equation
    float windSpeed = sqrt(2.0f * pressurePa / AIR_DENSITY);
    return windSpeed;
} 

  

  
//These methods need to be completed. 
float getVoltage(){
    return -1;
}

float getCurrent(){
    return -1; 
}

  

//LOAD METHODS HERE
//loads the load array with the correct binary values from the given uint8_t
void loadLoadArray(uint8_t loadValue){ 
    for (int i = 0; i < 8; i++){
        loadArray[i] = (loadValue >> (7 - i)) & 1; 
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


void loop(){
    handle_record_button();
    handle_actuator_buttons();
    handle_la_go_home();

    Serial.print(get_rpm());
    Serial.print(" ");
    Serial.println(la_current_position);
    delay(25);
    
    
}



  

  

//How will we represent the data recorded during our testing time? 

// Table | RPM | voltage | current | 

//PRINT OUT ALL FUNCTIONS IN A CSV FORMAT.
