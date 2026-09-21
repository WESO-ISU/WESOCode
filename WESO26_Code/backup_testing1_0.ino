//NEED TO KNOW SPECIFIC LIBRARIES TO USE FOR EACH COMPONENT.
#include <Servo.h>
#include <SD.h>

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

int IR_PIN = 1;
int LA_PIN = 0;
int ESTOP_WRITE_PIN = 22;
int ESTOP_READ_PIN = 23;

int LA_EXTEND_BTN_PIN = 24;
int LA_RETRACT_BTN_PIN = 25;
int RECORD_BTN_PIN = 26;

const int LA_EXTEND = 120;
const int LA_RETRACT = 80;
const int LA_STEP_SIZE = 1;
const int LA_HOME_POSITION = 102;
int la_current_position = 102;

const int SAMPLE_INTERVAL_MS = 100;
const int FLUSH_INTERVAL_MS = 5000;
const int BUFFER_SIZE = 50;

volatile bool is_recording = false;
char current_filename[20];

Servo linearActuator;

struct DataPoint {
    unsigned long timestamp;
    float rpm;
    int la_position;
};

DataPoint buffer[BUFFER_SIZE];
int write_index = 0;

unsigned long last_sample_time = 0;
unsigned long last_flush_time = 0;

void setup() {
    pinMode(IR_PIN, INPUT);
    Serial.begin(9600);

    linearActuator.attach(LA_PIN);
    linearActuator.write(LA_HOME_POSITION);
    pinMode(LA_EXTEND_BTN_PIN, INPUT_PULLUP);
    pinMode(LA_RETRACT_BTN_PIN, INPUT_PULLUP);
    pinMode(RECORD_BTN_PIN, INPUT_PULLUP);

    setup_sd();
}

void setup_sd(){
    if(!SD.begin(BUILTIN_SDCARD)){
        Serial.println("SD init has failed.");
        return;
    }
    Serial.println("SD setup was successful.");
}

void handle_actuator_buttons(){
    bool extendPressed = digitalRead(LA_EXTEND_BTN_PIN) == LOW;
    bool retractPressed = digitalRead(LA_RETRACT_BTN_PIN) == LOW;

    if(extendPressed && is_legal_la_step(LA_STEP_SIZE)){
        la_current_position += LA_STEP_SIZE;
        linearActuator.write(la_current_position);
        delay(15);
    } else if(retractPressed && is_legal_la_step(LA_STEP_SIZE)){
        la_current_position -= LA_STEP_SIZE;
        linearActuator.write(la_current_position);
        delay(15);
    }
}

void handle_actuator_write(int write_value){
    if(is_legal_la_write(write_value)){
        linearActuator.write(write_value);
    }
}

bool is_legal_la_step(int step_size){
    return !(la_current_position + step_size > LA_EXTEND || la_current_position - step_size < LA_RETRACT);
}

bool is_legal_la_write(int write_value){
    return !(write_value > LA_EXTEND || write_value < LA_RETRACT);
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
    if(f){
        f.println("timestamp_ms,rpm,la_position");
        f.close();
        is_recording = true;
        write_index = 0;
        last_sample_time = millis();
        last_flush_time = millis();
        Serial.print("Recording started: ");
        Serial.println(current_filename);
    } else {
        Serial.println("Failed to create file!");
    }
}

void stop_recording(){
    // Flush any remaining data before stopping
    if(write_index > 0){
        flush_to_sd();
    }
    is_recording = false;
    Serial.print("Recording stopped: ");
    Serial.println(current_filename);
}

void sample_data(){
    if(write_index < BUFFER_SIZE){
        DataPoint dp;
        dp.timestamp = millis();
        dp.rpm = get_rpm();
        dp.la_position = la_current_position;
        buffer[write_index++] = dp;
    }
}

void flush_to_sd(){
    File f = SD.open(current_filename, FILE_WRITE);
    if(f){
        for(int i = 0; i < write_index; i++){
            f.print(buffer[i].timestamp);
            f.print(",");
            f.print(buffer[i].rpm);
            f.print(",");
            f.println(buffer[i].la_position);
        }
        f.close();
        Serial.print("Flushed ");
        Serial.print(write_index);
        Serial.println(" records to sd");
        write_index = 0;
    } else {
        Serial.println("SD write failed");
    }
}

void handle_data_recording(){
    unsigned long now = millis();

    // Sample every 100ms
    if(now - last_sample_time >= SAMPLE_INTERVAL_MS){
        last_sample_time = now;
        sample_data();
    }

    // Flush every 5 seconds or if buffer is full
    if(now - last_flush_time >= FLUSH_INTERVAL_MS || write_index >= BUFFER_SIZE){
        last_flush_time = now;
        flush_to_sd();
    }
}

float get_voltage();

float get_current();

int lastState = HIGH;
unsigned long pulseTimes[5];
int pulseIndex = 0;
bool bufferFull = false;
int NUM_BLADES = 1;

float get_rpm() {
    int sensorVal = digitalRead(IR_PIN);

    if(lastState == HIGH && sensorVal == LOW){
        unsigned long now = micros();
        pulseTimes[pulseIndex] = now;
        pulseIndex = (pulseIndex + 1) % NUM_BLADES;
        if(pulseIndex == 0) bufferFull = true;
    }

    lastState = sensorVal;

    if(!bufferFull) return 0.0;

    int oldest = pulseIndex;
    int newest = (pulseIndex + NUM_BLADES - 1) % NUM_BLADES;

    unsigned long revTime_us = pulseTimes[newest] - pulseTimes[oldest];

    if(revTime_us == 0) return 0.0;

    float revTime_sec = revTime_us / 1000000.0;
    return 60.0 / revTime_sec;
}

float get_windspeed();

int change_load_resistence(float resistance);

void loop(){
    handle_record_button();
    handle_actuator_buttons();
    if(is_recording){
        handle_data_recording();
    }
}