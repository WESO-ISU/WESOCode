//NEED TO KNOW SPECIFIC LIBRARIES TO USE FOR EACH COMPONENT.
#include <servo.h>
#include <SD.h>
#include <TeensyThreads.h>


int LOAD_PIN1 = 2;  //    1280 ohm
int LOAD_PIN2 = 3;  //     640 ohm
int LOAD_PIN3 = 4;  //     320 ohm
int LOAD_PIN4 = 5;  //     160 ohm
int LOAD_PIN5 = 6;  //      80 ohm
int LOAD_PIN6 = 7;  //      40 ohm
int LOAD_PIN7 = 8;  //      20 ohm
int LOAD_PIN8 = 9;  //      10 ohm
int SATIC_LOAD = 4; //Talk to Jack if you forget what this means Eli. 

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

const int LA_EXTEND = 120; //This should be the furthest out the linear actuator can move. Adjust as needed. 
const int LA_RETRACT = 80; //This should be the furthest in the linear actuator can move. Adjust as needed.
//All linear actuator movement needs to be bounded by these numbers.
const int LA_STEP_SIZE = 1; //Common step size for moving linear actuator. 
const int LA_HOME_POSITION = 102; //This is the home position of the linear actuator, here.
int la_current_position = 102; //This also acts as the starting position for la. Remeber to change this value whenever the linear actuator is moved.

const int BUFFER_SIZE = 50;
const int FLUSH_INTERVAL_MS = 5000;
const int SAMPLE_INTERVAL_MS = 100;

volatile bool is_recording = false;
char current_filename[20];

Servo linearActuator;

struct DataPoint {
    unsigned long timestamp;
    float rpm;
    int la_position;
};

DataPoint buffer_A[BUFFER_SIZE];
DataPoint buffer_B[BUFFER_SIZE];

DataPoint* write_buffer = buffer_A;
DataPoint* flush_buffer = buffer_B;

volatile int write_index = 0;
volatile bool flush_requested = false;

Threads::Mutex buffer_mutex;

void setup() {
  pinMode(IR_PIN, INPUT);
  Serial.begin(9600);


  linearActuator.attach(LA_PIN);
  linearActuator.write((LA_HOME_POSITION));
  pinMode(LA_EXTEND_BTN_PIN, INPUT_PULLUP);
  pinMode(LA_RETRACT_BTN_PIN, INPUT_PULLUP);
  
  setup_sd();  
  threads.addThread(sampling_thread);
  threads.addThread(flush_thread);
  pinMode(RECORD_BTN_PIN, INPUT_PULLUP);
}

void setup_sd(){
    if(!SD.begin(BUILTIN_SDCARD)){
        Serial.println("SD init has failed.");
        return;
    }
    Serial.println("SD setup was successful.");
}


//E-stop states methods 

  

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

    if(extendPressed && is_legal_la_step(LA_STEP_SIZE)){
        la_current_position += LA_STEP_SIZE;
        linearActuator.write(la_current_position);
        delay(15); //Allows actuator to catch up
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
//FLAG
bool is_legal_la_step(int step_size){
    return !(la_current_position + step_size > LA_EXTEND || la_current_position - step_size < LA_RETRACT);
}

bool is_legal_la_write(int write_value){
    return !(write_value > LA_EXTEND || write_value < LA_RETRACT);
}

  
/*
void record_rpm(); // write to sd card  

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
        f.println("timestamp_ms,rpm,la_position");
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

  

float get_voltage(); 

float get_current(); 

int lastState = HIGH;
unsigned long pulseTimes[5];
int pulseIndex = 0;
bool bufferFull = false;
int NUM_BLADES = 1;

float get_rpm() {
  int sensorVal = digitalRead(IR_PIN);

  if (lastState == HIGH && sensorVal == LOW) {
    unsigned long now = micros();
    pulseTimes[pulseIndex] = now;
    pulseIndex = (pulseIndex + 1) % NUM_BLADES;
    if (pulseIndex == 0) bufferFull = true;
  }

  lastState = sensorVal;

  if (!bufferFull) return 0.0;

  int oldest = pulseIndex;
  int newest = (pulseIndex + NUM_BLADES - 1) % NUM_BLADES;

  unsigned long revTime_us = pulseTimes[newest] - pulseTimes[oldest];

  if (revTime_us == 0) return 0.0;

  float revTime_sec = revTime_us / 1000000.0;
  return 60.0 / revTime_sec;
}

float get_windspeed(); 

  

  

//State transition methods 

  

  

//All random methods to be sorted later 

  

int change_load_resistence(float resistance); 

void loop(){
    handle_record_button();
    handle_actuator_buttons();
}

  

  

//How will we represent the data recorded during our testing time? 

// Table | RPM | voltage | current | 

//PRINT OUT ALL FUNCTIONS IN A CSV FORMAT.

```