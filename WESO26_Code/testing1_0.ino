//NEED TO KNOW SPECIFIC LIBRARIES TO USE FOR EACH COMPONENT.
#include <servo.h>
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
int MOSI_PIN = 11
int MISO_PIN = 12;

int IR_PIN = 1;
int LA_PIN = 0;
int ESTOP_WRITE_PIN = 22;
int ESTOP_READ_PIN = 23;

int LA_EXTEND_BTN_PIN = 24;
int LA_RETRACT_BTN_PIN = 25;

const int LA_EXTEND = 120; //This should be the furthest out the linear actuator can move. Adjust as needed. 
const int LA_RETRACT = 80; //This should be the furthest in the linear actuator can move. Adjust as needed.
//All linear actuator movement needs to be bounded by these numbers.
const int LA_STEP_SIZE = 1; //Common step size for moving linear actuator. 
const int LA_HOME_POSITION = 102; //This is the home position of the linear actuator, here.
int la_current_position = 102; //This also acts as the starting position for la. Remeber to change this value whenever the linear actuator is moved.


Servo linearActuator;


void setup() {
  pinMode(IR_PIN, INPUT);
  Serial.begin(9600);


  linearActuator.attach(LA_PIN);
  linearActuator.write((LA_HOME_POSITION));
  pinMode(LA_EXTEND_BTN_PIN, INPUT_PULLUP);
  pinMode(LA_RETRACT_BTN_PIN, INPUT_PULLUP);
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

bool is_legal_la_step(int step_size){
    return !(la_current_position + step_size > LA_EXTEND || la_current_position - step_size < LA_RETRACT);
}

bool is_legal_la_write(int write_value){
    return !(write_value > LA_EXTEND || write_value < LA_RETRACT);
}

  

void record_rpm(); // write to sd card  

void record_voltage(); 

void record_current(); 

void record_windspeed(); //if pitot tube is here n all set up.  

void detect_load();

  

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

  

  

  

//How will we represent the data recorded during our testing time? 

// Table | RPM | voltage | current | 

//PRINT OUT ALL FUNCTIONS IN A CSV FORMAT.
