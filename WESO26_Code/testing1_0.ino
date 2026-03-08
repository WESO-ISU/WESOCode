//NEED TO KNOW SPECIFIC LIBRARIES TO USE FOR EACH COMPONENT.
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

void setup() {
  pinMode(IR_PIN, INPUT);
  Serial.begin(9600);
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
int NUM_BLADES = 5;

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
