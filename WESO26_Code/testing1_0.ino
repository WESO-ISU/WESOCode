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

}


//E-stop states methods 

  

void e_stop(); //change linear actuator to the stopping angle.  

  

//Start-up state methods 

  

//Active reading state methods 
void active_reading();

void set_pitch_angle(); //Requires analogWrite()

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

float get_rpm(); //probably could just be an int 

float get_windspeed(); 

  

  

//State transition methods 

  

  

//All random methods to be sorted later 

  

int change_load_resistence(float resistance); 

  

  

  

//How will we represent the data recorded during our testing time? 

// Table | RPM | voltage | current | 

//PRINT OUT ALL FUNCTIONS IN A CSV FORMAT.
