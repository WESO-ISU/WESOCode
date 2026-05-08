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

const bool LOAD_ACTIVE_HIGH = true;

const int LOAD_PINS[8] = {
    LOAD_PIN8, LOAD_PIN7, LOAD_PIN6, LOAD_PIN5,
    LOAD_PIN4, LOAD_PIN3, LOAD_PIN2, LOAD_PIN1
};

int loadArray[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void setup() {
    Serial.begin(9600);
    setup_loads();
}

void setup_loads() {
    for (int i = 0; i < 8; i++) {
        pinMode(LOAD_PINS[i], OUTPUT);
        digitalWrite(LOAD_PINS[i], LOAD_ACTIVE_HIGH ? LOW : HIGH); // default all off
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

void loadTestingHandler(){
    loadArray[7] = 1;
    loadLoad();
    delay(1000);
    
    loadArray[7] = 0;
    loadArray[6] = 1;
    loadLoad();
    delay(1000);

    loadArray[6] = 0;
    loadArray[5] = 1;
    loadLoad();
    delay(1000);

    loadArray[5] = 0;
    loadArray[4] = 1;
    loadLoad();
    delay(1000);
    
    loadArray[4] = 0;
    loadArray[3] = 1;
    loadLoad();
    delay(1000);

    loadArray[3] = 0;
    loadArray[2] = 1;
    loadLoad();
    delay(1000);

    loadArray[2] = 0;
    loadArray[1] = 1;
    loadLoad();
    delay(1000);

    loadArray[1] = 0;
    loadArray[0] = 1;
    loadLoad();
    delay(1000);
} 

void loop() {
    loadTestingHandler();
    delay(5000);
}
