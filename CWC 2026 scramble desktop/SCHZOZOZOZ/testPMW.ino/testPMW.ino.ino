// test_la_analog.ino
//DUTY LOWER BOUND: 3816
//DUTY UPPER BOUND: 3891
const int LA_PIN = 0;

void set_la(int DUTY) {
    //pulse_us = constrain(pulse_us, 1000, 2000);
    analogWriteResolution(12);
    analogWriteFrequency(LA_PIN, 50);
    analogWrite(LA_PIN, DUTY);
    //Serial.print("Pulse us: "); Serial.println(pulse_us);
    Serial.print("DUTY: "); Serial.println(DUTY);
    
}

void setup() {
    Serial.begin(9600);
    pinMode(LA_PIN, OUTPUT);
    analogWriteFrequency(LA_PIN, 50);
    Serial.println("Analog LA test ready. Enter 1000-2000.");
}

void loop() {
    if (Serial.available() > 0) {
        int pulse = Serial.parseInt();
        while (Serial.available() > 0) Serial.read();
        set_la(pulse);
    }
}