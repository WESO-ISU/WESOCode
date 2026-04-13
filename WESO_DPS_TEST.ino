//NEED TO KNOW SPECIFIC LIBRARIES TO USE FOR EACH COMPONENT.
#include <Servo.h>
#include <SD.h>
#include <TeensyThreads.h>
#include <stdint.h>
#include <SPI.h>

//Differential Pressor Sensor (DPS) setup stuff here
const int WIND_SENSOR_PIN = 14;
const int ADC_RESOLUTION = 4096;
const float ADC_VREF = 3.3f;
const float R1 = 47.0f; // Top resistor value - subject to change - not currently accurate
const float R2 = 47.0f; // Bottom resistor value - subject to change - not currently accurate
const float DIVIDER_FACTOR = (R1 + R2) / R2; 
const float VS = 5.0f;
//const float AIR_DENSITY_IA = 1.197f; //Magic number: this is the air density in Ames Iowa, or places with ~292m of elevation
//const float AIR_DENSITY_CO = 1.045f; //Magic number: this is the air density in Boulder Colorado, or places with ~1655m of elevation
const float AIR_DENSITY = 1.197f;      //^ 
const int NUM_SAMPLES = 10; //Amount of samples to average the reading for windspeed




void setup(){
    Serial.begin(9600);
    setup_dps();
}

void setup_dps(){
    analogReadResolution(12);
    pinMode(WIND_SENSOR_PIN, INPUT);
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

void loop(){
    
    Serial.println("Windspeed ms: " + get_windspeed());
    delay(250); //ms delay so I don't wanna kms
}