/*
   Teensy 4.1 — Hardware PWM on Pin 0 (FlexPWM4_2_A)
   Hardware inversion enabled so the 6N137 output is non‑inverted.
*/
 
#include <Arduino.h>
//#include "imxrt.h"
#include "imxrt_pmw.h"
 
// =========================
// CONFIGURATION
// =========================
const uint8_t PWM_PIN = 0;          // Teensy 4.1 pin 0 = FlexPWM4_2_A
const uint32_t PWM_FREQ = 50;       // 50 Hz for servo‑style timing
const uint16_t PWM_RES  = 16;       // 16‑bit resolution (0–65535)
 
// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);
  delay(500);
 
  // Set frequency and resolution
  analogWriteFrequency(PWM_PIN, PWM_FREQ);
  analogWriteResolution(PWM_RES);
 
  // =========================
  // HARDWARE INVERSION
  // =========================
  // Pin 0 = FlexPWM4, Submodule 2, Output A
  // Enable independent A/B control
  PWM4->SM[2].CTRL2 |= PWM_CTRL2_INDEP;
 
  // Invert A output (POLA bit)
  PWM4->SM[2].CTRL |= PWM_CTRL_POLA;
 
  Serial.println("PWM initialized with hardware inversion on pin 0.");
}
 
// =========================
// WRITE DUTY (0–65535)
// =========================
void writeDuty(uint16_t duty) {
  analogWrite(PWM_PIN, duty);
}
 
// =========================
// SERVO‑STYLE WRITE (1000–2000 us)
// =========================
void writeServoMicros(uint16_t us) {
  const uint32_t period_us = 20000;  // 20 ms frame
  uint32_t duty = (uint32_t(us) * 65535UL) / period_us;
  writeDuty(duty);
}
 
// =========================
// MAIN LOOP — DEMO
// =========================
void loop() {
  Serial.println("1000 us");
  writeServoMicros(1000);
  delay(2000);
 
  Serial.println("1500 us");
  writeServoMicros(1500);
  delay(2000);
 
  Serial.println("2000 us");
  writeServoMicros(2000);
  delay(2000);
}
