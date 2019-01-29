#include <Wire.h>
#include <Adafruit_MCP4725.h>
//https://www.pjrc.com/teensy/td_libs_Encoder.html
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

/*

   Steering
   Braking
   Acceleration
   Encoders

*/

//#define DEBUG

#define LOOP_RATE 10 //Hz

#define STEER_DIRECTION_PIN 4
#define STEER_STEP_PIN 5
#define STEER_POT_PIN A0

#define BRAKE_A_PIN 7
#define BRAKE_B_PIN 8

#define ENCODER_LEFT_A_PIN 2
#define ENCODER_LEFT_B_PIN 3
#define ENCODER_RIGHT_A_PIN 18
#define ENCODER_RIGHT_B_PIN 19


Adafruit_MCP4725 dac;

Encoder leftWheel(ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN);
Encoder rightWheel(ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);

void setup() {
  Serial.begin(115200);

  pinMode(STEER_DIRECTION_PIN, OUTPUT);
  pinMode(STEER_STEP_PIN, OUTPUT);
  pinMode(STEER_ENABLE_PIN, OUTPUT);

  pinMode(BRAKE_A_PIN, OUTPUT);
  pinMode(BRAKE_B_PIN, OUTPUT);

  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
  // For MCP4725A2 the address is 0x64 or 0x65
  dac.begin(0x62);

}

unsigned long prevRateTime = 0;
void loop() {
  if((millis() - prevRateTime) > (1000 / LOOP_RATE)) {
    #ifdef DEBUG
      Serial.print("Left Encoder: "); Serial.println(leftWheel.read());
      Serial.print("Right Encoder: "); Serial.println(rightWheel.read());
      Serial.println();
    #endif
  }
}
