#include <Wire.h>
#include <Adafruit_MCP4725.h>

/*

   Steering
   Braking
   Acceleration
   Encoders

*/

#define STEER_DIRECTION_PIN 4
#define STEER_STEP_PIN 5
#define STEER_ENABLE_PIN 6
#define STEER_POT_PIN A0

#define BRAKE_A_PIN 7
#define BRAKE_B_PIN 8

#define ENCODER_LEFT_A_PIN 2
#define ENCODER_LEFT_B_PIN 3
#define ENCODER_RIGHT_A_PIN 18
#define ENCODER_RIGHT_B_PIN 19


Adafruit_MCP4725 dac;

void setup() {
  Serial.begin(115200);

  pinMode(STEER_DIRECTION_PIN, OUTPUT);
  pinMode(STEER_STEP_PIN, OUTPUT);
  pinMode(STEER_ENABLE_PIN, OUTPUT);

  pinMode(BRAKE_A_PIN, OUTPUT);
  pinMode(BRAKE_B_PIN, OUTPUT);

  pinMode(ENCODER_LEFT_A_PIN, INPUT);
  pinMode(ENCODER_LEFT_B_PIN, INPUT);
  pinMode(ENCODER_RIGHT_A_PIN, INPUT);
  pinMode(ENCODER_RIGHT_B_PIN, INPUT);

  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
  // For MCP4725A2 the address is 0x64 or 0x65
  dac.begin(0x62);

}

void loop() {
  // put your main code here, to run repeatedly:
}
