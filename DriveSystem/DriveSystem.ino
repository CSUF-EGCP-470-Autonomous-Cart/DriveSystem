/*
   Acceleration
   Encoders
*/

#include <Wire.h>
#include <Adafruit_MCP4725.h>
//https://www.pjrc.com/teensy/td_libs_Encoder.html
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>


//Must be before ros.h is imported
#define ROSSERIAL_ARDUINO_TCP
#include <ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>

#include <PID_v1.h>


//#define DEBUG
#define LOOP_RATE 10 //Hz

#define ENCODER_LEFT_A_PIN 2
#define ENCODER_LEFT_B_PIN 3
#define ENCODER_RIGHT_A_PIN 18
#define ENCODER_RIGHT_B_PIN 19

#define ACCEL_PID_P 2
#define ACCEL_PID_I 5
#define ACCEL_PID_D 1
#define accelSetpoint, accelInput, accelOutout;


Adafruit_MCP4725 dac;

Encoder leftWheel(ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN);
Encoder rightWheel(ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 3, 22);
IPAddress server(192 , 168, 3, 11);
const uint16_t serverPort = 11411;
ros::NodeHandle nh;

void CmdVelCb(const geometry_msgs::Twist& cmd_vel) {
  //TODO Update acceleration PID setpoint
}

ros::Subscriber<geometry_msgs::Twist> twist_sub("cmd_vel", CmdVelCb);
std_msgs::Header odom_header;
nav_msgs::Odometry odom;
ros::Publisher odom_pub("", &odom);

PID accelPID(&accelInput, &accelOutout, &accelSetpoint, ACCEL_PID_P, ACCEL_PID_I, ACCEL_PID_D, DIRECT);
void setup() {
  Serial.begin(115200);

  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
  // For MCP4725A2 the address is 0x64 or 0x65
  dac.begin(0x62);

  Ethernet.begin(mac, ip);
  Serial.print("Client IP: ");
  Serial.println(Ethernet.localIP());

  //wait for ethernet shield to initalize
  delay(1000);
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  nh.subscribe(twist_sub);
  nh.advertise(odom_pub);

  odom_header.seq = 0;

  accelPID.SetMode(AUTOMATIC);
}

unsigned long prevRateTime = 0;
void loop() {
  if((millis() - prevRateTime) > (1000 / LOOP_RATE)) {
    prevPollTime = millis();

    #ifdef DEBUG
      Serial.print("Left Encoder: "); Serial.println(leftWheel.read());
      Serial.print("Right Encoder: "); Serial.println(rightWheel.read());
      Serial.println();
    #endif

    odom_header.seq = odom_header.seq++;
    odom.header = odom_header;

    //TODO update odom message

    if (nh.connected()) {
      Serial.print("Conneced...");
      odom_pub.publish(&odom);
      Serial.println("published!");
    }
    else {
      Serial.println("Not conneced");
    }

  }

  //TODO acceleration PID
  //read current speed
  //accelInput = ??
  //update acceleration PID_v1
  accelPID.Compute();
  //update acceleration output
  dac.setVoltage(accelOutput, false);



  nh.spinOnce();
  delay(100);
}
