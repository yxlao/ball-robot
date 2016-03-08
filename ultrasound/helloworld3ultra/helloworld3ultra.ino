/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include<Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <NewPing.h>
//#include<AFMotor.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

//#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
//#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
//#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(12, 11, 300); // NewPing setup of pins and maximum distance.
NewPing front_sonar(8, 10, 300);

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

ros::NodeHandle nh;

std_msgs::String str_msg, arm_cmd;
std_msgs::UInt8 ultrasonic_msg;

ros::Publisher chatter("chatter", &str_msg);
ros::Publisher back_ultrasonic_pub("back_ultrasonic", &ultrasonic_msg);
ros::Publisher front_ultrasonic_pub("front_ultrasonic", &ultrasonic_msg);
//ros::Publisher ard_angle_pub("ard_angle", &ang_msg);


//char hello[13] = "hello world!";
char forward[8] = "forward";
char back[5] = "back";
unsigned int ultrasonic_dist;
int MOTOR_SPEED= 70;

void callback(const std_msgs::String& msg){
  uint8_t i;
  String msgdata(msg.data);
//  String w("w");
//  String s("s");
  arm_cmd.data = msg.data;
  if (strcmp(msg.data, "e") == 0){
    arm_cmd.data = forward;
    ////run the second motor
    myMotor2->run(FORWARD);
    myMotor2->setSpeed(MOTOR_SPEED);
//    delay(500);
//    while (angle > a){
////      chatter.publish("%4.2f", angle); 
//      nh.spinOnce();
//      delay(5);
//    }
//    myMotor2->run(RELEASE);
  } else if (strcmp(msg.data, "d") == 0) {
    arm_cmd.data = back;
    myMotor2->run(BACKWARD);
    myMotor2->setSpeed(MOTOR_SPEED);
//    delay(500);
//    while (angle < b) {
////      chatter.publish("%4.2f", angle); 
//      nh.spinOnce();
//      delay(5);
//    }
//    myMotor2->run(RELEASE);
//    delay(50);
  } 
//  myMotor2->run(RELEASE);
   if (strcmp(msg.data, "w") == 0){
    arm_cmd.data = forward;
    ////run the second motor
    myMotor->run(FORWARD);
    myMotor->setSpeed(MOTOR_SPEED);
//    delay(500);
//    while (angle > a){
////      chatter.publish("%4.2f", angle); 
//      nh.spinOnce();
//      delay(5);
//    }
//    myMotor->run(RELEASE);
  } else if (strcmp(msg.data, "s") == 0) {
    arm_cmd.data = back;
    myMotor->run(BACKWARD);
    myMotor->setSpeed(MOTOR_SPEED);
//    delay(500);
//    while (angle < b) {
////      chatter.publish("%4.2f", angle); 
//      nh.spinOnce();
//      delay(5);
//    }
//    myMotor->run(RELEASE);
//    delay(50);
  } else if (strcmp(msg.data, "o") == 0) {
    myMotor4->run(BACKWARD);
    myMotor4->setSpeed(100);
    delay(1500);
    myMotor4->run(RELEASE);
  }else if (strcmp(msg.data, "c") == 0) {
    myMotor4->run(FORWARD);
    myMotor4->setSpeed(100);
    delay(1500);
    myMotor4->run(RELEASE);
  }
  if (strcmp(msg.data, "f") == 0){
    arm_cmd.data = forward;
    ////run the second motor
    myMotor3->run(FORWARD);
    myMotor3->setSpeed(MOTOR_SPEED);
//    delay(500);
//    while (angle > a){
////      chatter.publish("%4.2f", angle); 
//      nh.spinOnce();
//      delay(5);
//    }
//    myMotor3->run(RELEASE);
  } else if (strcmp(msg.data, "r") == 0) {
    arm_cmd.data = back;
    myMotor3->run(BACKWARD);
    myMotor3->setSpeed(MOTOR_SPEED);
//    delay(500);
//    while (angle < b) {
////      chatter.publish("%4.2f", angle); 
//      nh.spinOnce();
//      delay(5);
//    }
//    myMotor3->run(RELEASE);
//    delay(50);
  }

   if (strcmp(msg.data, "t") == 0){
    arm_cmd.data = forward;
    ////run the second motor
    myMotor4->run(FORWARD);
    myMotor4->setSpeed(MOTOR_SPEED);
//    delay(500);
//    while (angle > a){
////      chatter.publish("%4.2f", angle); 
//      nh.spinOnce();
//      delay(5);
//    }
//    myMotor4->run(RELEASE);
  } else if (strcmp(msg.data, "g") == 0) {
    arm_cmd.data = back;
    myMotor4->run(BACKWARD);
    myMotor4->setSpeed(MOTOR_SPEED);
//    delay(500);
//    while (angle < b) {
////      chatter.publish("%4.2f", angle); 
//      nh.spinOnce();
//      delay(5);
//    }
//    myMotor4->run(RELEASE);
//    delay(50);
  }
//  myMotor2->run(RELEASE);
  if (strcmp(msg.data, "x") == 0) {
    myMotor->run(RELEASE);
    myMotor2->run(RELEASE);
    myMotor3->run(RELEASE);
    myMotor4->run(RELEASE);
    
  }
  

  chatter.publish(&arm_cmd);
  delay(50);
  
}

ros::Subscriber<std_msgs::String> cmds("arm_cmds", &callback);
void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  nh.advertise(back_ultrasonic_pub);
  nh.advertise(front_ultrasonic_pub);
//  nh.advertise(ard_angle_pub);
  nh.subscribe(cmds);
    Serial.begin(57600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(20);
  myMotor2->setSpeed(20);
  myMotor3->setSpeed(20);
  myMotor4->setSpeed(20);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);
  myMotor2->run(FORWARD);
  // turn on motor
  myMotor2->run(RELEASE);
    myMotor3->run(FORWARD);
  // turn on motor
  myMotor3->run(RELEASE);
  myMotor4->run(FORWARD);
  // turn on motor
  myMotor4->run(RELEASE);
}

void loop()
{
//  str_msg.data = to_string(angle);
//  chatter.publish( &str_msg );
  ultrasonic_dist = sonar.ping_cm();
  ultrasonic_msg.data = ultrasonic_dist;
  back_ultrasonic_pub.publish(&ultrasonic_msg);
  
  ultrasonic_dist = front_sonar.ping_cm();
  ultrasonic_msg.data = ultrasonic_dist;
  front_ultrasonic_pub.publish(&ultrasonic_msg);
  nh.spinOnce();
  delay(300);
}

