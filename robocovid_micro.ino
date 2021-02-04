/*
    PWM1 -> CW
    PWM2 -> CCW
*/

#define USE_USBCON

#include <ros.h>
#include <robocovid_msgs/Motion.h>
#include "Motor.h"
#include <DueTimer.h>
#include <Servo.h>

#define PIN_M1_PWM1 2
#define PIN_M1_PWM2 3
#define PIN_M1_ENA1 24
#define PIN_M1_ENA2 30
#define PIN_M1_ENC1 A0
#define PIN_M1_ENC2 A1

#define PIN_M2_PWM1 4
#define PIN_M2_PWM2 5
#define PIN_M2_ENA1 32
#define PIN_M2_ENA2 38
#define PIN_M2_ENC1 A2
#define PIN_M2_ENC2 A3

#define PIN_M3_PWM1 6
#define PIN_M3_PWM2 7
#define PIN_M3_ENA1 40
#define PIN_M3_ENA2 46
#define PIN_M3_ENC1 A4
#define PIN_M3_ENC2 A5

float motor1, motor2, motor3;
int servo_sudut_x = 90;
int servo_sudut_y = 90;

Motor m1;
Motor m2;
Motor m3;

Servo servoX;
Servo servoY;

ros::NodeHandle nh;
robocovid_msgs::Motion nilai_msg;

void motionCb(const robocovid_msgs::Motion& motion_msg) {
  motor1 = int(motion_msg.rpm1.data);
  motor2 = int(motion_msg.rpm2.data);
  motor3 = int(motion_msg.rpm3.data);
  servo_sudut_x += int(motion_msg.servo1.data);
  servo_sudut_y += int(motion_msg.servo2.data);

  setMotorSpeed(motor1, motor2, motor3);
}

ros::Subscriber<robocovid_msgs::Motion> motion("robocovid_motion", motionCb);

void setup() {
  nh.initNode();
  nh.subscribe(motion);
  Serial.begin(115200);

  servoX.attach(51);
  servoY.attach(49);

  m1 = Motor(PIN_M1_ENA1, PIN_M1_ENA2, PIN_M1_PWM1, PIN_M1_PWM2, PIN_M1_ENC1, PIN_M1_ENC2, 135);
  m2 = Motor(PIN_M2_ENA1, PIN_M2_ENA2, PIN_M2_PWM1, PIN_M2_PWM2, PIN_M2_ENC1, PIN_M2_ENC2, 135);
  m3 = Motor(PIN_M3_ENA1, PIN_M3_ENA2, PIN_M3_PWM1, PIN_M3_PWM2, PIN_M3_ENC1, PIN_M3_ENC2, 135);

  //  m1.setPID(0.15, 0.11, 0.03);
  //  m2.setPID(0.15, 0.11, 0.03);
  //  m3.setPID(0.15, 0.11, 0.03);

//  attachInterrupt(digitalPinToInterrupt(m1.getPinEncoder()), m1DetectEncoder, RISING);
//  attachInterrupt(digitalPinToInterrupt(m2.getPinEncoder()), m2DetectEncoder, RISING);
//  attachInterrupt(digitalPinToInterrupt(m3.getPinEncoder()), m3DetectEncoder, RISING);
//
//  m1.reset();
//  m2.reset();
//  m3.reset();

 // Timer8.attachInterrupt(localControl0).start(50000);

  //setMotorSpeed(-20, 20, 20);
}

void loop() {
  servoY.write(servo_sudut_y);
  servoX.write(servo_sudut_x);
  nh.spinOnce();
  delay(15);
}

void setMotorSpeed(float motor1, float motor2, float motor3) {
//  m1.setSpeed(motor1);
//  m2.setSpeed(motor2);
//  m3.setSpeed(motor3);
  m1.setPWM(motor1);
  m2.setPWM(motor2);
  m3.setPWM(motor3);
}

void m1DetectEncoder() {
  m1.detectEncoder();
}

void m2DetectEncoder() {
  m2.detectEncoder();
}

void m3DetectEncoder() {
  m3.detectEncoder();
}

void localControl0() {
  m1.controlSpeed(0.05);
  m2.controlSpeed(0.05);
  m3.controlSpeed(0.05);
}
