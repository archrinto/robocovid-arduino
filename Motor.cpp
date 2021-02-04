#include <Arduino.h>
#include "Motor.h"

Motor::Motor() {
  
}

Motor::Motor(int pinEnable1, int pinEnable2, int pinPWM1, int pinPWM2, int pinEncoderA, int pinEncoderB, int ppr) {
	this->pinEnable1 = pinEnable1;
	this->pinEnable2 = pinEnable2;
	this->pinPWM1 = pinPWM1;
	this->pinPWM2 = pinPWM2;
	this->pinEncoderA = pinEncoderA;
	this->pinEncoderB = pinEncoderB;
	this->ppr = ppr;
	
	this->pinSetup();
}

void Motor::pinSetup() {
	pinMode(pinEnable1, OUTPUT);
	pinMode(pinEnable2, OUTPUT);
	pinMode(pinPWM1, OUTPUT);
	pinMode(pinPWM2, OUTPUT);
  pinMode(pinEncoderB, INPUT);
  pinMode(pinEncoderA, INPUT);

	setPWM(0);
  // digitalWrite(pinEnable,0);
}

static void interrupt(void *arg) {
  ((Motor *)arg)->detectEncoder();
}

void Motor::detectEncoder() {
	if(digitalRead(pinEncoderB) == 0){
		encoderCount +=1;
	} else {
		encoderCount -=1;
	}
}

int Motor::getPinEncoder() {
  return pinEncoderA;
}

void Motor::setSpeed(float speed) {
	speedPoint = speed;
}

void Motor::setPID(float kp, float ki, float kd) {
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
}

void Motor::setPWM(float pulse) {
	if (pulse > 255) pulse = 255;
	if (pulse < -255) pulse = -255;
	if (pulse > 0){
//    Serial.print(pulse);
//    Serial.println(">0");
		analogWrite(pinPWM1, pulse);  //set motor speed  
		analogWrite(pinPWM2, 0);
	} else if( pulse < 0){
//    Serial.print(pulse);
//    Serial.println("<0");
		analogWrite(pinPWM1, 0);  //set motor speed  
		analogWrite(pinPWM2, pulse*-1);
	} else {
		analogWrite(pinPWM1, 0);  //stop motor
		analogWrite(pinPWM2, 0);  //stop motor
	}
	digitalWrite(pinEnable1,1);
	digitalWrite(pinEnable2,1);
}

void Motor::calculateSpeed(float timeInterval) {
	currentSpeed = 60.0 * (encoderCount/ ppr) / timeInterval; // RPM
	encoderCount = 0;
}

void Motor::controlSpeed(float timeInterval) {
	calculateSpeed(timeInterval);
	speedError = speedPoint - currentSpeed;
	float pwmPulse = speedError * kp + sumSpeedError * ki + (speedError - prevSpeedError) * kd;
	prevSpeedError = speedError;
	sumSpeedError += speedError;

	if (sumSpeedError > 5000) sumSpeedError = 5000;
	if (sumSpeedError < -5000) sumSpeedError = -5000;

	setPWM(pwmPulse);
}

float Motor::getCurrentSpeed() {
	return currentSpeed;
}
void Motor::reset() {
  	digitalWrite(pinEnable1,  0);
	digitalWrite(pinEnable2,  0);
	speedPoint = 0;
	sumSpeedError = 0;
	speedError = 0;
	prevSpeedError = 0;
	currentSpeed = 0;
	encoderCount = 0;
}
