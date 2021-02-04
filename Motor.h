#ifndef Motor_h
#define Motor_h
#include <Arduino.h>

class Motor {
	private:
		int pinEnable1;
		int pinEnable2;
		int pinPWM1;
		int pinPWM2;
		int pinEncoderA;
		int pinEncoderB;
		float ppr;
		float kp;
		float ki;
		float kd;
		int encoderCount = 0;
		float sumSpeedError = 0;
		float speedError = 0;
		float prevSpeedError = 0;
		float speedPoint = 0;
		float currentSpeed = 0;

	public:
    Motor();
		Motor(int pinEnable1, int pinEnable2, int pinPWM1, int pinPWM2, int pinEncoderA, int pinEncoderB, int ppr);
		void pinSetup();
    int getPinEncoder();
		float getCurrentSpeed();
		void setSpeed(float sp);
		void setPID(float kp, float ki, float kd);
		void setPWM(float pulse);
		void detectEncoder();
		void calculateSpeed(float timeInterval);
		void controlSpeed(float timeInterval);
		void reset();
};
#endif
