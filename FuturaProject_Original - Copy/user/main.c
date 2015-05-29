#include "stm32f4xx.h"
#include "delay.h"
#include "led.h"
#include "button.h"
#include <stdio.h>
#include "usart.h"
#include "SPI.h"
#include "matrixDisplay.h"
#include "pwm.h"
#include "encoder.h"
#include "buzzer.h"
#include "main.h"
#include "sensor_Function.h"
#include "adc.h"

/* * * * * * * * * *
 * Setup Variables *
 * * * * * * * * * */
//Current Speed Variables
float curSpeedX = 0; //Current Translational Speed
float curSpeedW = 0; //Current Rotational Speed
//Target Speed Variables
int targetSpeedX = 0; //Target Translational Speed
int targetSpeedW = 0; //Target Rotational Speed
//Encoder Feedback Variables
int encoderFeedbackX = 0; //Translational Encoder Feedback
int encoderFeedbackW = 0; //Rotational Encoder Feedback
//Encoder Variables
int leftEncoder; 
int rightEncoder;
int leftEncoderChange;
int rightEncoderChange;
int encoderChange;
int leftEncoderOld;
int rightEncoderOld;
int leftEncoderCount;
int rightEncoderCount;
int encoderCount;
int distanceLeft;
//Error Variables
float posErrorX = 0;
float posErrorW = 0;
float oldPosErrorX = 0;
float oldPosErrorW = 0;
//PWM Variables
float posPwmX = 0;
float posPwmW = 0;

/* * * * * * * * * * * * *
 * Adjustable Variables  *
 * * * * * * * * * * * * */

//Feedback Control Variables
bool onlyUseGyroFeedback = false;
bool onlyUseEncoderFeedback = false;
//Sensor Variables
bool b_useIR = true;
bool b_useGyro = true;
bool b_useSpeedProfile = true;
//PID Variables
float pidInputX = 0;
float pidInputW = 0;
float kpX = .05, kdX = 0;
float kpW = 0, kdW = 0;
float kpW1 = 1;
float kdW1 = 26;
float kpW2 = 1;
float kdW2 = 36;
//Acceleration Variables
//temp
float accTrans = 2;
float decTrans = 2;
float accX = 600;
float decX = 600;
float accW = 1;
float decW = 1;
//Speed Settings
/*
int moveSpeed = speed_to_counts(500*2);
int turnSpeed = speed_to_counts(500*2);
int returnSpeed = speed_to_counts(500*2);
int stopSpeed = speed_to_counts(100*2);
int maxSpeed = speed_to_counts(2000*2);
*/
//Gyro Variables
int gyroFeedbackRatio = 5700;
//Analog Scale (?)
int a_scale = 1;
//IR Sensor Calibration
int DLMiddleValue;
int DRMiddleValue;
int sensorError = 0;


/* * * * * * * * * * * * * 
 * Function Declarations *
 * * * * * * * * * * * * */
void getSensorError(void);
void speedProfile(void);
void getEncoderStatus(void);
void calculateMotorPwm(void);
void updateCurrentSpeed(void);
int speed_to_counts(int accel);


/* * * * * * * * * *
 * Systick Handler *
 * * * * * * * * * */
void systick()
{
	if (b_useIR)
	{
		readSensor();
	}
	if (b_useGyro)
	{
		readGyro();
	}
	if (b_useSpeedProfile)
	{
		speedProfile();
	}
}

/* * * * * * * * * * * * *
 * Caculate Sensor Error *
 * * * * * * * * * * * * */
void getSensorError(void)//the very basic case
{
	if(DLSensor > DLMiddleValue && DRSensor < DRMiddleValue)
		sensorError = DLMiddleValue - DLSensor;
	else if(DRSensor > DRMiddleValue && DLSensor < DLMiddleValue)
		sensorError = DRSensor - DRMiddleValue;
	else
		sensorError = 0;
}

/* * * * * * * * * * * * * * * * * * * *
 *            Speed Profile            *
 *     Calculate what current speed    *
 * for each wheel should be and set it *
 * * * * * * * * * * * * * * * * * * * */
void speedProfile()
{
	getEncoderStatus();
	updateCurrentSpeed();
	calculateMotorPwm();
}

/* * * * * * * * * * * * * * * * * * * * *
 *           Get Encoder Status          *
 *  Determine how much further to travel *
 * * * * * * * * * * * * * * * * * * * * */

void getEncoderStatus()
{
	leftEncoder = getLeftEncCount();
	rightEncoder = getRightEncCount();

	leftEncoderChange = leftEncoder - leftEncoderOld;
	rightEncoderChange = rightEncoder - rightEncoderOld;
	encoderChange = (leftEncoderChange + rightEncoderChange)/2;
	leftEncoderOld = leftEncoder;
	rightEncoderOld = rightEncoder;

	leftEncoderCount += leftEncoderChange;
	rightEncoderCount += rightEncoderChange;
	encoderCount = (leftEncoderCount + rightEncoderCount)/2;

	distanceLeft -= encoderChange;
}

/* * * * * * * * * * * * * * * * * * * * *
 *           Update Current Speed        *
 *  Determine how much further to travel *
 * * * * * * * * * * * * * * * * * * * * */

void updateCurrentSpeed(void)
{
	//printf("Current Speed = %f\r\n", curSpeedX);
	if(curSpeedX < targetSpeedX)
	{
		curSpeedX += 0.5;//accTrans;
		if(curSpeedX > targetSpeedX)
			curSpeedX = targetSpeedX;
	}
	else if(curSpeedX > targetSpeedX)
	{
		curSpeedX -= 0.5;//decTrans;
		if(curSpeedX < targetSpeedX)
			curSpeedX = targetSpeedX;
	}
	if(curSpeedW < targetSpeedW)
	{
		curSpeedW += accW;
		if(curSpeedW > targetSpeedW)
			curSpeedW = targetSpeedW;
	}
	else if(curSpeedW > targetSpeedW)
	{
		curSpeedW -= decW;
		if(curSpeedW < targetSpeedW)
			curSpeedW = targetSpeedW;
	}	
	//printf("curSpeedX %f targetSpeedX %d \r\n", curSpeedX, targetSpeedX);
}
/* * * * * * * * * * * * * * * *
 *      Calculate Motor PWM    *
 *   Take speed variables and  *
 *  calculate motor pwm values *
 * * * * * * * * * * * * * * * */
void calculateMotorPwm()
{
 	int gyroFeedback;
 	int rotationalFeedback;
 	int sensorFeedback;
 	int leftBaseSpeed;
 	int rightBaseSpeed;

 	encoderFeedbackX = rightEncoderChange + leftEncoderChange;
 	encoderFeedbackW = rightEncoderChange - leftEncoderChange;

 	gyroFeedback = aSpeed/gyroFeedbackRatio; 
 	sensorFeedback = sensorError/a_scale; //what is a_scale?

 	if (onlyUseGyroFeedback)
 		rotationalFeedback = gyroFeedback;
 	else if (1)//onlyUseEncoderFeedback)
 		rotationalFeedback = encoderFeedbackW;
 	else
 		rotationalFeedback = encoderFeedbackW + gyroFeedback;
	printf("Encoder FeedbackX = %d\r\n", encoderFeedbackX);
 	posErrorX += curSpeedX - encoderFeedbackX;
 	posErrorW += curSpeedW - rotationalFeedback;

 	posPwmX = (kpX * posErrorX) + (kdX * (posErrorX - oldPosErrorX));
 	posPwmW = (kpW * posErrorW) + (kdW * (posErrorW - oldPosErrorW));
	
 	oldPosErrorX = posErrorX;
 	oldPosErrorW = posErrorW;
	
 	leftBaseSpeed = posPwmX - posPwmW;
 	rightBaseSpeed = posPwmX + posPwmW;
	//printf("targetX = %d\tLeft Speed = %d\tRight Speed = %d", targetSpeedX, leftBaseSpeed, rightBaseSpeed);
 	setLeftPwm(leftBaseSpeed);
 	setRightPwm(rightBaseSpeed);
}
int speed_to_counts(int accel)
{
		return accel;
}

/* * * * * * * * * * * * * * * * * * * *
 * Basic Movements										 *
 * 1. forwardDistance								   *
 *										                 *
 * * * * * * * * * * * * * * * * * * * */
void forwardDistance(int distance, int left_speed, int right_speed, bool coast) 
{
	distanceLeft = distance;
	/*
	int curEnc = getLeftEncCount();
	turning = 0;
	while (getLeftEncCount() - curEnc < distance) {
		left_enc = getLeftEncCount();
		right_enc = getRightEncCount(); 
		//printf("Encoder counts left: %d\r\n",distance - getLeftEncCount() + curEnc); 
		//displayMatrix("FWD");
		targetLeft = left_speed;
		targetRight = right_speed;
	}

	if (!coast) {
		targetLeft = 0;
		targetRight = 0;
	}
	turning = 1;*/
	while (distanceLeft>=0) 
	{
		//targetSpeedX = 100;
		//targetSpeedW = 0;
		setLeftPwm(100);
		setRightPwm(100);
	}
		setLeftPwm(0);
		setRightPwm(0);
	//targetSpeedX = 0;
	//targetSpeedW = 0;
}

void button1_interrupt(void) 
{
}

void button2_interrupt(void) 
{
}

int main(void) 
{
	Systick_Configuration();
	LED_Configuration();
	button_Configuration();
	usart1_Configuration(9600);
	SPI_Configuration();
  TIM4_PWM_Init();
	Encoder_Configration();
	buzzer_Configuration();
	ADC_Config();
	
	//curSpeedX = 0;
	//curSpeedW = 0;
	//shortBeep(2000, 8000);
	/*
	while(1) 
	{
		readSensor();
		readGyro();
		readVolMeter();
		printf("LF %d RF %d DL %d DR %d aSpeed %d angle %d voltage %d lenc %d renc %d\r\n", LFSensor, RFSensor, DLSensor, DRSensor, aSpeed, angle, voltage, getLeftEncCount(), getRightEncCount());
		displayMatrix("UCLA");
		
		setLeftPwm(100);
		setRightPwm(100);
		delay_ms(1000);
	}*/
	//forwardDistance(4000,0,0,true);
	//displayMatrix("Sped");
	//targetSpeedX = 100;
	//delay_ms(2000);
	
	displayMatrix("DSPD");
	targetSpeedX = 1000;
	delay_ms(2000);
	printf("==============================================\n\r=======================================================\r\n");
	displayMatrix("STOP");
	targetSpeedX = 0;
	delay_ms(10000);
}
