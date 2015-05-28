//Current Speed Variables
float curSpeedX = 0; //Current Translational Speed
float curSpeedW = 0; //Current Rotational Speed
//Target Speed Variables
int targetSpeedX = 0; //Target Translational Speed
int targetSpeedW = 0; //Target Rotational Speed
//Encoder Feedback Variables
int encoderFeedbackX = 0; //Translational Encoder Feedback
int encoderFeedbackW = 0; //Rotational Encoder Feedback
//Error Variables
float posErrorX = 0;
float posErrorW = 0;
float oldPosErrorX = 0;
float oldPosErrorW = 0;
//PWM Variables
int posPwmX = 0;
int posPwmW = 0;
//PID Variables
float pidInputX = 0;
float pidInputW = 0;
float kpX = 2, kdX = 4;
float kpW = 1, kdW = 12;
float kpW1 = 1;
float kdW1 = 26;
float kpW2 = 1;
float kdW2 = 36;
//Acceleration Variables
float accX = 600;
float decX = 600;
float accW = 1;
float decW = 1;
//Speed Settings
int moveSpeed = speed_to_counts(500*2);
int turnSpeed = speed_to_counts(500*2);
int returnSpeed = speed_to_counts(500*2);
int stopSpeed = speed_to_counts(100*2);
int maxSpeed = speed_to_counts(2000*2);
//Gyro Variables
int gyroFeedbackRatio = 5700;

/* * * * * * * * * *
 * Systick Handler *
 * * * * * * * * * */
void systick()
{
	millis++;
	if (b_useIR)
	{
		readSensor();
	}
	if (b_useGyro)
		readGyro();
	}
	if (b_useSpeedProfile)
	{
		speedProfile();
	}
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
 *          Update Current Speed         *
 *  Update speed variables to accelerate *
 *        or decelerate as needed       *
 * * * * * * * * * * * * * * * * * * * * */
void updateCurrentSpeed()
{
	if (curSpeedX < targetSpeedX)
	{
		curSpeedX += (float)(speed_to_counts(accX*2)/100);
		if (curSpeedX > targetSpeedX)
		{
			curSpeedX = targetSpeedX;
		}
	}
	else if (curSpeedX > targetSpeedX)
	{
		curSpeedX -= (float)(speed_to_counts(decX*2)/100);
		if (curSpeedX < targetSpeedX)
		{
			curSpeedX = targetSpeedX;
		}
	}
	if (curSpeedW < targetSpeedW)
	{
		curSpeedW += accW;
		if (curSpeedW > targetSpeedW)
		{
			curSpeedW = targetSpeedW;
		}
	}
	else if (curSpeedW > targetSpeedW)
	{
		curSpeedW -= decW;
		if (curSpeedW < targetSpeedW)
		{
			curSpeedW = targetSpeedW;
		}
	}
}

/* * * * * * * * * * * * * * * *
 *      Calculate Motor PWM    *
 *   Take speed variables and  *
 *  calculate motor pwm values *
 * * * * * * * * * * * * * * * */
 void calculateMotorPwm()
 {
 	int gyroFeedback;
 	int rotationalFeeback;
 	int sensorFeedback;
 	int leftBaseSpeed;
 	int rightBaseSpeed;

 	encoderFeedbackX = rightEncoderChange + leftEncoderChange;
 	encoderFeedbackW = rightEncoderChange - leftEncoderChange;

 	gyroFeedback = aSpeed/gyroFeedbackRatio;
 	sensorFeedback = sensorError/a_scale;

 	if (onlyUseGyroFeedback)
 		rotationalFeedback = gyroFeedback;
 	else if (onlyUseEncoderFeedback)
 		rotationalFeedback = encoderFeedbackW;
 	else
 		rotationalFeedback = encoderFeedbackW + gyroFeedback;

 	posErrorX += curSpeedX - encoderFeedbackX;
 	posErrorW += curSpeedW - rotationalFeedback;

 	posPwmX = (kpX * posErrorX) + (kdX * (posErrorX - oldPosErrorX));
 	posPwmW = (kpW * posErrorW) + (kdW * (posErrorW - oldPosErrorW));

 	oldPosErrorX = posErrorX;
 	oldPosErrorW = posErrorW;

 	leftBaseSpeed = posPwmX - posPwmW;
 	rightBaseSpeed = posPwmX + posPwmW;

 	setLeftPwm(leftBaseSpeed);
 	setRightPwm(rightBaseSpeed);
 }