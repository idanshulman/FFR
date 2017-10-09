//For Gyro
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
//Other
#include<math.h>

//Motors
#define MOTOR_LF 3
#define MOTOR_LB 4
#define MOTOR_RF 5
#define MOTOR_RB 6

//US_OUT - Trigger Pins
#define Trig_FL 35
#define Trig_RF 45
#define Trig_RB 47
#define Trig_LB 49
#define Trig_LF 51
#define Trig_FR 53

//US_IN - Echo Pins
#define Echo_FL 34
#define Echo_RF 44
#define Echo_RB 46
#define Echo_LB 48
#define Echo_LF 50
#define Echo_FR 52

//IR
#define IR_LF A1
#define IR_RF A2
#define IR_FR A7
#define IR_FL A8
#define IR_BL A9
#define IR_BR A10

//MC-Avram mic
#define MC 37

//fan
#define FAN 43

//UV
#define UV 42

//Gyro
MPU6050 mpu;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float euler[3];
float lastAngle = 0;

int resetAngle = 0;

//Gloabl defines
#define FINISHED true

//Stages defines
#define NONE 0
#define ALIGN 1
#define TURN_RIGHT 2
#define FOLLOW_WALL 3


//Globals
bool aligned = false; //is aligned after first alignment
bool startDirection = false; //true = facing dog's possible direction
int stage = ALIGN;

//Function declaration
int USRead(int trigPin, int echoPin);
int MoveMotor(int motorPort, int power);
void GyroInit();
int GyroRead();
float GetAngle();
void GyroReset();
bool align(int drivePower);

void setup()
{
	//US_OUT
	pinMode(Trig_FL, OUTPUT);
	pinMode(Trig_RF, OUTPUT);
	pinMode(Trig_RB, OUTPUT);
	pinMode(Trig_LB, OUTPUT);
	pinMode(Trig_LF, OUTPUT);
	pinMode(Trig_FR, OUTPUT);
	//US_IN
	pinMode(Echo_FL, INPUT);
	pinMode(Echo_RF, INPUT);
	pinMode(Echo_RB, INPUT);
	pinMode(Echo_LB, INPUT);
	pinMode(Echo_LF, INPUT);
	pinMode(Echo_FR, INPUT);
	//IR
	pinMode(IR_LF, INPUT);
	pinMode(IR_RF, INPUT);
	pinMode(IR_FR, INPUT);
	pinMode(IR_FL, INPUT);
	pinMode(IR_BL, INPUT);
	pinMode(IR_BR, INPUT);
	//MC-Avram
	pinMode(MC, INPUT);
	//UV
	pinMode(UV, INPUT);
	//Serial
	Serial.begin(9600);

	//Gyro Init
	GyroInit();
	GyroReset(); //reset gyro cause I said so

	delay(500);
}

void loop()
{
	switch (stage)
	{
		case ALIGN:
		{
			if (align(50))
			{
				stage = startDirection ? TURN_RIGHT : FOLLOW_WALL;
			}
			break;
		}
		case TURN_RIGHT:
		{

			break;
		}
		default:
		{
			//do nothing as default
			break;
		}
	}
}


/*
USRead - reads ultrasonic value
*/
int USRead(int trigPin, int echoPin)
{
	int duration = 0;
	int distance = 0;

	// Clears the trigPin
	digitalWrite(trigPin, LOW);
	delayMicroseconds(2);
	// Sets the trigPin on HIGH state for 10 micro seconds
	digitalWrite(trigPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(trigPin, LOW);
	// Reads the echoPin, returns the sound wave travel time in microseconds
	duration = pulseIn(echoPin, HIGH);
	// Calculating the distance
	distance = duration*0.034 / 2;

	return distance; //return the result
}

/*
MoveMotor - moving a specific motor
int motorPort - what motor to move
int power - the power to send to the motor - between 0 and 100
*/
int MoveMotor(int motorPort, int power) //true for forward
{
	power = map(power, 0, 100, 0, 255);
	analogWrite(motorPort, power); //set power to the motor
}

/*
GyroInit - initialize the gyro MPU6050 sensor
*/
void GyroInit()
{
	//start wire transmission
	Wire.begin();
	TWBR = 24;
	//initialize gyro
	mpu.initialize();
	mpu.dmpInitialize();
	//set gyro offsets - if fucked them up use calibration program in shulman's drive (Drive A)
	mpu.setXAccelOffset(1352);
	mpu.setYAccelOffset(-4082);
	mpu.setZAccelOffset(2166);
	mpu.setXGyroOffset(81);
	mpu.setYGyroOffset(93);
	mpu.setZGyroOffset(121);
	mpu.setDMPEnabled(true); //just enable
	packetSize = mpu.dmpGetFIFOPacketSize();
	fifoCount = mpu.getFIFOCount();
	Serial.println("Gyro Initialized"); //inform serial
}

/*
GyroRead - read the gyro values
*/
int GyroRead()
{
	while (fifoCount < packetSize)  //get packets
	{
		fifoCount = mpu.getFIFOCount();
	}

	if (fifoCount > packetSize * 8) //if more than 8 fucets (packets) - reset FIFO
	{
		mpu.resetFIFO();
		fifoCount = mpu.getFIFOCount();
		return 1; //error resetting
	}
	else 
	{
		while (fifoCount >= packetSize) 
		{
			mpu.getFIFOBytes(fifoBuffer, packetSize);
			fifoCount -= packetSize;

			if (fifoCount % packetSize == 0) 
			{
				mpu.dmpGetQuaternion(&q, fifoBuffer);
				mpu.dmpGetEuler(euler, &q);
				/*
				Serial.print("euler\t");
				Serial.print(euler[0] * 180 / M_PI);
				Serial.print("\t");
				Serial.print(euler[1] * 180 / M_PI);
				Serial.print("\t");
				Serial.println(euler[2] * 180 / M_PI);
				*/
			}
			else 
			{
				mpu.resetFIFO();
				Serial.println("Reset");
			}
		}
	}
}

/*
GetAngle - get current angle of the gyro
*/
float GetAngle()
{
	int error = GyroRead(); //Read Gyro
	if (error != 1)
	{
		return (euler[0] * 180 / M_PI) - resetAngle;
		lastAngle = (euler[0] * 180 / M_PI) - resetAngle;
	}
	else
	{
		return lastAngle;
	}
}

/*
GyroReset - reset the gyro by adjusting the global variable
*/
void GyroReset()
{
	resetAngle = GetAngle();
}

/*
align the robot the the wall in the start of the program
*/
bool align(int drivePower)
{
	//variable declaration
	static int startDisFront = USRead(Trig_RF, Echo_RF); //start front direction of right side (front) for range comparison
	static int startDisBack = USRead(Trig_RB, Echo_RB); //start front direction of right side (back) for range comparison
	static int disFront = 0; //front distance
	static int disBack = 0; //back distance
	startDirection = (startDisFront > 60 || startDisBack > 60); //check which side the robot

	if (startDirection) //set what sensors to use for the alignment process
	{
		//left
		disFront = USRead(Trig_LF, Echo_LF);
		disBack = USRead(Trig_LB, Echo_LB);
	}
	else
	{
		//right
		disFront = USRead(Trig_RF, Echo_RF);
		disBack = USRead(Trig_RB, Echo_RB);
	}

	if (!aligned)
	{
		if (startDirection ? disFront < disBack : !(disFront < disBack)) //depend on the start direction set the right condition
		{
			MoveMotor(MOTOR_RB, drivePower);
			MoveMotor(MOTOR_LF, drivePower);
		}
		else if (startDirection ? disFront > disBack : !(disFront > disBack)) //depend on the start direction set the right condition
		{
			MoveMotor(MOTOR_RF, drivePower);
			MoveMotor(MOTOR_LB, drivePower);
		}
		else
		{
			aligned = true; //change alignment status in globals
		}
	}
	else //stop all motors
	{
		MoveMotor(MOTOR_RF, 0);
		MoveMotor(MOTOR_RB, 0);
		MoveMotor(MOTOR_LF, 0);
		MoveMotor(MOTOR_LB, 0);

		return FINISHED;
	}
}

/*
turnRight - turn right
int angle - angle to turn
int power - motor power
*/
bool turnRight(int power, int angle)
{
	//add code here
}



//NOTES

//dis [cm] = elapsed time * sound velocity (340 m\s) / 100 / 2

// angle is euler[0] * 180 / M_PI