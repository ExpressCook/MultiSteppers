#include "ProStepper.h"
#include "DCMotor.h"
#include "LinearActuator.h"
#define DEBUG false

//x y direction range
long rangeX = 1800;
long rangeY = 800;

//stepper in x direction
//small motor move the carrier
int Dir1 = 28;
int Step1 = 26;
int Slp1 = 24;
int Res1 = 22;
int En1 = 1;

int xSpeed = 400;
int xAccel = 600;
int xDeAccel = 600;

//limit switch in x direction
int switch1 = 53;

//stepper in y direction
//big motor move the hole gantry
int Dir2 = 36;
int Step2 = 34;
int Slp2 = 32;
int Res2 = 30;
int En2 = 1;

int ySpeed = 400;
int yAccel = 500;
int yDeAccel = 500;

//limit switch in y direction
int switch2 = 52;

//emergency stop, interrupt
//0-2 1-3 2-21 3-20 4-19 5-18 
int hardStopInter = 2;
int hardStopPin = 21;

//DC rotate
unsigned char rotate_dir = 7;
unsigned char rotate_pwm = 9;
unsigned char rotate_fb = A0;
unsigned char rotate_nd2 = 4;
unsigned char rotate_nsf = 12;

//DC linear
unsigned char linear_dir = 8;
unsigned char linear_pwm = 10;
unsigned char linear_fb = A1;
unsigned char linear_nd2 = 4;
unsigned char linear_nsf = 12;
int linear_pos = A2;

//range of linear actuator
long rangeLMin = 100;
long rangeLMax = 900;

//serial port 


ProStepper step1 (1,Dir1,Step1,Slp1,Res1,En1);
ProStepper step2 (1,Dir2,Step2,Slp2,Res2,En2);
DCMotor dcRotate (rotate_dir,rotate_pwm, rotate_fb, rotate_nd2, rotate_nsf);
DCMotor dcLinear(linear_dir, linear_pwm, linear_fb, linear_nd2, linear_nsf);
LinearActuator linearAct (dcLinear, linear_pos);

void setup()
{
  //set up serial port	
  Serial1.begin(9600); //9600
  Serial1.setTimeout(5);

  //set up the switch
  pinMode(switch1, INPUT_PULLUP);
  pinMode(switch2, INPUT_PULLUP);
  pinMode(hardStopPin, INPUT_PULLUP);
  attachInterrupt(hardStopInter, emergencyStop, LOW);

  //set up the motion profile of motor X
  step1.setMaxSpeed(xSpeed);
  step1.setAcceleration(xAccel);
  step1.setDeAcceleration(xDeAccel);

  //set up the motion profile of motor y
  step2.setMaxSpeed(ySpeed);
  step2.setAcceleration(yAccel);
  step2.setDeAcceleration(yDeAccel);

  //set up the DC motor
  dcRotate.init();
  linearAct.init();
}

//information extracted from serial comunication
char chosedMotor;
char mode;
long value;

void loop()
{
	//parse the command
	if(Serial1.available()>1)
	{
		// x y 
		// g(general for both x and y)
		// r(rotation) l(linear actuator)
		do
		{
			chosedMotor = Serial1.read();
		}
		while(chosedMotor!='x' && chosedMotor!='y' && chosedMotor!='g'
			  && chosedMotor!='r' && chosedMotor!='l');
		
		if(chosedMotor=='x' || chosedMotor=='y')
		{	
			// r(relative) a(absolute)
			do
			{
				mode = Serial1.read();
			}
			while(mode!='a' && mode!='r');

			//1000
			value = Serial1.parseInt();
		}
		else if(chosedMotor=='r')
		{
			// speed control
			// open loop(o) close loop(c)
			do
			{
				mode = Serial1.read();
			}
			while(mode!='o' && mode!='c');

			//1000
			value = Serial1.parseInt();
		}
		else if(chosedMotor=='l')
		{
			// p(relative) t(tolerance)
			do
			{
				mode = Serial1.read();
			}
			while(mode!='p' && mode!='t');

			//1000
			value = Serial1.parseInt();
		}
		else if(chosedMotor=='g')
		{
			// c(callibration)
			do
			{
				mode = Serial1.read();
			}
			while(mode!='c');
		}
		
		//$ (end mark)
		Serial1.read();
		executeCommand();
	}

	//runing the motors
	step1.run();
	step2.run();
	linearAct.run();

	//report the motor status
	reportState(); 
}

void executeCommand()
{
	Serial1.print(chosedMotor);
	Serial1.print(mode);
	Serial1.println(value);

	long currentPos = 0;
	if(chosedMotor == 'x')
	{
		if(mode == 'r')
		{
			currentPos = step1.getCurrentPos();
			value = constrain(-currentPos,0,rangeX-currentPos);
			step1.move(value);
		}
		else if(mode == 'a')
		{
			value = constrain(value,0,rangeX);
			step1.moveTo(value);
		}
	}
	else if(chosedMotor == 'y')
	{
		if(mode == 'r')
		{
			currentPos = step2.getCurrentPos();
			value = constrain(-currentPos,0,rangeY-currentPos);
			step2.move(value);
		}
		else if(mode == 'a')
		{
			value = constrain(value,0,rangeY);
			step2.moveTo(value);
		}
	}
	else if(chosedMotor == 'r')
	{
		if(mode == 'o')
			dcRotate.setSpeed(value);
	}
	else if(chosedMotor == 'l')
	{
		if(mode == 'p')
		{
			value = constrain(value, rangeLMin, rangeLMax);
			linearAct.moveTo(value);
		}
	}
	else if(chosedMotor == 'g')
	{
		if(mode == 'c')
			callibrateMotors();
	}
}

unsigned long lastReportTime = 0;
unsigned long ReportInterval = 500;
void reportState()
{
	unsigned long now = millis();
	if(now - lastReportTime >= ReportInterval)
	{
		//make a report
		// xp100$yp1000$
		String endMark = "$";
		String report = "xp";
		report += step1.getCurrentPos();
		report += endMark;

		report += "yp";
		report += step2.getCurrentPos();
		report += endMark; 

		lastReportTime = now;
	}
}

void callibrateMotors()
{
	//callibrate dc motors
	dcRotate.setSpeed(0);
	linearAct.moveTo(rangeLMin);

	//callibrate stepper motors
	step1.setMaxSpeed(150);
	step2.setMaxSpeed(150);

	step1.move(-rangeX-1000);
	step2.move(-rangeY-1000);

	int s1, s2 = HIGH;
	bool c1=false;
	bool c2=false;
	while( (!c1) || (!c2) )
	{
#if DEBUG
		Serial1.print("calibrating:");
		Serial1.print(c1);
		Serial1.print(c2);
		Serial1.print(s1);
		Serial1.println(s2);
#endif

		step1.run();
		step2.run();
		linearAct.run();

		s1 = digitalRead(switch1);
		s2 = digitalRead(switch2);

		if(!c1 && (s1 == LOW))
		{
#if DEBUG
		Serial1.print("x switch hit");
#endif
			c1 = true;
			step1.hardStop();
			step1.setPosition(0);
		}

		if(!c2 && (s2 == LOW))
		{
#if DEBUG
		Serial1.print("y switch hit");
#endif
			c2 = true;
			step2.hardStop();
			step2.setPosition(0);
		}
	}

	step1.recover();
	step2.recover();
	step1.setMaxSpeed(xSpeed);
	step2.setMaxSpeed(ySpeed);
}

void emergencyStop()
{
	step1.hardStop();
	step2.hardStop();
	dcRotate.setSpeed(0);
	linearAct.stop();
}
