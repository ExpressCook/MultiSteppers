#include "ProStepper.h"
#define DEBUG true

//x y direction range
long rangeX = 5000;
long rangeY = 3000;

//stepper in x direction
//small motor move the carrier
int Dir1 = 28;
int Step1 = 26;
int Slp1 = 24;
int Res1 = 22;
int En1 = 1;

int xSpeed = 600;
int xAccel = 700;
int xDeAccel = 700;

//limit switch in x direction
int switch1 = 52;

//stepper in y direction
//big motor move the hole gantry
int Dir2 = 36;
int Step2 = 34;
int Slp2 = 32;
int Res2 = 30;
int En2 = 1;

int ySpeed = 600;
int yAccel = 700;
int yDeAccel = 700;

//limit switch in y direction
int switch2 = 53;

//emergency stop, interrupt
//0-2 1-3 2-21 3-20 4-19 5-18 
int hardStopInter = 5;
int hardStopPin = 18;

ProStepper step1 (1,Dir1,Step1,Slp1,Res1,En1);
ProStepper step2 (1,Dir2,Step2,Slp2,Res2,En2);

void setup()
{
  //set up serial port	
  Serial.begin(115200); //9600
  Serial.setTimeout(2);

  //set up the switch
  pinMode(switch1, INPUT_PULLUP);
  pinMode(switch2, INPUT_PULLUP);
  pinMode(hardStopPin, INPUT_PULLUP);
  attachInterrupt(hardStopInter, emergencyStop, FALLING);

  //set up the motion profile of motor X
  step1.setMaxSpeed(xSpeed);
  step1.setAcceleration(xAccel);
  step1.setDeAcceleration(xDeAccel);

  //set up the motion profile of motor y
  step2.setMaxSpeed(ySpeed);
  step2.setAcceleration(yAccel);
  step2.setDeAcceleration(yDeAccel);
}

//information extracted from serial comunication
char chosedStep;
char mode;
long position;


void loop()
{
	if(Serial.available()>1)
	{
		// x y 
		// g(general for both x and y)
		do
		{
			chosedStep = Serial.read();
		}
		while(chosedStep!='x' && chosedStep!='y' && chosedStep!='g');
		
		if(chosedStep=='x' || chosedStep=='y')
		{	
			// r(relative) a(absolute)
			do
			{
				mode = Serial.read();
			}
			while(mode!='a' && mode!='r');

			//1000
			position = Serial.parseInt();
		}
		else if(chosedStep=='g')
		{
			// c(callibration)
			do
			{
				mode = Serial.read();
			}
			while(mode!='c');
		}
		
		//$ (end mark)
		Serial.read();

		executeCommand();
	}

	step1.run();
	step2.run();

	reportState(); 
}

void executeCommand()
{
#if DEBUG

	Serial.print(chosedStep);
	Serial.print(mode);
	Serial.println(position);

#endif

	long currentPos = 0;
	if(chosedStep == 'x')
	{
		if(mode == 'r')
		{
			currentPos = step1.getCurrentPos();
			position = constrain(-currentPos,0,rangeX-currentPos);
			step1.move(position);
		}
		else if(mode == 'a')
		{
			position = constrain(position,0,rangeX);
			step1.moveTo(position);
		}
	}
	else if(chosedStep == 'y')
	{
		if(mode == 'r')
		{
			currentPos = step2.getCurrentPos();
			position = constrain(-currentPos,0,rangeY-currentPos);
			step2.move(position);
		}
		else if(mode == 'a')
		{
			position = constrain(position,0,rangeY);
			step2.moveTo(position);
		}
	}
	else if(chosedStep == 'g')
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
	step1.setMaxSpeed(300);
	step2.setMaxSpeed(300);

	step1.move(-rangeX-1000);
	step2.move(-rangeY-1000);

	int s1, s2 = HIGH;
	bool c1, c2 = false;
	while( (!c1) || (!c2) )
	{
		step1.run();
		step2.run();

		s1 = digitalRead(switch1);
		s2 = digitalRead(switch2);

		if(!c1 && (s1 == LOW))
		{
			c1 = true;
			step1.hardStop();
			step1.setPosition(0);
		}

		if(!c2 && (s2 == LOW))
		{
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
#if DEBUG
	Serial.println("emergency stop!!");
#endif 

	step1.hardStop();
	step2.hardStop();
}
