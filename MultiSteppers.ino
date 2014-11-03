#include "ProStepper.h"
#define DEBUG true

//stepper in x direction
//small motor move the carrier
int Dir1 = 28;
int Step1 = 26;
int Slp1 = 24;
int Res1 = 22;
int En1 = 1;

//limit switch in x direction
int switch1 = 52;

//stepper in y direction
//big motor move the hole gantry
int Dir2 = 36;
int Step2 = 34;
int Slp2 = 32;
int Res2 = 30;
int En2 = 1;

//limit switch in y direction
int switch2 = 53;

ProStepper step1 (1,Dir1,Step1,Slp1,Res1,En1);
ProStepper step2 (1,Dir2,Step2,Slp2,Res2,En2);

void setup()
{
  //Serial.begin(115200);
  Serial.begin(9600);
  Serial.setTimeout(2);

  step1.setMaxSpeed(600);
  step1.setAcceleration(700);
  step1.setDeAcceleration(700);

  step2.setMaxSpeed(600);
  step2.setAcceleration(700);
  step2.setDeAcceleration(700);
}

char chosedStep;
char mode;
long position;

void loop()
{
	if(Serial.available()>3)
	{
		// x y
		do
		{
			chosedStep = Serial.read();
		}
		while(chosedStep!='x' && chosedStep!='y');
			
		// r a
		do
		{
			mode = Serial.read();
		}
		while(mode!='a' && mode!='r');

		//1000
		position = Serial.parseInt();
		//$
		Serial.read();

		executeCommand();
	}

	step1.run();
	step2.run();
  
}

void executeCommand()
{
#if DEBUG

	Serial.print(chosedStep);
	Serial.print(mode);
	Serial.println(position);

#endif

	if(chosedStep == 'x')
	{
		if(mode == 'r')
			step1.move(position);
		else if(mode == 'a')
			step1.moveTo(position);
	}
	else if(chosedStep == 'y')
	{
		if(mode == 'r')
			step2.move(position);
		else if(mode == 'a')
			step2.moveTo(position);
	}
}

void callibrateMotor()
{

}
