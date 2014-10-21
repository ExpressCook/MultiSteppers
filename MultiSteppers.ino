#include "ProStepper.h"

int Step1 = 13;
int Dir1 = 12;
int En1 = 1;
int Step2 = 11;
int Dir2 = 10;
int En2 = 1;

ProStepper step1 (1,Dir1,Step1,En1);
ProStepper step2 (1,Dir2,Step2,En2);

void setup()
{
  Serial.begin(9600);

  step1.setMaxSpeed(600);
  step1.setAcceleration(700);
  step1.setDeAcceleration(700);

}

void loop()
{
	if(Serial.available())
	{
		int position = Serial.parseInt();
		step1.moveTo(position);
	}

	step1.run();
  
}
