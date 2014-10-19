#include "ProStepper.h"

void setup()
{
  Serial.begin(9600);
  ProStepper step_X (1,2,3,5);
}

void loop()
{
  
}

