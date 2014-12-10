#include "DCMotor.h"

// Constructors ////////////////////////////////////////////////////////////////
DCMotor::DCMotor()
{}

DCMotor::DCMotor(unsigned char MDIR, unsigned char MPWM, unsigned char MFB,
                 unsigned char nD2, unsigned char nSF)
{
  //Pin map
  //PWM1 and PWM2 cannot be remapped because the library assumes PWM is on timer1
  _nD2 = nD2;
  _MDIR = MDIR;
  _nSF = nSF;
  _MFB = MFB;
  _MPWM = MPWM; 
}

// Public Methods //////////////////////////////////////////////////////////////
void DCMotor::init()
{
// Define pinMode for the pins and set the frequency for timer1.

  pinMode(_MDIR,OUTPUT);
  pinMode(_MPWM,OUTPUT);
  pinMode(_MFB,INPUT);
  pinMode(_nD2,OUTPUT);
  digitalWrite(_nD2,HIGH); // default to on
  pinMode(_nSF,INPUT);

  #if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__)
  // Timer 1 configuration
  // prescaler: clockI/O / 1
  // outputs enabled
  // phase-correct PWM
  // top of 400
  //
  // PWM frequency calculation
  // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
  TCCR1A = 0b10100000;
  TCCR1B = 0b00010001;
  ICR1 = 400;
  #endif
}
// Set speed for motor 1, speed is a number betwenn -400 and 400
void DCMotor::setSpeed(int speed)
{
  _speed = speed;
  unsigned char reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 400)  // Max PWM dutycycle
    speed = 400;
  #if defined(__AVR_ATmega168__)|| defined(__AVR_ATmega328P__)
  OCR1A = speed;
  #else
  analogWrite(_MPWM,speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
  #endif
  if (reverse)
    digitalWrite(_MDIR,HIGH);
  else
    digitalWrite(_MDIR,LOW);
}

//return the motor speed
int DCMotor::getSpeed()
{
  return _speed;
}

// Return motor 1 current value in milliamps.
unsigned int DCMotor::getCurrent()
{
  // 5V / 1024 ADC counts / 525 mV per A = 9 mA per count
  return analogRead(_MFB) * 9;
}

// Return error status
unsigned char DCMotor::getFault()
{
  return !digitalRead(_nSF);
}