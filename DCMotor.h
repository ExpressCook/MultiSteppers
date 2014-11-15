#ifndef DCMotor_h
#define DCMotor_h

#include <Arduino.h>

class DCMotor
{
  public:  
    // CONSTRUCTORS
    DCMotor();
    DCMotor(unsigned char MDIR, unsigned char MPWM, unsigned char MFB,
            unsigned char nD2, unsigned char nSF); 
    
    // PUBLIC METHODS
    void init(); // Initialize TIMER 1, set the PWM to 20kHZ. 
    void setSpeed(int speed); // Set speed for M1.
    unsigned int getCurrent(); // Get current reading for M1. 
    unsigned char getFault(); // Get fault reading.
    
  private:
    unsigned char _MDIR;
    unsigned char _MPWM;
    unsigned char _MFB;
    unsigned char _nD2;
    unsigned char _nSF;
};

#endif