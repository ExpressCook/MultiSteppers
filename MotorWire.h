#ifndef MotorWire_h
#define MotorWire_h

#include <Arduino.h>

//stepper X
const int Dir1 = 28;
const int Step1 = 26;
const int Slp1 = 24;
const int Res1 = 22;
const int En1 = 2;

//Limit Switch X
const int switch1 = 53;

//stepper Y
const int Dir2 = 36;
const int Step2 = 34;
const int Slp2 = 32;
const int Res2 = 30;
const int En2 = 2;

//Limit Switch Y
const int switch2 = 52;

//Emergency Stop
//0-2 1-3 2-21 3-20 4-19 5-18 
const int hardStopInter = 1;
const int hardStopPin = 3;

//DC rotation
const unsigned char rotate_dir = 7;
const unsigned char rotate_pwm = 9;
const unsigned char rotate_fb = A0;
const unsigned char rotate_nd2 = 4;
const unsigned char rotate_nsf = 12;
const int encd_1 = 2; //21
const int encd_2 = 3; //20

//Linear actuator
const unsigned char linear_dir = 8;
const unsigned char linear_pwm = 10;
const unsigned char linear_fb = A1;
const unsigned char linear_nd2 = 4;
const unsigned char linear_nsf = 12;
const int linear_pos = A2;

//distance hall sensor
const int hallSensor = 8;

#endif