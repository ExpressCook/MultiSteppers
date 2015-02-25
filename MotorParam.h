#ifndef MotorParam_h
#define MotorPara_h

//serial port 
const int serialRate = 19200;
const int serialTimeout = 5;

//x y direction range
const long rangeX = 1700;
const long rangeY = 720;

//stepper in x direction
//small motor move the carrier
const int xSpeed = 400;
const int xAccel = 600;
const int xDeAccel = 600;

//stepper in y direction
//big motor move the hole gantry
const int ySpeed = 400;
const int yAccel = 500;
const int yDeAccel = 500;

//range of linear actuator
const long rangeLMin = 50;
const long rangeLMax = 885; //for cdr

#endif