#ifndef ProStepper_h
#define Prostepper_h

#include <Arduino.h>

class ProStepper
{
public:
	//constructors
	ProStepper(int stepSize, int pinDir, int pinStep, int pinEn);

	//set motion profile
	//only valid after the next call of move
	void setMaxSpeed(int maxSpeed);
	void setAcceleration(int acceleration);
	void setDeAcceleration(int deacceleration);

	//set target position
	void moveTo(long absolute);
	void move(long relative);
	bool run();
	void stop(long minDeaccelSteps);
	void hardStop();

	//get current states
	float getCurrentSpeed();
	long getCurrentPos();
	long getTargetPos();

private:
	typedef enum
    {
		DIRECTION_CCW = 0,  // Counter Clockwise
        DIRECTION_CW  = 1   //Clockwise
    } Direction;

	void computeNewSpeed();
	void computeNewInterval();
	void computeNewLimit();
	void step();
	void setDirection(int direction);

	int _pinDir;
	int _pinStep;
	int _pinEn;
	int _stepSize; //degree
	unsigned int _pulseWidth; //micro

	long _position;
	long _targetPosition;
	int _direction; //0 is counterclock 1 is clockwise

	int _maxSpeed;  //degree per second
	int _acceleration; //degree per second 2
	int _deacceleration; // degree per second 2

	long _totalSteps;     
	long _accelStepsToMaxSpeed;
	long _accelStepsLimit;
	long _accelStop;
	long _deaccelStart;
	long _deaccelValue;

	long _stepCount; //used in a single motion
	long _accelCount;

	unsigned long _lastStepTime; //micro

	unsigned long _stepInterval;  //micro //zero means stop
	unsigned long _stepIntervalRemain; 
	unsigned long _initStepInterval; //micro
	unsigned long _minStepInterval;  //micro
	
};

#endif