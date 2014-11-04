#ifndef ProStepper_h
#define Prostepper_h

#include <Arduino.h>

class ProStepper
{
public:
	//constructors
	ProStepper(int stepSize, int pinDir, int pinStep, int pinSlp, int pinRes, int pinEn);

	//set motion profile
	//only valid after the next call of move
	void setMaxSpeed(long maxSpeed);
	void setAcceleration(long acceleration);
	void setDeAcceleration(long deacceleration);
	void setPosition(long position);

	//nonblock movement
	void moveTo(long absolute);
	void move(long relative);
	bool run();
	void stop(long minDeaccelSteps);

	//block movement
	//better not to use, will affect other runing motor
	void blockMoveTo(long absolute);
	void blockMove(long relative);

	//change the motor state
	void hardStop();
	void recover();
	void sleep();
	void wake();

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

	//pin and parameter related to stepper motor
	int _pinDir;
	int _pinStep;
	int _pinSlp;
	int _pinRes;
	int _pinEn;
	int _stepSize; //degree
	unsigned int _pulseWidth; //micro

	//current state
	bool _isDisabled;
	bool _isSleep;

	//position information
	long _position;
	long _targetPosition;
	int _direction; //0 is counterclock 1 is clockwise

	//motion profile
	long _maxSpeed;  //degree per second
	long _acceleration; //degree per second 2
	long _deacceleration; // degree per second 2

	//crtical steps marks change of different motion
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

	//used to give up current motion and execute new motion
	bool _hasCommand;
	long _storedPosition;
	
};

#endif