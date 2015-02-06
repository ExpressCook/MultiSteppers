#include "RotateMotor.h"
#include "RotateParam.h"
#include <Arduino.h>

//interruption code for the rotation motor
static volatile unsigned long _time1;
static volatile unsigned long _time2; 
static volatile bool _isFirst;
static volatile bool _isFinish;

RotateMotor::RotateMotor(DCMotor *motor, int encd1, int encd2)
{
	_motor = motor;
	_encd1 = encd1;
	_encd2 = encd2;
}

void RotateMotor::init()
{
	_motor->init();
	_motor->setSpeed(0);
	_targetSpeed = 0;
	_currentSpeed = 0;
}

void RotateMotor::rotateWithSpeed(long speed)
{	
	//motor can only rotate in one direction
	_targetSpeed = constrain(speed, 0, 580);

	if(_targetSpeed==0)
		stop();
}

long RotateMotor::getRotateSpeed()
{
	updateSpeed();
	return _currentSpeed;
}

long RotateMotor::getDesiredSpeed()
{
	return _targetSpeed;
}

bool RotateMotor::run()
{
	if(_targetSpeed!=0)
	{
		updateSpeed();

		_diff = _targetSpeed - _currentSpeed;
		_integ = _integ + _diff;
		_diffDiff = _diff - _lastDiff;
		_lastDiff = _diff;

		//speed range(0->580) pwm range(0->400)
		// 0.68 = 400/580
		_control = (0.68 * _targetSpeed) + //feed forward control
			   KP*_diff + 			  //feedback control	
			   KI*_integ/5000 + 
			   KD*_diffDiff/50; 

		_control = constrain(_control, 0, 400);

		_motor->setSpeed(_control);
	}
}

void RotateMotor::stop()
{
	_motor->setSpeed(0);
	_targetSpeed = 0;
	_currentSpeed = 0;
}

void RotateMotor::updateSpeed()
{
	//Time all in milliseconds
	static unsigned long maxTime = 2;
	unsigned long startTime, currentTime;

	//record the time
	startInterrupt();	
	startTime = currentTime = millis();
	while(!_isFinish && (currentTime - startTime)<maxTime)
	{
		currentTime = millis();
	}	
	stopInterrupt();

	//caculate the time
	if(_isFinish)
		// 464.64 count / rev => 129120 = 1000*1000*60/464.64
		_currentSpeed = 129120/(_time2-_time1);
	else
		_currentSpeed = 0;
}

void countEncd()
{
	if(_isFirst)
	{
		_time1 = micros();
		_isFirst = false;
	} 
	else
	{
		_time2 = micros();
		_isFinish = true;
	}
}

void RotateMotor::startInterrupt()
{	
	_isFirst = true; _isFinish=false;
	attachInterrupt(_encd1, countEncd, CHANGE);
	attachInterrupt(_encd2, countEncd, CHANGE);
}

void RotateMotor::stopInterrupt()
{
	detachInterrupt(_encd1);
	detachInterrupt(_encd2);
}

