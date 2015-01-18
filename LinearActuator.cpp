#include "LinearActuator.h"
#include "LinearParam.h"
#include "MotorParam.h"

#define DEBUG false

LinearActuator::LinearActuator(DCMotor motor, int pinPos)
{
	_motor = motor;
	_pinPos = pinPos;
}

void LinearActuator::init()
{
	_motor.init();
	pinMode(_pinPos, INPUT);

	//initialze poistion info
	updatePos();
	_targetPosition = _position;
	setSpeed(0);

	//initialize control state
	_diff = 0;
	_lastDiff = 0;
	_diffDiff = 0;
	_integ = 0;
	_control = 0;

	//other
	_isCurrentLimitOn = false;
}

void LinearActuator::setSpeed(int speed)
{
	_motor.setSpeed(speed);
}

void LinearActuator::moveTo(long absolute)
{
	_isCurrentLimitOn = false;

	if(absolute != _targetPosition ||
		absolute <= 1023 ||
		absolute >= 0)
		_targetPosition = absolute;
}

void LinearActuator::move(long relative)
{
	moveTo(_position + relative);
}

void LinearActuator::moveTillHit()
{
	_isCurrentLimitOn = true;
	_targetPosition = rangeLMax;
}

bool LinearActuator::run()
{	
	//check the current
	if(_isCurrentLimitOn)
	{
		if(getCurrent()>MAX_CURRENT)
			stop();
	}

	//calculating state 
	updatePos();
	_diff = _targetPosition - _position;
	_integ = _integ + _diff;
	_diffDiff = _diff - _lastDiff;
	_lastDiff = _diff;

	//run PID control
	if(_diff == 0)
	{
		setSpeed(0);
	}
	else
	{
		_control = KP*_diff + KI*_integ/5000 + KD*_diffDiff/50;
		_control = constrain(_control, -400, 400);
		setSpeed(_control);
	}

#if DEBUG
	Serial.println("run acctuator:");
	Serial.print("speed:");
	Serial.println(_motor.getSpeed());
	Serial.print("pos:");
	Serial.println(_position);
	Serial.print("target:");
	Serial.println(_targetPosition);
#endif

	//update position again, may not necessary?
	updatePos();
}

void LinearActuator::stop()
{
	_motor.setSpeed(0);
	_targetPosition = _position;
}

unsigned int LinearActuator::getCurrent()
{
	return _motor.getCurrent();
}

long LinearActuator::getCurrentPos()
{
	updatePos();
	return _position;
}

long LinearActuator::getTargetPos()
{
	return _targetPosition;
}

void LinearActuator::updatePos()
{
	_position = analogRead(_pinPos);
	//0 - 1023
}

