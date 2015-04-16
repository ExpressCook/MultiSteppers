#include "LinearActuator.h"
#include "LinearParam.h"
#include "MotorParam.h"
#include <Arduino.h>

#define DEBUG false

LinearActuator::LinearActuator(DCMotor *motor, int pinPos)
{
	_motor = motor;
	_pinPos = pinPos;
}

void LinearActuator::init()
{
	_motor->init();
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
	_currentLimit = 50;
	_currentCount = 3;
}

void LinearActuator::setSpeed(int speed)
{
	_motor->setSpeed(speed);
}

void LinearActuator::moveTo(long absolute)
{
	// protective for linear actuator current
	_isCurrentLimitOn = true;
	_currentLimit = 80;
	_currentCount = 5;

	if(absolute != _targetPosition ||
		absolute <= 1023 ||
		absolute >= 0)
		_targetPosition = absolute;
}

void LinearActuator::move(long relative)
{
	moveTo(_position + relative);
}

void LinearActuator::moveTillHit(int strength)
{
	_isCurrentLimitOn = true;
	_currentLimit = strength;
	_targetPosition = rangeLMax;

	if(strength<10)
		_currentCount = 1;
	else if(strength<20)
		_currentCount = 2;
	else if(strength<60)
		_currentCount = 3;
	else if(strength<100)
		_currentCount = 5;
	else
		_currentCount = 8;
}

bool LinearActuator::run()
{	
	//check the current
	static unsigned long lastCurrentTime=0, currentInterval=10;
	unsigned long now;

	static int curCount = 0;

	if(_isCurrentLimitOn)
	{
		now = millis();
		if(now - lastCurrentTime>=currentInterval)
		{
			lastCurrentTime=now;

			int current = getCurrent();
			current = current>180? 0:current;

			if(current>_currentLimit)
			{
				curCount++;
				if(curCount>=_currentCount)
				{
					stop();
					curCount = 0;
				}
			}

#if DEBUG
	Serial.print("llCurrent:");
	Serial.println(llCurrent);
	Serial.print("lCurrent:");
	Serial.println(lCurrent);
	Serial.print("Current:");
	Serial.println(current);
#endif
		}
	}
	else
	{
		curCount = 0;
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
	//Serial.println("run acctuator:");
	//Serial.print("speed:");
	//Serial.println(_motor->getSpeed());
	//Serial.print("pos:");
	//Serial.println(_position);
	//Serial.print("target:");
	//Serial.println(_targetPosition);
#endif

	//update position again, may not necessary?
	updatePos();
}

void LinearActuator::stop()
{
	_motor->setSpeed(0);
	_targetPosition = _position;
}

unsigned int LinearActuator::getCurrent()
{
	return _motor->getCurrent();
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

