#include "ProStepper.h"
#define DEBUG true

ProStepper::ProStepper(int stepSize, int pinDir, int pinStep, int pinEn)
{
	_stepSize = stepSize;
	_pinDir = pinDir;
	_pinStep = pinStep;
	_pinEn = pinEn;
	_pulseWidth = 1;

	setMaxSpeed(1);
	setAcceleration(1);
	setDeAcceleration(1);

	_position=0;
	_direction=DIRECTION_CW;

	_stepCount=0;
	_accelCount=0;
	_totalSteps=0;
	_lastStepTime=0;
	_stepInterval =0;
	_stepIntervalRemain=0;

	//set up pinmode
	pinMode(_pinDir, OUTPUT);
	pinMode(_pinStep, OUTPUT);
	pinMode(_pinEn, OUTPUT);
}

void ProStepper::setMaxSpeed(int maxSpeed)
{
	_maxSpeed = maxSpeed;
	_minStepInterval = _stepSize * 1000000 / _maxSpeed; //equation 1
	//computeNewLimit();
}

void ProStepper::setAcceleration(int acceleration)
{
	_acceleration = acceleration;
	_initStepInterval = 676000 * sqrt(2*_stepSize/_acceleration); //equation 2
	//computeNewLimit();
}

void ProStepper::setDeAcceleration(int deacceleration)
{
	_deacceleration = deacceleration;
	//computeNewLimit();
}

void ProStepper::moveTo(long absolute)
{
	if(absolute != _targetPosition)
		move(absolute - _position);
}

void ProStepper::move(long relative)
{
	//no need to move
	if(relative == 0)
		return;

	_targetPosition = _position + relative;

	//the stepper is still moving
	if(_stepInterval != 0)
	{
		long minDeaccelSteps = (_stepSize*1000000)
						/(_stepInterval*_stepInterval*_deacceleration/500000);
		long minDeaccelDistance = minDeaccelSteps * _stepSize;

		// the stepper unable to stop in relative distance
		// first stop then move in another direction
		if(relative < minDeaccelDistance)
		{
			stop(minDeaccelSteps);
			moveTo(_targetPosition);
		}
		// the stepper already in deaccelraion phase
		// just wait a little to let it finish
		else if(_stepCount>_deaccelStart)
		{	
			while(run())
			{}
			moveTo(_targetPosition);
		}
		// the stepper can continue work
		// just need to modify some limit value
		else
		{
			//modify the current settings
			_totalSteps = _stepCount + (abs(relative)/_stepSize);
			computeNewLimit();
		}
	}
	//the stepper motor has finished moving
	else
	{
		if(relative<0)
			setDirection(DIRECTION_CCW);
		else
			setDirection(DIRECTION_CW);
	
		_totalSteps = (abs(relative)/_stepSize)-1; //_totalSteps start with 0
		computeNewLimit();
	
		_stepInterval = _initStepInterval;
		_stepCount = 0;
		_accelCount = 0;
		_lastStepTime = micros();
	}

}

bool ProStepper::run()
{
	if(_stepCount<=_totalSteps && _stepInterval!=0)
	{	
		//make a step
		unsigned long currentTime = micros();
		if(currentTime - _lastStepTime>=_stepInterval)
		{
			step();
			_lastStepTime = currentTime;

			//update position
			if(_direction==DIRECTION_CW)
				_position += _stepSize;
			else
				_position -= _stepSize;	

			//caculate the new speed
			computeNewSpeed();
		}
		return false;
	}
	else
	{
#if DEBUG

		Serial.println("finish move!");
		Serial.print("targetPos:");
		Serial.println(_targetPosition);
		Serial.print("currentPos:");
		Serial.println(_position);

#endif
		return true;
	}
}

//block until the motor stop
// do not ensure correct position
void ProStepper::stop(long minDeaccelSteps)
{
	if(minDeaccelSteps == 0)
		minDeaccelSteps = (_stepSize*1000000)
						/(_stepInterval*_stepInterval*_deacceleration/500000);

	_totalSteps = _stepCount + minDeaccelSteps+1;
	computeNewLimit();

	while(run())
	{}
}

void ProStepper::hardStop()
{
	digitalWrite(_pinEn, LOW);
}

void ProStepper::computeNewSpeed()
{
	//step count indicate the number of finished step
	if(_stepCount < _accelStop)
	{
		//accelerate
		_stepCount++;
		_accelCount++;
		computeNewInterval();
	}
	else if(_stepCount == _accelStop && _stepCount!=_deaccelStart)
	{
		//run init
		_stepCount++;
		_stepInterval = _minStepInterval;

	}
	else if(_stepCount < _deaccelStart)
	{
		//run
		_stepCount++;
	}
	else if(_stepCount == _deaccelStart)
	{
		//deaccelerate init
		_stepCount++;
		_accelCount = _deaccelValue;
		computeNewInterval();
	}
	else if(_stepCount < _totalSteps)
	{
		//deaccelerate
		_stepCount++;
		_accelCount++;
		computeNewInterval();
	}
	else if(_stepCount == _totalSteps)
	{
		_stepCount = 0;
		_accelCount = 0;
		_totalSteps = 0;

		_stepInterval = 0;
		_stepIntervalRemain = 0;
	}

#if DEBUG

	Serial.print("step count:");
	Serial.println(_stepCount);
	Serial.print("accel count:");
	Serial.println(_accelCount);
	Serial.print("interval:");
	Serial.println(_stepInterval);

#endif

}

void ProStepper::computeNewInterval()
{
	//equation 3
	_stepInterval = _stepInterval - 
	               (2*_stepInterval + _stepIntervalRemain/(4*_accelCount+1)); 
 	_stepIntervalRemain = (2*_stepInterval + _stepIntervalRemain)%(4*_accelCount+1);

}

void ProStepper::computeNewLimit()
{
	_accelStepsToMaxSpeed = (_maxSpeed * _maxSpeed)/(2*_stepSize*_acceleration) ;  //equation 4
	_accelStepsLimit = (_totalSteps*_deacceleration)/(_acceleration+_deacceleration);  //equation 5
	
	if(_accelStepsToMaxSpeed > _accelStepsLimit)
	{
		_deaccelValue = -(_totalSteps - _accelStepsLimit);
		_accelStop = _accelStepsLimit;
	}
	else
	{
		_deaccelValue = -(_accelStepsToMaxSpeed * _acceleration / _deacceleration);	
		_accelStop = _accelStepsToMaxSpeed;
	}

	_deaccelStart = _totalSteps + _deaccelValue;
}

void ProStepper::step()
{
	digitalWrite(_pinStep,LOW);
	digitalWrite(_pinStep,HIGH);
	delayMicroseconds(_pulseWidth);
	digitalWrite(_pinStep,LOW);
}

void ProStepper::setDirection(int direction)
{
	if(direction != _direction)
	{
		_direction = direction;
		if(_direction == DIRECTION_CW)
			digitalWrite(_pinDir, HIGH);
		else
			digitalWrite(_pinDir, LOW);	
	}
}

float ProStepper::getCurrentSpeed()
{
	if(_stepInterval == 0)
		return 0;
	else
		return (_stepSize * 1000000)/_stepInterval;
}

long ProStepper::getCurrentPos()
{
	return _position;
}

long ProStepper::getTargetPos()
{
	return _targetPosition;
}