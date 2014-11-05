#include "ProStepper.h"
#define DEBUG false

ProStepper::ProStepper(int stepSize, int pinDir, int pinStep, int pinSlp, int pinRes, int pinEn)
{
	_stepSize = stepSize;
	_pinDir = pinDir;
	_pinStep = pinStep;
	_pinSlp = pinSlp;
	_pinRes = pinRes;
	_pinEn = pinEn;
	_pulseWidth = 1;

	//setMaxSpeed(1);
	//setAcceleration(1);
	//setDeAcceleration(1);

	_isDisabled = false;
	_isSleep = false;

	_position=0;
	_direction=DIRECTION_CW;
	
	_stepCount=0;
	_accelCount=0;
	_totalSteps=0;
	_lastStepTime=0;
	_stepInterval =0;
	_stepIntervalRemain=0;

	_hasCommand = false;
	_storedPosition = 0;

	//set up pinmode
	pinMode(_pinDir, OUTPUT);
	pinMode(_pinStep, OUTPUT);
	pinMode(_pinSlp, OUTPUT);
	pinMode(_pinRes, OUTPUT);
	pinMode(_pinEn, OUTPUT);

	//init pin output
	digitalWrite(_pinDir, HIGH);
	digitalWrite(_pinSlp, HIGH);
	digitalWrite(_pinRes, HIGH);
}

void ProStepper::setMaxSpeed(long maxSpeed)
{
	_maxSpeed = maxSpeed;
	_minStepInterval = _stepSize * 1000000 / _maxSpeed; //equation 1

#if DEBUG
	Serial.println("------------SetMaxSpeed------------");
	Serial.print("minStepInterval:");
	Serial.println(_minStepInterval);
#endif 
}

void ProStepper::setAcceleration(long acceleration)
{
	_acceleration = acceleration;
	_initStepInterval = 6760 * sqrt(20000*_stepSize/_acceleration); //equation 2

#if DEBUG
	Serial.println("------------SetAccel------------");
	Serial.print("initStepInterval");
	Serial.println(_initStepInterval);
#endif 
}

void ProStepper::setDeAcceleration(long deacceleration)
{
	_deacceleration = deacceleration;
	//computeNewLimit();
}

void ProStepper::setPosition(long position)
{
	_position = position;
}

void ProStepper::moveTo(long absolute)
{
	if(absolute != _position)
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
		//compute the minDeaccelSteps
		long minDeaccelSteps = 50;
		if(_stepCount<_accelStop)
			minDeaccelSteps = _stepCount;
		else if(_stepCount<_deaccelStart)
			minDeaccelSteps = - _deaccelValue;

		long minDeaccelDistance = minDeaccelSteps * _stepSize;

		// the stepper unable to stop in relative distance
		// first stop then move in another direction
		if(_direction == DIRECTION_CCW)
			relative = -relative;

		if(relative < minDeaccelDistance)
		{
#if DEBUG
			Serial.println("~~~~~~first try to stop then move opposite~~~~~");
			Serial.print("minDeaccelSteps is:");
			Serial.println(minDeaccelSteps);
#endif

			stop(minDeaccelSteps);

			_hasCommand = true;
			_storedPosition = _targetPosition;
		}
		// the stepper already in deaccelraion phase
		// just wait a little to let it finish
		else if(_stepCount>_deaccelStart)
		{	
#if DEBUG
			Serial.println("~~~~~~wait till the deaccel end~~~~~");
#endif
			_hasCommand = true;
			_storedPosition = _targetPosition;
		}
		// the stepper can continue work
		// just need to modify some limit value
		else
		{
#if DEBUG
			Serial.println("~~~~~~just need to modify the settings~~~~~");
#endif

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

#if DEBUG
	Serial.println("----------Move To----------");
	Serial.print("initStepInterval:");
	Serial.println(_stepInterval);
	Serial.println("totalSteps:");
	Serial.println(_totalSteps);
	Serial.print("initStepCount:");
	Serial.println(_stepCount);
#endif 

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

		Serial.println("---------finish move!----------");
		Serial.print("targetPos:");
		Serial.println(_targetPosition);
		Serial.print("currentPos:");
		Serial.println(_position);
		Serial.print("finalStepInterval:");
		Serial.println(_stepInterval);

#endif
		return true;
	}
}

// do not ensure correct position
// when minDeaccelSteps = 0 it will automatically caculate the value
void ProStepper::stop(long minDeaccelSteps)
{
	if(minDeaccelSteps == 0)
	{
		if(_stepCount<_accelStop)
			minDeaccelSteps = _stepCount;
		else if(_stepCount<_deaccelStart)
			minDeaccelSteps = - _deaccelValue;
		else
			//already deaccelerating just need to wait
			return;
	}
		
	_totalSteps = _stepCount + minDeaccelSteps;
	_deaccelStart = _stepCount;
	_accelStop = _deaccelStart>_accelStop ? _accelStop : _deaccelStart; 
	_deaccelValue = -minDeaccelSteps;

}

void ProStepper::blockMoveTo(long absolute)
{
	moveTo(absolute);
	while(!run()){};
}

void ProStepper::blockMove(long relative)
{
	move(relative);
	while(!run()){};
}

void ProStepper::hardStop()
{
	digitalWrite(_pinEn, LOW);
	digitalWrite(_pinRes, LOW);

	//reset values
	_stepCount = 0;
	_accelCount = 0;
	_totalSteps = 0;
	_stepInterval = 0;
	_stepIntervalRemain = 0;
	_hasCommand = false;
	_storedPosition = 0;
	_targetPosition = _position;

	_isDisabled = true;
}

void ProStepper::recover()
{
	if(_isDisabled)
	{
		digitalWrite(_pinRes, HIGH);
		digitalWrite(_pinEn, HIGH);
		_isDisabled = false;	
	}
}

void ProStepper::sleep()
{
	if(!_isDisabled)
		hardStop();

	digitalWrite(_pinSlp, LOW);
	_isSleep = true;
}

void ProStepper::wake()
{
	if(_isSleep)
	{
		recover();
		digitalWrite(_pinSlp, HIGH);
		_isSleep = false;
	}
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

		if(_hasCommand)
		{
			moveTo(_storedPosition);
			_hasCommand = false;
			_storedPosition = 0;
		}
	}

#if DEBUG

	Serial.println("---------run and next---------------");
	Serial.print("step count:");
	Serial.println(_stepCount);
	Serial.print("interval:");
	Serial.println(_stepInterval);

#endif

}

void ProStepper::computeNewInterval()
{
	//equation 3
	long interval = _stepInterval;
	//_stepInterval = _stepInterval - 
	//               ((2*_stepInterval + _stepIntervalRemain)/(4*_accelCount+1)); 
	interval = interval - 
	           (2*interval/(4*_accelCount+1));
 	//_stepIntervalRemain = (2*_stepInterval + _stepIntervalRemain)%(4*_accelCount+1);

	_stepInterval = interval;

#if DEBUG

	Serial.println("++++compute interval+++++");
	Serial.print("accel count:");
	Serial.println(_accelCount);
	Serial.print("new interval:");
	Serial.println(_stepInterval);

#endif
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

#if DEBUG

	Serial.println("--------------new limit----------");
	Serial.print("accel to max speed need:");
	Serial.println(_accelStepsToMaxSpeed);
	Serial.print("accel limit is:");
	Serial.println(_accelStepsLimit);
	Serial.print("accel stop at:");
	Serial.println(_accelStop);
	Serial.print("deaccel start at:");
	Serial.println(_deaccelStart);

#endif 
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

#if DEBUG

		Serial.println("--------direction set-----------");
		Serial.print("direction:");
		Serial.println(_direction);

#endif	
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