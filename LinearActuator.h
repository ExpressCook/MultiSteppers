#ifndef LinearActuator_h
#define LinearActuator_h

#include <Arduino.h>
#include "DCMotor.h"

class LinearActuator
{
	public:
		LinearActuator(DCMotor motor, int pinPos);
		
		void init();
		void setSpeed(int speed);

		void moveTo(long absolute);
		void move(long relative);
		bool run();
		void stop();

		unsigned int getCurrent();
		long getCurrentPos();
		long getTargetPos();

	private:
		void updatePos();

		int _pinPos;
		DCMotor _motor;

		int _speed;
		long _position;
		long _targetPosition;

		long _Kp;
		long _Ki;
		long _Kd;

		long _diff;
		long _lastDiff;
		long _diffDiff;
		long _integ;
		long _control;
};

#endif