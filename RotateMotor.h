#ifndef RotateMotor_h
#define RotateMotor_h

#include "DCMotor.h"

class RotateMotor
{
	public:
		RotateMotor(DCMotor *motor, int encd1, int encd2);

		void init();

		void rotateWithSpeed(long speed);
		long getRotateSpeed();
		long getDesiredSpeed();

		bool run();
		void stop();

	private:
		void updateSpeed();
		void startInterrupt();
		void stopInterrupt();

		//state information
		DCMotor *_motor;
		long _targetSpeed;   //rpm
		long _currentSpeed;

		//control state
		long _diff;
		long _lastDiff;
		long _diffDiff;
		long _integ;
		long _control;

		//encoder wiring
		int _encd1;
		int _encd2;	
};

#endif