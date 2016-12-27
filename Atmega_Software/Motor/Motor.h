#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor
{
  public:
	// Constructor
    Motor(int IN1pin, int IN2pin);
	// Functions
    void setRPM(float rpm1, float rpm2);
	float getRPM(int pulses);

  private:
	// Constructor
	int _IN1pin, _IN2pin;
	float _PulsesToRPM, _PWMtoRPM;
	// Functions
	float rpm, rpm_filt, dt, timeold;
	float PulsesToRPM, PWMtoRPM;
};

#endif
