/**
*
* Space Rover Software Project
*
* SiERA  : Estaca Robotics Association
* ESTACA : Transportation engineering school (France)
*
*
*/


#include "Arduino.h"
#include "Motor.h"

Motor::Motor(int IN1pin, int IN2pin)
{
	// Get all values
	_IN1pin = IN1pin;
	_IN2pin = IN2pin;

	// Init params
	rpm = 0.0, rpm_filt = 0.0, dt = 0.1, timeold = 0.0;
	PulsesToRPM = 139.0, PWMtoRPM = 7.75;

	// Set Pins
	pinMode(_IN1pin, OUTPUT);
	pinMode(_IN2pin, OUTPUT);

	// Init all PWM pins to LOW                                                                                                                                                                                                                                                                                           tialisation du moteur en mode arret
	digitalWrite(_IN1pin, LOW);
	digitalWrite(_IN2pin, LOW);

}

void Motor::setRPM(float rpm1, float rpm2)
{
	// Protect against motor brake
	if(rpm1 != 0 && rpm2 !=0){
	        rpm1 = 0.f;
		rpm2 = 0.f;
	}

	// Send PWM signals
    	analogWrite(_IN1pin, PWMtoRPM*rpm1);
    	analogWrite(_IN2pin, PWMtoRPM*rpm2);
}

float Motor::getRPM(int pulses){
	 // Calcul RPM
     	if(pulses >= 12){
        	dt = millis()-timeold;
         	rpm = _PulsesToRPM*pulses/dt;
		// Filter value
         	if(rpm >= 0.0 && rpm <= 100.0){
              		rpm = 1.1554*rpm - 0.2193;
              		rpm += -0.0001*rpm*rpm*rpm+0.007*rpm*rpm-0.1503*rpm+1.4415;
              		rpm_filt = rpm_filt+0.2*(rpm-rpm_filt);
         	}
		 // Reinit
         timeold = millis();
         pulses = 0;
     	}

	 // Cancel noise
     	else{
		rpm_filt = 0.0;
     	}

	return rpm_filt;
}
