#include "Arduino.h"
#include "IRsensor.h"

IRsensor::IRsensor(uint8_t analogPin)
{
	// Init all sensors values
	measurement = 0, analogReadMax = 730;
	convert_in_mm = 0.42, dt = 0.0;

	// Setup read pin
	_analogPin = analogPin;
}

float IRsensor::Obstacle()
{
	dt = millis()-dt;
	measurement = int((analogReadMax-analogRead(_analogPin))*convert_in_mm);
	measurement_filt = measurement_filt + dt*(measurement-measurement_filt)/1000;
	dt = millis();
	return measurement_filt;
}
