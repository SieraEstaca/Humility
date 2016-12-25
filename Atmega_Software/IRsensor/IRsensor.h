#ifndef IRsensor_h
#define IRsensor_h

#include "Arduino.h"

class IRsensor
{
  public:
    IRsensor(uint8_t analogPin);
	float Obstacle();

  private:
	int measurement, measurement_filt, analogReadMax;
	float convert_in_mm, dt;
	uint8_t _analogPin;
};

#endif
