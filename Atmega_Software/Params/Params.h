#ifndef Params_h
#define Params_h

#include "Arduino.h"

// Synchronize parameters
bool request = false;
bool rpi = true;
int counter = 0;

// IR sensor parameters
int leftDist = 0;
int rightDist = 0;

// Serial parameters
String strReceived, firstValue, secondValue, thirdValue;
int commaIndex, secondCommaIndex, thirdCommaIndex, fourthCommaIndex;

// Control parameters
struct controlParams{
	float speed_ref;
	float speed_mes;
	volatile int pulse_count; 
};

#endif