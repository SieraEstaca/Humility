/**
*
* Space Rover Software Project
*
* SiERA  : Estaca Robotics Association
* ESTACA : Transportation engineering school (France)
*
*
*/

#include <Motor.h>
#include <Timer.h>
int chrono = 100;
Timer get_2_rpi;
Timer set_2_rpi;

Motor leftUP_motor(11, 10);
Motor righUP_motor( 9,  8);

float leftUp_ref = 0.0, righUp_ref = 0.0, leftDw_ref = 0.0, righDw_ref = 0.0; // Reference RPM
float leftUp_mes = 0.0, righUp_mes = 0.0, leftDw_mes = 0.0, righDw_mes = 0.0; // Measurement 

String strReceived, firstValue, secondValue, thirdValue, stringToSend;
int commaIndex, secondCommaIndex;

volatile int _pulse_leftUp = 0, _pulse_righUp = 0;

void setup()
{
  // Start serial link with rpi
  Serial.begin(9600);

  // Init timer functions for arduino and rpi link
  get_2_rpi.every(chrono, getRPM_ref);
  set_2_rpi.every(chrono, setRPM_mes);

  // Avoid "NaN" for the first RPM calculation
  delay(1000);

  // For encoder
  attachInterrupt(digitalPinToInterrupt(18), pulse_leftUp, RISING);
  attachInterrupt(digitalPinToInterrupt(19), pulse_righUp, RISING);
}

void loop()
{
  // Rpi - Arduino communicate
  get_2_rpi.update();
  set_2_rpi.update();

  // Left Up Motor 
  leftUP_motor.setRPM(leftUp_ref,0.0);
  leftUp_mes = leftUP_motor.getRPM(int(_pulse_leftUp));
  _pulse_leftUp = 0;
  
  // Right Up Motor 
  righUP_motor.setRPM(righUp_ref,0.0);
  righUp_mes = righUP_motor.getRPM(int(_pulse_righUp));
  _pulse_righUp = 0;
}

void getRPM_ref(){
  // Read new speed rotation references
  strReceived = Serial.readStringUntil('\n');
  
  // Split the string
  commaIndex = strReceived.indexOf(',');
  secondCommaIndex = strReceived.indexOf(',', commaIndex+1);
  firstValue = strReceived.substring(0, commaIndex);
  secondValue = strReceived.substring(commaIndex+1, secondCommaIndex);
  
  // Convert value
  leftUp_ref = firstValue.toFloat();
  righUp_ref = secondValue.toFloat();
}

void setRPM_mes(){
  // Convert in string all encoders data
  stringToSend.concat(String(leftUp_mes, 6));
  stringToSend.concat(",");
  stringToSend.concat(String(righUp_mes, 6));
  stringToSend.concat("\n");

  // Send all data with one line
  Serial.println(stringToSend);
  stringToSend = "";
}

void pulse_leftUp(){
  _pulse_leftUp++;
}
void pulse_righUp(){
  _pulse_righUp++;
}

