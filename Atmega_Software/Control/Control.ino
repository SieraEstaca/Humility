#include <Thread.h>
#include <Motor.h>
#include <Timer.h>

int period = 100;
Thread control = Thread();
Timer get_2_rpi;
Timer set_2_rpi;

Motor leftUP_motor(7, 6);
Motor righUP_motor(9, 8);

float leftUp_ref = 0.0, righUp_ref = 0.0, leftDw_ref = 0.0, righDw_ref = 0.0; 
float leftUp_mes = 0.0, righUp_mes = 0.0, leftDw_mes = 0.0, righDw_mes = 0.0; 

String strReceived, firstValue, secondValue, thirdValue, stringToSend;
int commaIndex, secondCommaIndex;

volatile int _pulse_leftUp = 0, _pulse_righUp = 0;

unsigned long to = 0;
void setup()
{
  // Start serial link with rpi
  Serial.begin(9600);
  delay(1000);

  // Init timer functions for arduino and rpi link
  get_2_rpi.every(period, getRPM_ref);
  set_2_rpi.every(period, setRPM_mes);

  // Set control-command thread
  control.onRun(command);
  control.setInterval(period*2);
  
  // For encoder
  attachInterrupt(digitalPinToInterrupt(20), pulse_leftUp, RISING);
  attachInterrupt(digitalPinToInterrupt(21), pulse_righUp, RISING);
}

void loop()
{
  bidirectional();
  
  if(control.shouldRun())
    control.run();
}

void command(){
  // Left Up Motor 
  leftUP_motor.setRPM(0.0, leftUp_ref);
  leftUp_mes = leftUP_motor.getRPM(int(_pulse_leftUp));
  _pulse_leftUp = 0;

  // Right Up Motor 
  righUP_motor.setRPM(0.0, righUp_ref);
  righUp_mes = righUP_motor.getRPM(int(_pulse_righUp));
  _pulse_righUp = 0;
}

void bidirectional(){
  //if(Serial.available()>0)
  get_2_rpi.update();
  set_2_rpi.update();
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
  stringToSend.concat(String(leftUp_mes, 2));
  stringToSend.concat(",");
  stringToSend.concat(String(righUp_mes, 2));
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

