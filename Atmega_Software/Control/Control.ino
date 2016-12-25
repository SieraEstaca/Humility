// Open-Source class
#include <Thread.h>
#include <ThreadController.h>
#include <Timer.h>
// Our designed class
#include <Motor.h>
#include <IRsensor.h>

// For IR sensor
IRsensor left_IRsensor(A0);
int leftDist = 0, righDist = 0;

// For control loop
Thread control = Thread();
// ThreadController controller = ThreadController();
Motor leftUP_motor(7, 6);
Motor righUP_motor(9, 8);
float leftUp_ref = 0.0, righUp_ref = 0.0, leftDw_ref = 0.0, righDw_ref = 0.0; 
float leftUp_mes = 0.0, righUp_mes = 0.0, leftDw_mes = 0.0, righDw_mes = 0.0;
volatile int _pulse_leftUp = 0, _pulse_righUp = 0; 

// For serial communication
Timer get_2_rpi;
Timer send_2_rpi;
bool request = false, rpi = true;
int period = 100, counter = 0;
String strReceived, firstValue, secondValue, thirdValue, fourthValue;
int commaIndex, secondCommaIndex, thirdCommaIndex, fourthCommaIndex;

void setup()
{
  Serial.begin(9600);
  delay(1000);
  
  // Init timer functions for arduino and rpi link
  get_2_rpi.every(period, getRPM_ref);
  send_2_rpi.every(period, sendRPM_mes);
  
  // Set control-command thread
  control.onRun(command);
  control.setInterval(period*2);
  // controller.add(&control);
  
  // For encoder
  attachInterrupt(digitalPinToInterrupt(20), pulse_leftUp, RISING);
  attachInterrupt(digitalPinToInterrupt(21), pulse_righUp, RISING);
}

void loop()
{
  // Serial link between Python (Raspberry in our case) and Arduino
  get_2_rpi.update(); 
  send_2_rpi.update();
  
  // Main thread for motor control (5Hz) 
  if(control.shouldRun())
    control.run();
}

void command(){ 
  if(rpi){
    leftDist = left_IRsensor.Obstacle();
    
    // Left Up Motor 
    leftUP_motor.setRPM(0.0, leftUp_ref);
    leftUp_mes = leftUP_motor.getRPM(int(_pulse_leftUp));
    _pulse_leftUp = 0;
  
    // Right Up Motor 
    righUP_motor.setRPM(0.0, righUp_ref);
    righUp_mes = righUP_motor.getRPM(int(_pulse_righUp));
    _pulse_righUp = 0;
  }
  else{
    leftUP_motor.setRPM(0.0, 0.0);
    righUP_motor.setRPM(0.0, 0.0);
  }
}

void getRPM_ref(){
  if(Serial.available()>0){
    // Read new speed rotation references
    strReceived = Serial.readStringUntil('\n');
    
    // Split the string
    commaIndex = strReceived.indexOf(',');
    secondCommaIndex = strReceived.indexOf(',', commaIndex+1);
    thirdCommaIndex = strReceived.indexOf(',', secondCommaIndex+1);
    firstValue = strReceived.substring(0, commaIndex);
    secondValue = strReceived.substring(commaIndex+1, secondCommaIndex);
    thirdValue = strReceived.substring(secondCommaIndex+1, thirdCommaIndex);
    
    // Convert value
    leftUp_ref = firstValue.toFloat();
    righUp_ref = secondValue.toFloat();
    request = thirdValue;

    counter=0;
    rpi = true;
 }
 else{
    counter++;
    if(counter==30)
      rpi = false;
 }
}

void sendRPM_mes(){
  if(request && Serial.available()==0){
    Serial.print(String(leftUp_mes, 2) + "," + String(righUp_mes, 2) + "," + String(leftDist) + "," + String(leftDist) + "\n");
    request = false;
  }
}

void pulse_leftUp(){
  _pulse_leftUp++;
}
void pulse_righUp(){
  _pulse_righUp++;
}
