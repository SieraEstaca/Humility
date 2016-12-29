// Open-Source class
#include <Thread.h>
#include <Timer.h>
// Our designed class
#include <Motor.h>
#include <IRsensor.h>
#include <Params.h>

/* INITIALIZE */ 
IRsensor left_IRsensor(A0);
controlParams leftUp  = {0, 0, 0};    Motor leftUP_motor(7, 6);
controlParams rightUp = {0, 0, 0};    Motor rightUP_motor(10, 11);
controlParams leftDw  = {0, 0, 0};    Motor leftDW_motor(9, 8);
controlParams rightDw = {0, 0, 0};    Motor rightDW_motor(12, 13);
Timer get_2_rpi;
Timer send_2_rpi;
Thread control = Thread();

void setup()
{
  Serial.begin(9600);
  delay(1000);
  
  // Timer setup
  get_2_rpi.every(100, getRPM_ref);
  send_2_rpi.every(100, sendRPM_mes);
  
  // Control thread setup
  control.onRun(command);
  control.setInterval(200);
  
  // Encoder setup
  attachInterrupt(digitalPinToInterrupt(20), pulse_leftUp, RISING);
  attachInterrupt(digitalPinToInterrupt(18), pulse_righUp, RISING);
  attachInterrupt(digitalPinToInterrupt(21), pulse_leftDw, RISING);
  attachInterrupt(digitalPinToInterrupt(19), pulse_righDw, RISING);
}

void loop() /* frequency = 5 Hz */
{
  // Serial communication with Raspberry thread 
  get_2_rpi.update(); 
  send_2_rpi.update();
  
  // Main thread for motor control (and even IR sensor)
  if(control.shouldRun())
    control.run();
}



/*
----- MOTORS, IRsensors THREAD
*/

void command(){ 
  if(rpi){
    leftDist = left_IRsensor.Obstacle();
    
    // Left Up Motor 
    leftUP_motor.setRPM(0.0, leftUp.speed_ref);
    leftUp.speed_mes = leftUP_motor.getRPM(int(leftUp.pulse_count));
    leftUp.pulse_count = 0;
  
    // Right Up Motor 
    rightUP_motor.setRPM(0.0, rightUp.speed_ref);
    rightUp.speed_mes = rightUP_motor.getRPM(int(rightUp.pulse_count));
    rightUp.pulse_count = 0;

    // Left Dw Motor 
    leftDW_motor.setRPM(0.0, leftDw.speed_ref);
    leftDw.speed_mes = leftDW_motor.getRPM(int(leftDw.pulse_count));
    leftDw.pulse_count = 0;
  
    // Right Dw Motor 
    rightDW_motor.setRPM(0.0, rightDw.speed_ref);
    rightDw.speed_mes = rightDW_motor.getRPM(int(rightDw.pulse_count));
    rightDw.pulse_count = 0;
  }
  else{
    leftUP_motor.setRPM(0.0, 0.0);
    rightUP_motor.setRPM(0.0, 0.0);
    leftDW_motor.setRPM(0.0, 0.0);
    rightDW_motor.setRPM(0.0, 0.0);
  }
}

/*
----- PYSERIAL
*/

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
    leftUp.speed_ref = firstValue.toFloat();
    rightUp.speed_ref = secondValue.toFloat();
    leftDw.speed_ref = firstValue.toFloat();
    rightDw.speed_ref = secondValue.toFloat();
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
    Serial.print(String(leftUp.speed_mes, 2) + "," + String(rightUp.speed_mes, 2) + "," + String(leftDist) + "," + String(leftDist) + "\n");
    request = false;
  }
}

/*
----- ENCODER
*/

void pulse_leftUp(){
  leftUp.pulse_count++;
}
void pulse_righUp(){
  rightUp.pulse_count++;
}
void pulse_leftDw(){
  leftDw.pulse_count++;
}
void pulse_righDw(){
  rightDw.pulse_count++;
}
