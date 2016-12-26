// Open-Source class
#include <Thread.h>
#include <Timer.h>
// Our designed class
#include <Motor.h>
#include <IRsensor.h>
#include <Params.h>

/* INITIALIZE */ 
IRsensor left_IRsensor(A0);
controlParams leftUp = {0, 0, 0}; Motor leftUP_motor(7, 6);
controlParams rightUp = {0, 0, 0}; Motor rightUP_motor(9, 8);
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
  attachInterrupt(digitalPinToInterrupt(21), pulse_righUp, RISING);
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
  }
  else{
    leftUP_motor.setRPM(0.0, 0.0);
    rightUP_motor.setRPM(0.0, 0.0);
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
    leftUp.speed_ref = firstValue.toFloat();
    rightUp.speed_ref = secondValue.toFloat();
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

void pulse_leftUp(){
  leftUp.pulse_count++;
}
void pulse_righUp(){
  rightUp.pulse_count++;
}
