# Humility
Final year project : Space rover design and mechatronics engineering

- - - -

#### Python 2.7 code on Raspberry Pi 3
* I2C for Rpi link with SenseHat 
* Pyserial communication with Arduino Mega2560
* Multithread soft with wheel odometry navigation and finite state machine (fsm) based guidance

#### C++ code on Arduino Mega2560 
* Control of each Motors and IRsensor
* Bidirectionnel communication for serial link with rpi
* Use of ArduinoThread

#### Warnings 
* Vision algorithm was not developped even if OpenCV and a image process thread is on the code
* Kalman filter is not activate in the code (the model don't work correctly in some bends)
* Servomotors were not used in our rover beacuse of electronical overheat on power supply board

#### Photos
![Alt Text](https://github.com/SieraEstaca/Humility/blob/master/Test_Rover.jpg)
![Alt Text](https://github.com/SieraEstaca/Humility/blob/master/Final_Rover.JPG)

### Do-It-Yourself
