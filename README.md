# Humility
Space rover design and mechatronics engineering

- - - -

#### Python 2.7 code on Raspberry Pi 3
* I2C for Rpi link with SenseHat (link = https://learn.adafruit.com/adafruits-raspberry-pi-lesson-4-gpio-setup/configuring-i2c)
* Pyserial communication with Arduino Mega2560
* Multithread soft with wheel odometry navigation and finite state machine (fsm) based guidance
* **WARNING** Vision algorithm was not developped and Kalman filter is not activate in the code (the model don't work correctly in some bends)

#### C++ code on Arduino Mega2560 
* Control of each Motors and IRsensor
* Bidirectionnel communication for serial link with rpi
* Use of ArduinoThread

### Do-It-Yourself
