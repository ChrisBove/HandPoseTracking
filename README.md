# HandPoseTracking
RBE 580 Biomedical Robotics Project for IMU-based Hand Pose Tracking Validation

### Arduino Setup

Copy the folders in Arduino libraries into your default sketch location (Usually `Users/userName/Documents/Arduino/libraries` on Windows).

The most recent code (ArduinoInterface_580) can be compiled on an Arduino Mega or Arduino Uno. See [this page](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/wiring-and-test) for details on interfacing the IMU's. One IMU must have its Address pin pulled to 3.3V, changing its address to 0x29, so that both IMU's can operate on the same I2C ports. This configures them on Channels A and B as the Adafruit interface defines them, which correspond to addresses 0x28 and 0x29 respectively. They should also be pulled up with external resistors as [this site](http://tronixstuff.com/2010/10/20/tutorial-arduino-and-the-i2c-bus/) explains.

To compile last year's code unmodified, change the board type: Tools->Board: -> Arduino Yun

### Matlab Setup 

TODO
