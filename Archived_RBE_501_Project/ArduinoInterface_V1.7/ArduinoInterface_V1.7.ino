/*******************************************************************************************************
   Arduino interface for RBE 501, Robot Dynamics Hand Pose Estimation Project
   Version 1.7  Last modified by JWilder on 4/21/2016
   Team Members: Joseph Brown, Jeffrey Miscione, Kunal Patel, Adela Wee, Jeffrey Wilder
   Contributor(s): Jeffrey Wilder
********************************************************************************************************

   This firmware runs on the SAMD21 MCU (Sparkfun SAMD21 Mini Breakout) that provides an interface with
   the sensors used in this project. It is intended to communicate via serial link with a Matlab
   interface which is responsible for the data collection and calculation.

   The software is set up for 4 Adafruit BNO055 IMUs (up to 8 with modifications), connected via a
   TCA9548A I2C Multiplexer to allow all sensors to use the same address.

   Calibration is automatic and handled by the sensors themselves. The MCU waits for all sensors to
   report adequate calibration (in the mean time, sending statCal) before sending the "go" command
   to the Matlab interface indicating that all sensors are calibrated and ready to be used.

*******************************************************************************************************/

/****************** Macros: ***************************************************************************/
#include <Wire.h> //I2C Library
#include <Adafruit_Sensor.h> //Adafruit sensor library used for sensor interface datatypes
#include <Adafruit_BNO055.h> //BNO055 Library 
#include <utility/imumaths.h> //Another support library for IMUs
#include <utility/quaternion.h> //Needed for toEuler() function
#include <LiquidCrystal.h> //LCD library
#include <math.h> //Used for M_PI constant

//Define matlab commands:
//Input Commands:
#define getAll 'a' //Read all sensors
#define get1 '1' //Read sensor 1
#define get2 '2' //Read sensor 2
#define get3 '3' //Read sensor 3
#define get4 '4' //Read sensor 4
#define rst 'R' //Execute a software reset
//Output Status Indicators:
#define statGo "GO!" //Tell Matlab sensors are ready
#define statCal "CAL" //Still calibrating
#define statError "ERR" //Error initializing one or more sensors

//MISC definitions:
#define TRUE 1
#define FALSE 0
#define RAD M_PI/180 //Multiply by RAD to convert degrees to radians
#define DEG 180/M_PI //Multiply by DEG to convert radians to degrees
#define LED 13 //Status indicator LED is on pin 13
#define resetPin 2 //Pin 2 is tied to the reset pin of the MCU, and setting it low resets the system
#define LCD_RS 3 //LCD pins
#define LCD_RW 4
#define LCD_EN 5
#define LCD_D4 6
#define LCD_D5 7
#define LCD_D6 8
#define LCD_D7 9
#define LCD_NUMROWS 2
#define LCD_NUMCOLS 8

//Configuration Macros:
#define BAUD 115200    //The desired baud rate for the serial communication
#define MUX_ADDR 0x70  //The address of the TCA9548A I2C Multiplexer
#define NUMSENSORS 4   //The number of sensors connected
#define CAL TRUE  //Set TRUE or FALSE to calibrate or not.
#define CAL_LVL 3 //Desired calibration level. 3 is best, 1 is lowest level. This is ignored if CAL is FALSE 

/*********** Variable Definitions: ********************************************************************/
//Declare an IMU object. Only one is needed because the data is saved externally and sensors are multiplexed.
Adafruit_BNO055 s = Adafruit_BNO055(55, 0x28); //Sensor ID 55 for BNO055 I2C address 0x28
imu::Quaternion quat; //Quaternion variable
imu::Vector<3> euler; //Euler angle vector calculated from quaternion

//Calibration values
uint8_t sys = 0;
uint8_t gyro = 0;
uint8_t accel = 0;
uint8_t mag = 0;

// initialize the LCD with the numbers of the interface pins
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7); //RS, EN, D4, D5, D6, D7

char calStr[18]; //Declare an empty string for

/************** Setup Function ************************************************************************/
void setup(void)
{
  Wire.begin(); //Initialize I2C Interface

  pinMode(LED, OUTPUT); //Set up the LED to indicate when calibration is finished
  digitalWrite(LED, HIGH); //Turn it off until cal finished.
  digitalWrite(resetPin, HIGH);
  pinMode(resetPin, OUTPUT);
  //digitalWrite(resetPin, HIGH);

  SerialUSB.begin(BAUD); //Initialize serial communication
  while (!SerialUSB) ; // Wait for Serial monitor to open

  //*** Set up the LCD: ****
  //Ground the R/W pin:
  pinMode(LCD_RW, OUTPUT);
  digitalWrite(LCD_RW, 0);

  // set up the LCD's number of columns and rows:
  lcd.begin(LCD_NUMCOLS, LCD_NUMROWS);
  lcd.clear();
  // Indicate initialization
  lcd.print("Startup ");
  lcd.setCursor(0, 1);
  lcd.print("and Init");

  // Initialize each sensor
  for (int i = 1; i <= NUMSENSORS; i++) {
    sensorSelect(i); //Select sensor 1
    if (!s.begin())
    {
      // There was a problem detecting the BNO055 ... check your connections
      SerialUSB.print(statError); //If the sensor fails to initialize, send an init error
      //Show error and problem sensor on LCD:
      lcd.setCursor(0, 0);
      lcd.print("INIT ERR");
      lcd.setCursor(0, 1);
      lcd.print("Sensor ");
      lcd.print(i);
      while (1); //Do we want MATLAB to trigger a reset here?
    }
    delay(100);
    s.setExtCrystalUse(true);
  }

  //**** Wait for successful calibration report on each sensor: ****
  for (int i = 1; i <= NUMSENSORS; i++) {
    sensorSelect(i);
    s.getCalibration(&sys, &gyro, &accel, &mag); //Get current calibration
    //Indicate current sensor
    lcd.setCursor(0, 0);
    lcd.print("Cal IMU");
    lcd.print(i);
    //While want to calibrate and sensor is still calbrating, wait for system calibration to reach desired level
    while (CAL && (sys < CAL_LVL)) {
      SerialUSB.println(statCal);
      s.getCalibration(&sys, &gyro, &accel, &mag);
      //Print current calibration status to LCD
      lcd.setCursor(0, 1);
      lcd.print("S");
      lcd.print(sys);
      lcd.print("G");
      lcd.print(gyro);
      lcd.print("A");
      lcd.print(accel);
      lcd.print("M");
      lcd.print(mag);

      delay(50); //Give value time to change
    }
  }

  SerialUSB.println(statGo); //Tell matlab everything has initialized
  digitalWrite(LED, TRUE); //Turn on LED to indicate cal finished.
  lcd.clear();
  lcd.print(" Ready! "); //Show ready on LCD
}

/**************** Main Loop ***************************************************************************/
void loop(void) {

  if (SerialUSB.available()) { //Wait for new serial command
    char  command = (char)SerialUSB.read();

    switch (command) {

      case getAll: //Get all sensor readings
        for (int i = 1; i <= NUMSENSORS; i++) {
          sensorSelect(i); //Select the i'th sensor
          quat = s.getQuat(); //get the quaternion orientation
          euler = quat.toEuler(); //convert it to euler angles
          SerialUSB.print(euler.x()*DEG, 0); //Print the x orientation with zero decimal places. Note that euler is a vector of [z,y,x] euler angles...toEuler function returns z y x euler angles
          SerialUSB.print(' ');
          SerialUSB.print(euler.y()*DEG, 0);
          SerialUSB.print(' ');
          SerialUSB.println(euler.z()*DEG, 0);
        }
        break;

      case get1: //Read sensor 1
        sensorSelect(1);
        quat = s.getQuat();
        euler = quat.toEuler();
        SerialUSB.print(euler.x()*DEG, 0); //Print the x orientation with zero decimal places. Note that euler is a vector of [z,y,x] euler angles...toEuler function returns z y x euler angles
        SerialUSB.print(' ');
        SerialUSB.print(euler.y()*DEG, 0);
        SerialUSB.print(' ');
        SerialUSB.println(euler.z()*DEG, 0);
        break;

      case get2: //Read sensor 2
        sensorSelect(2);
        quat = s.getQuat();
        euler = quat.toEuler();
        SerialUSB.print(euler.x()*DEG, 0); //Print the x orientation with zero decimal places. Note that euler is a vector of [z,y,x] euler angles...toEuler function returns z y x euler angles
        SerialUSB.print(' ');
        SerialUSB.print(euler.y()*DEG, 0);
        SerialUSB.print(' ');
        SerialUSB.println(euler.z()*DEG, 0);
        break;

      case get3: //Read sensor 3
        sensorSelect(3);
        quat = s.getQuat();
        euler = quat.toEuler();
        SerialUSB.print(euler.x()*DEG, 0); //Print the x orientation with zero decimal places. Note that euler is a vector of [z,y,x] euler angles...toEuler function returns z y x euler angles
        SerialUSB.print(' ');
        SerialUSB.print(euler.y()*DEG, 0);
        SerialUSB.print(' ');
        SerialUSB.println(euler.z()*DEG, 0);
        break;

      case get4: //Read sensor 4
        sensorSelect(4);
        quat = s.getQuat();
        euler = quat.toEuler();
        SerialUSB.print(euler.x()*DEG, 0); //Print the x orientation with zero decimal places. Note that euler is a vector of [z,y,x] euler angles...toEuler function returns z y x euler angles
        SerialUSB.print(' ');
        SerialUSB.print(euler.y()*DEG, 0);
        SerialUSB.print(' ');
        SerialUSB.println(euler.z()*DEG, 0);
        break;

      case rst: //Reset the system upon command
        lcd.clear();
        lcd.print("Dis-");
        lcd.setCursor(0, 1);
        lcd.print("-connect");
        digitalWrite(resetPin, LOW);
        break;

      default:
        break;
    }
  }
}

/************* Functions ******************************************************************************/

/*
   sensorSelect() is used to control the I2C multiplexer to select the current sensor.
   It requires the sensor number (range from 0 to 7) as a parameter.
*/
void sensorSelect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

