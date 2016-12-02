/*
 * Modified by RBE 580 Team to work with 2 IMU's on a MEGA
 * 
 */

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
#define USE_LCD false
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
#define NUMSENSORS 2   //The number of sensors connected
#define CAL FALSE  //Set TRUE or FALSE to calibrate or not.
#define CAL_LVL 3 //Desired calibration level. 3 is best, 1 is lowest level. This is ignored if CAL is FALSE 

/*********** Variable Definitions: ********************************************************************/
//Declare an IMU object. Only one is needed because the data is saved externally and sensors are multiplexed.
Adafruit_BNO055 s1 = Adafruit_BNO055(55, 0x28); //Sensor ID 55 for BNO055 I2C address 0x28
Adafruit_BNO055 s2 = Adafruit_BNO055(55, 0x29); // Sensor ID 55 for second address (can only have 2)
Adafruit_BNO055 * sensors[2];
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
//  Wire.begin(); //Initialize I2C Interface

  pinMode(LED, OUTPUT); //Set up the LED to indicate when calibration is finished
  digitalWrite(LED, HIGH); //Turn it off until cal finished.
  digitalWrite(resetPin, HIGH);
  pinMode(resetPin, OUTPUT);
  //digitalWrite(resetPin, HIGH);

  Serial.begin(BAUD); //Initialize serial communication
  while (!Serial) ; // Wait for Serial monitor to open

  if(USE_LCD) {
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
  }
  else {
    Serial.println("startup and init");
  }

  sensors[0] = &s1;
  sensors[1] = &s2;
  
  // Initialize each sensor
  for (int i = 0; i < NUMSENSORS; i++) {
    if (!sensors[i]->begin())
    {
      // There was a problem detecting the BNO055 ... check your connections
      Serial.print(statError); //If the sensor fails to initialize, send an init error
      if(USE_LCD) {
        //Show error and problem sensor on LCD:
        lcd.setCursor(0, 0);
        lcd.print("INIT ERR");
        lcd.setCursor(0, 1);
        lcd.print("Sensor ");
        lcd.print(i);
      }
      else {
        Serial.print("INIT ERR ");
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.println("Reset System.");
      }
      while (1); //Do we want MATLAB to trigger a reset here?
    }
    delay(100);
    
    sensors[0]->setExtCrystalUse(true);
    sensors[1]->setExtCrystalUse(true);
  }

  //**** Wait for successful calibration report on each sensor: ****
  for (int i = 0; i < NUMSENSORS; i++) {
    sensors[i]->getCalibration(&sys, &gyro, &accel, &mag); //Get current calibration
    //Indicate current sensor
    if(USE_LCD) {
      lcd.setCursor(0, 0);
      lcd.print("Cal IMU");
      lcd.print(i);
    }
    else {
      Serial.print("Cal IMU ");
      Serial.println(i);
    }
    //While want to calibrate and sensor is still calbrating, wait for system calibration to reach desired level
    while (CAL && (sys < CAL_LVL)) {
      if(!USE_LCD) Serial.print(i);
      Serial.println(statCal);
      sensors[i]->getCalibration(&sys, &gyro, &accel, &mag);
      if(USE_LCD) {
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
      }
      else {
        Serial.print("S");
        Serial.print(sys);
        Serial.print("G");
        Serial.print(gyro);
        Serial.print("A");
        Serial.print(accel);
        Serial.print("M");
        Serial.println(mag);
      }

      delay(50); //Give value time to change
    }
  }

  Serial.println(statGo); //Tell matlab everything has initialized
  digitalWrite(LED, TRUE); //Turn on LED to indicate cal finished.
  if(USE_LCD){
    lcd.clear();
    lcd.print(" Ready! "); //Show ready on LCD
  }
  else {
    Serial.println("Ready!!!");
  }
}

/**************** Main Loop ***************************************************************************/
void loop(void) {

  if (Serial.available()) { //Wait for new serial command
    char  command = (char)Serial.read();

    switch (command) {

      case getAll: //Get all sensor readings
        for (int i = 0; i < NUMSENSORS; i++) {
          quat = sensors[i]->getQuat(); //get the quaternion orientation
          euler = quat.toEuler(); //convert it to euler angles
          Serial.print(euler.x()*DEG, 0); //Print the x orientation with zero decimal places. Note that euler is a vector of [z,y,x] euler angles...toEuler function returns z y x euler angles
          Serial.print(' ');
          Serial.print(euler.y()*DEG, 0);
          Serial.print(' ');
          Serial.println(euler.z()*DEG, 0);
        }
        break;

      case get1: //Read sensor 1
        quat = sensors[0]->getQuat();
        euler = quat.toEuler();
        Serial.print(euler.x()*DEG, 0); //Print the x orientation with zero decimal places. Note that euler is a vector of [z,y,x] euler angles...toEuler function returns z y x euler angles
        Serial.print(' ');
        Serial.print(euler.y()*DEG, 0);
        Serial.print(' ');
        Serial.println(euler.z()*DEG, 0);
        break;

      case get2: //Read sensor 2
        quat = sensors[1]->getQuat();
        euler = quat.toEuler();
        Serial.print(euler.x()*DEG, 0); //Print the x orientation with zero decimal places. Note that euler is a vector of [z,y,x] euler angles...toEuler function returns z y x euler angles
        Serial.print(' ');
        Serial.print(euler.y()*DEG, 0);
        Serial.print(' ');
        Serial.println(euler.z()*DEG, 0);
        break;

      case rst: //Reset the system upon command
        if(USE_LCD) {
          lcd.clear();
          lcd.print("Dis-");
          lcd.setCursor(0, 1);
          lcd.print("-connect");
        }
        else {
          Serial.println("Disconnect");
        }
        software_Reset();
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

/*
 * from this thread http://forum.arduino.cc/index.php?topic=49581.0
 */
void software_Reset() {
  asm volatile("  jmp 0");
}
