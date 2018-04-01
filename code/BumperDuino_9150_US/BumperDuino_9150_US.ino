/*
  Ardumower (www.ardumower.de)
  Copyright (c) 2013-2015 by Alexander Grau
  Copyright (c) 2013-2015 by Sven Gennat
  Copyright (c) 2014 by Maxime Carpentieri    
  Copyright (c) 2015 by Uwe Zimprich
  Copyright (c) 2015 by Frederic Goddeeres
  Copyright (c) 2015 by Jürgen Lange
  
  
  Autor: Jürgen Lange
  Stand: 19.08.2015
  Version: 0.04 Testversion
  
  Private-use only! (you need to ask for a commercial-use)
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
  Private-use only! (you need to ask for a commercial-use)
 */
 
 
// I2C device class (I2Cdev) demonstration Arduino sketch for MPU9150
// 1/4/2013 original by Jeff Rowberg <jeff@rowberg.net> at https://github.com/jrowberg/i2cdevlib
//          modified by Aaron Weiss <aaron@sparkfun.com>
//          and by Explore Labs <info@explorelabs.com>
//
// Changelog:
//     2011-10-07 - initial release
//     2013-01-04 - added raw magnetometer output
//     2014-09-23 - Explore Labs IMU 9DoF AHRS compatibility

/* ============================================
 I2Cdev device library code is placed under the MIT license
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ===============================================
/******************************************
	PURPOSE:		Test of IMU Filtering with MPU9150 and simple complementary filtering 
	Created by              Tim Strohbach 
	DATE:			08/2015
*******************************************/
// MPU-9150 Accelerometer + Gyro + Compass
// -----------------------------
// Code adapted and combined from:
// http://playground.arduino.cc/Main/MPU-9150
// https://github.com/kriswiner/MPU-9150
// http://www.instructables.com/id/Accelerometer-Gyro-Tutorial/step3/Combining-the-Accelerometer-and-Gyro/
// https://aerospace.honeywell.com/~/media/Images/Plymouth%20Website%20PDFs/Magnetic%20Sensors/Technical%20Articles/Applications_of_Magnetic_Sensors_for_Low_Cost_Compass_Systems.ashx
//
// Datasheets available here under 'download documents':
// http://www.invensense.com/products/motion-tracking/9-axis/mpu-9150/
//
// Hardware setup:
// MPU9150 Breakout --------- Arduino
// VCC --------------------- 3.3V
// SDA ----------------------- A4 (4k7 Ohm to 3.3v)
// SCL ----------------------- A5 (4k7 Ohm to 3.3v)
// GND ---------------------- GND
// AD0 ---------------------- GND (by 2k2 Ohm)
// -->EDA, ECL, INT are not connected
//
//
// The general used coordinate system is given below. 
// X-Axis is pointing FORWARD in flight direction
// Y-Axis is pointing to the RIGHT side of the AC
// Z-Axis is pointing DOWNWARDS
// Rotation around x-axis is ROLL and clockwise positive OR a roll to the right is positive
// Rotation around y-axis is PITCH and clockwise positive OR nose up is positive
// Rotation around z-axis is YAW and clockwise positive OR a turn to east is positive
// 
// NOTE: The reference will be the coordinate system of the Magnetometer of the MPU9150 
//       Hence PRINTED X,Y on sensor are MISLEADING --> Change x and y in mind!!!!

#include <Wire.h>
#include "RunningMedian.h"
#include <avr/wdt.h>
#include "PinChangeInterrupt.h"
#include <Pin.h>  // Include Pin Library
// Register names according to the datasheet.
// According to the InvenSense document
// "MPU-9150 Register Map and Descriptions Revision 4.0",


#define MPU9150_XA_OFFSET_H        0x06 // User-defined trim values for accelerometer, populate with calibration routine
#define MPU9150_XA_OFFSET_L_TC     0x07
#define MPU9150_YA_OFFSET_H        0x08
#define MPU9150_YA_OFFSET_L_TC     0x09
#define MPU9150_ZA_OFFSET_H        0x0A
#define MPU9150_ZA_OFFSET_L_TC     0x0B
#define MPU9150_SELF_TEST_X        0x0D   // R/W
#define MPU9150_SELF_TEST_Y        0x0E   // R/W
#define MPU9150_SELF_TEST_Z        0x0F   // R/W
#define MPU9150_SELF_TEST_A        0x10   // R/W
#define MPU9150_XG_OFFS_USRH       0x13  // User-defined trim values for gyroscope, populate with calibration routine
#define MPU9150_XG_OFFS_USRL       0x14
#define MPU9150_YG_OFFS_USRH       0x15
#define MPU9150_YG_OFFS_USRL       0x16
#define MPU9150_ZG_OFFS_USRH       0x17
#define MPU9150_ZG_OFFS_USRL       0x18
#define MPU9150_SMPLRT_DIV         0x19   // R/W
#define MPU9150_CONFIG             0x1A   // R/W
#define MPU9150_GYRO_CONFIG        0x1B   // R/W
#define MPU9150_ACCEL_CONFIG       0x1C   // R/W
#define MPU9150_FF_THR             0x1D   // R/W
#define MPU9150_FF_DUR             0x1E   // R/W
#define MPU9150_MOT_THR            0x1F   // R/W
#define MPU9150_MOT_DUR            0x20   // R/W
#define MPU9150_ZRMOT_THR          0x21   // R/W
#define MPU9150_ZRMOT_DUR          0x22   // R/W
#define MPU9150_FIFO_EN            0x23   // R/W
#define MPU9150_I2C_MST_CTRL       0x24   // R/W
// Slave control registers from 0x25-0x35 not needed
#define MPU9150_I2C_MST_STATUS     0x36   // R
#define MPU9150_INT_PIN_CFG        0x37   // R/W
#define MPU9150_INT_ENABLE         0x38   // R/W
#define MPU9150_INT_STATUS         0x3A   // R 
// Data registers
#define ACCEL_XOUT_H       0x3B   // R  
#define ACCEL_XOUT_L       0x3C   // R  
#define ACCEL_YOUT_H       0x3D   // R  
#define ACCEL_YOUT_L       0x3E   // R  
#define ACCEL_ZOUT_H       0x3F   // R  
#define ACCEL_ZOUT_L       0x40   // R  
#define TEMP_OUT_H         0x41   // R  
#define TEMP_OUT_L         0x42   // R  
#define GYRO_XOUT_H        0x43   // R  
#define GYRO_XOUT_L        0x44   // R  
#define GYRO_YOUT_H        0x45   // R  
#define GYRO_YOUT_L        0x46   // R  
#define GYRO_ZOUT_H        0x47   // R  
#define GYRO_ZOUT_L        0x48   // R 

#define MPU9150_EXT_SENS_DATA_00   0x49   // R  
#define MPU9150_EXT_SENS_DATA_01   0x4A   // R  
#define MPU9150_EXT_SENS_DATA_02   0x4B   // R  
#define MPU9150_EXT_SENS_DATA_03   0x4C   // R  
#define MPU9150_EXT_SENS_DATA_04   0x4D   // R  
#define MPU9150_EXT_SENS_DATA_05   0x4E   // R  
#define MPU9150_EXT_SENS_DATA_06   0x4F   // R  
#define MPU9150_EXT_SENS_DATA_07   0x50   // R  
#define MPU9150_EXT_SENS_DATA_08   0x51   // R  
#define MPU9150_EXT_SENS_DATA_09   0x52   // R  
#define MPU9150_EXT_SENS_DATA_10   0x53   // R  
#define MPU9150_EXT_SENS_DATA_11   0x54   // R  
#define MPU9150_EXT_SENS_DATA_12   0x55   // R  
#define MPU9150_EXT_SENS_DATA_13   0x56   // R  
#define MPU9150_EXT_SENS_DATA_14   0x57   // R  
#define MPU9150_EXT_SENS_DATA_15   0x58   // R  
#define MPU9150_EXT_SENS_DATA_16   0x59   // R  
#define MPU9150_EXT_SENS_DATA_17   0x5A   // R  
#define MPU9150_EXT_SENS_DATA_18   0x5B   // R  
#define MPU9150_EXT_SENS_DATA_19   0x5C   // R  
#define MPU9150_EXT_SENS_DATA_20   0x5D   // R  
#define MPU9150_EXT_SENS_DATA_21   0x5E   // R  
#define MPU9150_EXT_SENS_DATA_22   0x5F   // R  
#define MPU9150_EXT_SENS_DATA_23   0x60   // R  
#define MPU9150_MOT_DETECT_STATUS  0x61   // R  
#define MPU9150_I2C_SLV0_DO        0x63   // R/W
#define MPU9150_I2C_SLV1_DO        0x64   // R/W
#define MPU9150_I2C_SLV2_DO        0x65   // R/W
#define MPU9150_I2C_SLV3_DO        0x66   // R/W
#define MPU9150_I2C_MST_DELAY_CTRL 0x67   // R/W
#define MPU9150_SIGNAL_PATH_RESET  0x68   // R/W
#define MPU9150_MOT_DETECT_CTRL    0x69   // R/W
#define MPU9150_USER_CTRL          0x6A   // R/W
#define MPU9150_PWR_MGMT_1         0x6B   // R/W
#define MPU9150_PWR_MGMT_2         0x6C   // R/W
#define MPU9150_FIFO_COUNTH        0x72   // R/W
#define MPU9150_FIFO_COUNTL        0x73   // R/W
#define MPU9150_FIFO_R_W           0x74   // R/W
#define MPU9150_WHO_AM_I           0x75   // R

//Adresses for DIRECT ACCESS of the AK8975A
#define AK8975A_ADDRESS  0x0C  // Adress of Compass in MPU9150 (see PS MPU9150 page 28)
#define WHO_AM_I_AK8975A 0x00 // should return 0x48
#define INFO             0x01
#define AK8975A_ST1      0x02  // data ready status bit 0
#define AK8975A_XOUT_L	 0x03  // data
#define AK8975A_XOUT_H	 0x04
#define AK8975A_YOUT_L	 0x05
#define AK8975A_YOUT_H	 0x06
#define AK8975A_ZOUT_L	 0x07
#define AK8975A_ZOUT_H	 0x08
#define AK8975A_ST2      0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8975A_CNTL     0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8975A_ASTC     0x0C  // Self test control
#define AK8975A_ASAX     0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8975A_ASAY     0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8975A_ASAZ     0x12  // Fuse ROM z-axis sensitivity adjustment value

// I2C addresses:
#define MPU9150_ADDRESS 0x68      //Adress of MPU6050 is 0x68 (AD0 set to Ground, see PS MPU9150 page 15)

// Definition of global constant variables
const float pi = PI;
const float RadToDeg = 180/PI;
// Declination Angle is 2 deg 11 min POSITIVE (to east) at 27412 Germany, can be found on  http://www.magnetic-declination.com/
const float declinationAngle = 2.217;  


/*** Specify sensor full scale and scale resolutions per LSB for the sensors ***/
// Possible accelerometer scales (and their register bit settings) are:
// 2 G (00), 4 G (01), 8 G (10), and 16 G  (11).
#define aRange 2.0       // 2G acceleration resolution (max. sensitivity)
#define Ascale 0         // Scale bit setting as 'int' --> e.g. 3 = 0b11 or 1 = 0b01
// Possible gyro scales (and their register bit settings) are
// 250 °/s (00), 500 °/s (01), 1000 °/s (10), and 2000 °/s  (11). 
#define gRange 250.0     // 250°/s gyro resolution (max. sensitivity)
#define Gscale 0         // Scale bit setting as 'int' --> e.g. 3 = 0b11 or 1 = 0b01

const float accRes = aRange/32768.0;  // 16 bit, 2G --> 0,000061035 g = per LSB
const float gyroRes = gRange/32768.0; // 16 bit, 250 deg/sec --> = per LSB
const float compRes = 1229.0/4096.0;  // = 0,3µT per LSB --> 1 Milligauss [mG] =   0,1 Mikrotesla [µT]

//Variables where our values can be stored
int temp;
int16_t accRaw[3];
int16_t gyroRaw[3];
int16_t compRaw[3];
int16_t compRaw_Filter[3];
int16_t tempRaw[1];

float magCalibration[3] = {0.0,0.0,0.0};  // Factory mag calibration scale sensitivity values
// User environmental x,y,z-axis correction in milliGauss (not in the raw values!)
//float magBias[3] = {-5.0, 3.5, -18.0}; //Values for MPU9150 Sensor 1
float magBias[3] = {23.0, 22, -24};     //Values for MPU9150 Sensor 2
boolean readMagDatatoggled = false;
float accG[3];
float gyroDegPerSec[3];
float gzRad;
float compMicroT[3];
float ax, ay, az, gx, gy, gz, mx, my, mz, mx_h, my_h; // variables to hold sensor data above just for better overview here


float pitchAcc, rollAcc, yawMag;  //Define variables for the "raw" angles calculated directly from the Acc values
//Define variables for the angles incl. start values. Those angles are in degrees
float pitch = 0;
float roll = 0;
float yaw = 0;  
//roll and pitch are needed in radians for compass tilt compensation
float pitchRad, rollRad;

float temperature = -272.15;
//Definition of variables for time and integral time calculation
uint32_t now, lastupdate;
double dT; //need to be precise due to short time intervalls

float a_coeff = 0.95; // Coefficient for sensor fusion (how much weight put on the angle integration --> short term trust to gyro)
float a_CompFilter = 0.9; // Coefficient for low pass filtering the raw comp data --> e.g. 0.9 is buidling the floating average of the last 10 values(?)

boolean turnPiPlus = false;
boolean turnPiMinus = false;
boolean turnPiSmall = false;


RunningMedian pitchMedian = RunningMedian(5);
RunningMedian rollMedian = RunningMedian(5);


// These constants won't change.  They're used to give names
// to the pins used:
const int sensor1InPin = A0;  // Analog input pin that the MPX5010 is attached to
const int sensor2InPin = A1;  // Analog input pin that the MPX5010 is attached to
//const int analogOut1Pin = 9; // Analog output pin Bumper 1 (PWM)
//const int analogOut2Pin = 10; // Analog output pin Bumper 2 (PWM)
const int uSPinLeft = 6;
const int uSPinCenter = 7;
const int uSPinRight = 8;
const int Bumper1OutPin = 9;
const int Bumper2OutPin = 10;
const int LEDCollision1 = 11;
const int LEDCollision2 = 12;
const int LEDActive = 13;
const int LEDFault = 16;
const int LEDok = 17;
int LEDActiveState = LOW;             // ledState used to set the LED

int sensor1Diff = 0;  // = sensor1DiffSecond - sensor1DiffFirst
int sensor1DiffFirst = 0;        // value for zero Point
int sensor1DiffSecond = 0;        // value for touch Point
int sensor1MAP = 0;        // value output to the PWM (analog out)
int sensor1Trigger = 5;  // Sensor-Trigger-Level Sensor 1 +++++++ Adjust sensitivity here +++++++++
int trigger1Counter = 0; // Trigger-Counter Sensor 1
byte sensor1State = 0; // Sensor-State Sensor 1 zero point or touch point
byte count1FirstRead = 0; // Read-Counter for zero point Sensor 1
byte count1SecondRead = 0; // Read-Counter for touch point Sensor 1
byte flagAdc0Read = 0; // Flag its Time to read ADC Sensor 1
int soundBumper1 = 0;

int sensor2Diff = 0;  // = sensor2DiffSecond - sensor2DiffFirst
int sensor2DiffFirst = 0;        // value for zero Point
int sensor2DiffSecond = 0;       // value for touch Point
int sensor2MAP = 0;        // value output to the PWM (analog out)
int sensor2Trigger = 5;  // Sensor-Trigger-Level Sensor 2 +++++++ Adjust sensitivity here +++++++++
int trigger2Counter = 0;  // Trigger-Counter Sensor 2
byte sensor2State = 0;  // Sensor-State Sensor 2 zero point or touch point
byte count2FirstRead = 0;  // Read-Counter for zero point Sensor 2
byte count2SecondRead = 0;  // Read-Counter for touch point Sensor 2
byte flagAdc1Read = 0;  // Flag its Time to read ADC Sensor 2
int soundBumper2 = 2;
boolean bumperLeftTrigger = false;
boolean bumperRightTrigger = false;
boolean lastBumperLeftTrigger = false;
boolean lastBumperRightTrigger = false;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;        // will store last time Timer was updated

const unsigned long intervalLedActive = 250; // interval at which LED was blink (milliseconds)
unsigned long previousMillisLedActive = 0;




volatile unsigned long pulseLengthUSLeft = 300;
volatile unsigned long lastPulseLengthUSLeft = 300;
volatile unsigned long pulseStartUSLeft = 0;
volatile bool USLeftTriggerActive = false;

volatile unsigned long pulseLengthUSCenter = 300;
volatile unsigned long lastPulseLengthUSCenter = 300;
volatile unsigned long pulseStartUSCenter = 0;
volatile bool USCenterTriggerActive = false;

volatile unsigned long pulseLengthUSRight = 300;
volatile unsigned long lastPulseLengthUSRight = 300;
volatile unsigned long pulseStartUSRight = 0;
volatile bool USRightTriggerActive = false;

float USLeftDistance = 0;
float USCenterDistance = 0;
float USRightDistance = 0;

unsigned long lastUSTriggerTime = 0;


Pin myPinUSLeft = Pin(uSPinLeft);
Pin myPinUSCenter = Pin(uSPinCenter);
Pin myPinUSRight = Pin(uSPinRight);
enum {
  SEN_SONAR_CENTER,      // 0..SONAR_TRIGGER_DISTANCE
  SEN_SONAR_LEFT,        // 0..SONAR_TRIGGER_DISTANCE
  SEN_SONAR_RIGHT,       // 0..SONAR_TRIGGER_DISTANCE
};
char senSonarTurn = SEN_SONAR_RIGHT;
RunningMedian sonarDistCenterMedian = RunningMedian(5);
RunningMedian sonarDistRightMedian = RunningMedian(5);
RunningMedian sonarDistLeftMedian = RunningMedian(5);


const long resetInterval = 300000UL;
unsigned long previousMillisReset = 0;
unsigned long previousMillisPrint = 0;
unsigned long previousMillisMpu = 0;
const long printInterval = 25;
const long mpuInterval = 15;
const long interval = 25;           // interval at which ADC was read (milliseconds)
const long USTriggerInterval = 30;
//================================== SETUP ==========================================
void setup() 
{
  WatchDog_Setup();
    // initialize serial communications at 19200 bps:
  Serial.begin(115200);
    // initialize device
        // join I2C bus (I2Cdev library doesn't do this automatically)
  
  wdt_reset(); 
    // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  digitalWrite(A4,LOW); //Disable pullup to 5V for SDA line
  digitalWrite(A5,LOW); //Disable pullup to 5V for SCL line
  //*** Init the MPU9150 ***/
  // Reset the device to defaults
  writeByte(MPU9150_ADDRESS, MPU9150_PWR_MGMT_1, 0x80); delay(100);
  // Clear the 'sleep' bit to start the sensor and give the device a moment to wake up
  writeByte(MPU9150_ADDRESS, MPU9150_PWR_MGMT_1, 0x00); delay(100); 
    wdt_reset(); 
  // Read the WHO_AM_I register for MPU-9150, this is a good test of communication
  uint8_t c = readByte(MPU9150_ADDRESS, MPU9150_WHO_AM_I); 
  if (c == 0x68) Serial.println(F("communication test with MPU6050 successful: MPU9150 is online..."));
  else{
    Serial.println(F("Communication test with MPU6050 FAILED"));
    while(1) ; // Loop forever if communication doesn't happen
  }
    wdt_reset(); 
  MPU6050SelfTest(); // Self test of MPU6050 to check if all values are within factory calibration --> Make sure this method is excecuted BEFORE initMPU9150()!
  calibrateMPU9150(); // Calibration of the MPU6050 chip for gyro & acc offsets --> Make sure this method is excecuted BEFORE initMPU9150()!
  //Initializing routine for MPU9150: Wakeup, Timesource, DLPF, Samplerate, Gyro & Acc range config
  initMPU9150(); //This sets the device in the desired state for normal operation

  //INIT Compass for direct access
  c = readByte(AK8975A_ADDRESS, WHO_AM_I_AK8975A); 
  if(c==0x48) Serial.println(F("Communication test with AK8975A Magnetometer successful"));
  else{
    Serial.println(F("Communication test with AK8975A Magnetometer FAILED !!!! "));
    while(1)  wdt_reset();  // Loop forever if communication doesn't happen
    }
  writeByte(AK8975A_ADDRESS, AK8975A_CNTL, 0x00); // Power down
    wdt_reset(); 
  delay(10);
    wdt_reset(); 
  initAK8975A(magCalibration);  //Initializing routine for AK8975A: Load x,y,z-axis sensitivity values 
  // Output the magnetic bias error which are calculated in a separate procedure - use only from time to time
  //magcalMPU9150(magBias);
  Serial.print(F("X-Axis offset/bias adjustment value in mG ")); Serial.println(magBias[0], 2);
  Serial.print(F("Y-Axis offset/bias adjustment value in mG ")); Serial.println(magBias[1], 2);
  Serial.print(F("Z-Axis offset/bias adjustment value in mG ")); Serial.println(magBias[2], 2);
  
  
  // setup Ports
  pinMode(LEDCollision1, OUTPUT);
  pinMode(LEDCollision2, OUTPUT);
  pinMode(LEDActive, OUTPUT);
  pinMode(LEDFault, OUTPUT);
  pinMode(LEDok, OUTPUT);
  pinMode(Bumper1OutPin, OUTPUT);
  pinMode(Bumper2OutPin, OUTPUT);
  digitalWrite(LEDCollision1,HIGH);
  digitalWrite(LEDCollision2,HIGH);
  digitalWrite(LEDActive,HIGH);
  digitalWrite(LEDFault,HIGH);
  digitalWrite(LEDok,HIGH);
  delay(2000);
  digitalWrite(LEDActive,LOW);
  digitalWrite(LEDFault,LOW);
  digitalWrite(LEDok,LOW);
  digitalWrite(LEDCollision1,LOW);
  digitalWrite(LEDCollision2,LOW);
  digitalWrite(Bumper1OutPin,LOW);
  digitalWrite(Bumper2OutPin,LOW);
  myPinUSLeft.setInput();
  myPinUSCenter.setInput();
  myPinUSRight.setInput();
  
  attachPCINT(digitalPinToPCINT(uSPinLeft), doUSLeftISR, CHANGE);
  attachPCINT(digitalPinToPCINT(uSPinCenter), doUSCenterISR, CHANGE);
  attachPCINT(digitalPinToPCINT(uSPinRight), doUSRightISR, CHANGE);
  wdt_reset(); 
  previousMillisReset = millis();
  previousMillisMpu = millis();
  lastUSTriggerTime = millis();
  previousMillis = millis();
}
//================================== END SETUP =======================================
//====================================================================================
//==================================== MAIN ==========================================
void loop() 
{
  // ----------------------------------------- mS Timer for measurement interval -----
  currentMillis = millis();
  if(currentMillis - previousMillisReset >= resetInterval) delay(10000); //trigger WDT to reset
  wdt_reset(); 
  
  if(currentMillis - previousMillis >= interval) 
  {
    previousMillis = currentMillis;
       
    flagAdc0Read = 1;
    flagAdc1Read = 1;
  }
  // --------------------------------------------------------------------------------
  if((currentMillis - previousMillisPrint >= printInterval)&& (flagAdc0Read == 0) && (flagAdc1Read == 0)){
    previousMillisPrint = currentMillis;
    printReadings();
  }
  
  if((currentMillis - previousMillisMpu >= mpuInterval) && (flagAdc0Read == 0) && (flagAdc1Read == 0)){
    previousMillisMpu = currentMillis;
    readMPU();
  }
  
  if ((currentMillis-lastUSTriggerTime >= USTriggerInterval) && (flagAdc0Read == 0) && (flagAdc1Read == 0)){      
    lastUSTriggerTime = currentMillis;
    readUS();
  }
  
  // ----------------------------------------------- active LED blink ---------------
  if(currentMillis - previousMillisLedActive >= intervalLedActive) 
  {
    previousMillisLedActive = currentMillis;   
    if (LEDActiveState == LOW)
      LEDActiveState = HIGH;
    else
      LEDActiveState = LOW;

    // set the LED with the ledState of the variable:
    digitalWrite(LEDActive, LEDActiveState);
  }
  // --------------------------------------------------------------------------------
  // ------------------------- Calculate Zero-Point ---------------------------------
  // ------------------------------- SENSOR 1 ---------------------------------------
  if(sensor1State < 1)
  {
    if(flagAdc0Read == 1 &&  count1FirstRead < 4)
    {    // read the analog in value: sensor1DiffFirst
      sensor1DiffFirst = sensor1DiffFirst + analogRead(sensor1InPin);
      count1FirstRead++;
      flagAdc0Read = 0;
    }
    if(count1FirstRead >= 3)
    {
      sensor1DiffFirst = sensor1DiffFirst / 4;
      sensor1State = 1;
    }
  }
  // --------------------------------------------------------------------------------
  // ------------------------- Calculate Zero-Point ---------------------------------
  // ------------------------------- SENSOR 2 ---------------------------------------
  if(sensor2State < 1)
  {
    if(flagAdc1Read == 1 &&  count2FirstRead < 4)
    {    // read the analog in value: sensor2DiffFirst
      sensor2DiffFirst = sensor2DiffFirst + analogRead(sensor2InPin);
      count2FirstRead++;
      flagAdc1Read = 0;
    }
    if(count2FirstRead >= 3)
    {
      sensor2DiffFirst = sensor2DiffFirst / 4;
      sensor2State = 1;
    }
  }
  
  // Hier kann durch eine Verzögerung die Empfindlichkeit erhöt werden.
  // Das geht jedoch auf Kosten der Reaktionsgeschwindigkeit
  // Achtung kein delay() verwenden sonst wird die Bearbeitung unterbrochen

  // --------------------------------------------------------------------------------
  // ------------------------- Calculate Touch-Point --------------------------------
  // ------------------------------- SENSOR 1 ---------------------------------------  
  if(sensor1State >= 1)
  {
    if(flagAdc0Read == 1 &&  count1SecondRead < 4)
    { // read the analog in value: sensor1DiffSecond
      sensor1DiffSecond = sensor1DiffSecond + analogRead(sensor1InPin);
      count1SecondRead++;
      flagAdc0Read = 0;
    }
    if(count1SecondRead >= 3)
    {
      sensor1DiffSecond = sensor1DiffSecond / 4;
      sensor1State = 2;
    }
  }
  // --------------------------------------------------------------------------------
  // ------------------------- Calculate Touch-Point --------------------------------
  // ------------------------------- SENSOR 2 ---------------------------------------
  if(sensor2State >= 1)
  {
    if(flagAdc1Read == 1 &&  count2SecondRead < 4)
    { // read the analog in value: sensor1DiffSecond
      sensor2DiffSecond = sensor2DiffSecond + analogRead(sensor2InPin);
      count2SecondRead++;
      flagAdc1Read = 0;
    }
    if(count2SecondRead >= 3)
    {
      sensor2DiffSecond = sensor2DiffSecond / 4;
      sensor2State = 2;
    }
  }
  
  // --------------------------------------------------------------------------------
  // ------------------------- Calculate Zero-Touch-Diff ----------------------------
  // ------------------------------- SENSOR 1 ---------------------------------------  
  if(sensor1State == 2)
  {
    sensor1Diff = sensor1DiffSecond - sensor1DiffFirst;
    sensor1DiffSecond = 0;
    count1SecondRead = 0;
    sensor1State = 3;
  }
  // --------------------------------------------------------------------------------
  // ------------------------- Calculate Zero-Touch-Diff ----------------------------
  // ------------------------------- SENSOR 2 ---------------------------------------
  if(sensor2State == 2)
  {
    sensor2Diff = sensor2DiffSecond - sensor2DiffFirst;
    sensor2DiffSecond = 0;
    count2SecondRead = 0;
    sensor2State = 3;
  }
  // --------------------------------------------------------------------------------
  // -------------------- Check Zero-Touch-Diff for Trigger -------------------------
  // ------------------------------- SENSOR 1 ---------------------------------------
  if(sensor1Diff >= sensor1Trigger & sensor1State == 3) 
  {
    //digitalWrite(LEDCollision1,HIGH);
    //digitalWrite(Bumper1OutPin,HIGH);
    trigger1Counter++;
    /*Serial.print("counter 1 = " );
    Serial.print(trigger1Counter);
    Serial.print("\t sensor dif 1 = ");
    Serial.println(sensor1Diff);*/
    bumperLeftTrigger = true;
    //Serial.println("BL1");
  } else if (sensor1State == 3) 
  {
    sensor1State = 0;
    sensor1DiffFirst = 0;
    sensor1DiffSecond = 0;
    count1SecondRead = 0;
    count1FirstRead = 0;
    //digitalWrite(LEDCollision1,LOW);
    //digitalWrite(Bumper1OutPin,LOW);
    bumperLeftTrigger = false;
    //Serial.println("BL0");
  }
  // --------------------------------------------------------------------------------
  // -------------------- Check Zero-Touch-Diff for Trigger -------------------------
  // ------------------------------- SENSOR 2 ---------------------------------------
  if(sensor2Diff >= sensor2Trigger & sensor2State == 3) 
  {
    //digitalWrite(LEDCollision2,HIGH);
    //digitalWrite(Bumper2OutPin,HIGH);
    trigger2Counter++;
    bumperRightTrigger = true;
    //Serial.println("BR1");
    /*
    Serial.print("counter 2 = " );
    Serial.print(trigger2Counter);
    Serial.print("\t sensor dif 2 = " );
    Serial.println(sensor2Diff);
    */
  } else if (sensor2State == 3) 
  {
    sensor2State = 0;
    sensor2DiffFirst = 0;
    sensor2DiffSecond = 0;
    count2SecondRead = 0;
    count2FirstRead = 0;
    //digitalWrite(LEDCollision2,LOW);
    //digitalWrite(Bumper2OutPin,LOW);
    bumperRightTrigger = false;
    //Serial.println("BR0");
  }
  
}// end void loop()
//==================================== END MAIN ======================================
//====================================================================================

void doUSLeftISR() {
  // look for a low-to-high on channel A
  if (USLeftTriggerActive == false){
    if (myPinUSLeft.getValue() == LOW) {
      lastPulseLengthUSLeft = pulseLengthUSLeft;
      pulseLengthUSLeft = micros() - pulseStartUSLeft;
      if (pulseLengthUSLeft < 150) pulseLengthUSLeft = 0;
      if (pulseLengthUSLeft > 25000) pulseLengthUSLeft = 0;
    }
    else   
    {
     pulseStartUSLeft = micros();
    }
  }
}

void doUSCenterISR() {
  // look for a low-to-high on channel A
  if (USCenterTriggerActive == false){
    if (myPinUSCenter.getValue() == LOW) {
      lastPulseLengthUSCenter = pulseLengthUSCenter;
      pulseLengthUSCenter = micros() - pulseStartUSCenter;
      if (pulseLengthUSCenter < 150) pulseLengthUSCenter = 0;
      if (pulseLengthUSCenter > 25000) pulseLengthUSCenter = 0;
    }
    else   
    {
     pulseStartUSCenter = micros();
    }
  }
}

void doUSRightISR() {
  // look for a low-to-high on channel A
  if (USRightTriggerActive == false){
    if (myPinUSRight.getValue() == LOW) {
      lastPulseLengthUSRight = pulseLengthUSRight;
      pulseLengthUSRight = micros() - pulseStartUSRight;
      if (pulseLengthUSRight < 150) pulseLengthUSRight = 0;
      if (pulseLengthUSRight > 25000) pulseLengthUSRight = 0;
    }
    else   
    {
     pulseStartUSRight = micros();
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void WatchDog_Setup(void)
{
  cli();                       // disable all interrupts
  wdt_reset();                 // reset the WDT timer

  // Enter Watchdog Configuration mode:
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  // Set Watchdog settings:
  WDTCSR = (1 << WDIE) | (1 << WDE) | (0 << WDP3) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0);
  sei();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
   WDIE - Enables Interrupts. This will give you the
   chance to include one last dying wish (or a few
   lines of code...) before the board is reset. This is a
   great way of performing interrupts on a regular
   interval should the watchdog be configured to not
   reset on time-out.

*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(WDT_vect) // Watchdog timer interrupt.
{
  // Chance to express a last dying wish for the program
  // Include your code here - be careful not to use functions they may cause the interrupt to hang and
  // prevent a reset.
  Serial.print("W");
  Serial.println("\t");
}

void readUS(){    
    switch(senSonarTurn) {
      case SEN_SONAR_RIGHT:
        USCenterDistance = (double)pulseLengthUSCenter*17.0/1000.0;
        if (USCenterDistance >= 2.5) sonarDistCenterMedian.add(USCenterDistance);
        if ((sonarDistCenterMedian.getCount() >= 3) && (USCenterDistance > 0)) USCenterDistance = sonarDistCenterMedian.getMedian();
        Serial.print("SC");
        Serial.print(USCenterDistance);
        Serial.println("\t");
        USRightTriggerActive = true;
        myPinUSRight.setOutput();
        myPinUSRight.setHigh();
        delayMicroseconds(10);
        myPinUSRight.setLow();
        myPinUSRight.setInput();
        USRightTriggerActive = false;
        senSonarTurn = SEN_SONAR_LEFT;
        break;
      
      case SEN_SONAR_LEFT:
        USRightDistance = (double)pulseLengthUSRight*17.0/1000.0;
        if (USRightDistance >= 2.5) sonarDistRightMedian.add(USRightDistance);
        if ((sonarDistRightMedian.getCount() >= 3) && (USRightDistance > 0)) USRightDistance = sonarDistRightMedian.getMedian();
        Serial.print("SR");
        Serial.print(USRightDistance);
        Serial.println("\t");
        USLeftTriggerActive = true;
        myPinUSLeft.setOutput();
        myPinUSLeft.setHigh();
        delayMicroseconds(10);
        myPinUSLeft.setLow();
        myPinUSLeft.setInput();
        USLeftTriggerActive = false;
        senSonarTurn = SEN_SONAR_CENTER;
        break;
      
      case SEN_SONAR_CENTER:
        USLeftDistance = (double)pulseLengthUSLeft*17.0/1000.0;
        if (USLeftDistance >= 2.5) sonarDistLeftMedian.add(USLeftDistance);
        if ((sonarDistLeftMedian.getCount() >= 3) && (USLeftDistance > 0)) USLeftDistance = sonarDistLeftMedian.getMedian();
        Serial.print("SL");
        Serial.print(USLeftDistance);
        Serial.println("\t");
        USCenterTriggerActive = true;
        myPinUSCenter.setOutput();
        myPinUSCenter.setHigh();
        delayMicroseconds(10);
        myPinUSCenter.setLow();
        myPinUSCenter.setInput();
        USCenterTriggerActive = false;
        senSonarTurn = SEN_SONAR_RIGHT;
        break;
      
      default:
        senSonarTurn = SEN_SONAR_RIGHT;
        break;
    }
}

void readMPU(){
  // Read all sensor values which the sensor provides and calculate phyisical values incl. corrections

  if (readMagData(compRaw)){
    readMagDatatoggled = false;
    readAccelData(accRaw);
    readGyroData(gyroRaw);
    readTempData(tempRaw); 
    
    temperature = tempRaw[0]/340.0;
    temperature += 35.0;
    // Low-Pass Filter the magnetic raw data in x, y, z since a filter later is not so easy to implement
    for(int j = 0 ;j<3;j++){
      compRaw_Filter[j] = a_CompFilter * compRaw_Filter[j] + (1-a_CompFilter) * compRaw[j];
    }
    
    for(int i = 0 ;i<3;i++){
      gyroDegPerSec[i] = gyroRaw[i] * gyroRes;
      accG[i] = accRaw[i] * accRes;
      compMicroT[i] = compRaw_Filter[i] * magCalibration[i] * compRes - magBias[i];
    }  
    
  
    //Safe all the main values for a better overview and for sign, direction and axis correction here
    //MPU6050 is the acc
    //The coordinate system of the Magnetometer will be used, hence PRINTED X,Y on sensor are MISLEADING --> Change x and y in mind!!!!
    ax = -accG[1];           // X-Axis is pointing FORWARD in flight direction (Aircraft nose pointing to earth = positive values) --> Invert and use y-axis of MPU6050
    ay = -accG[0];           // Y-Axis is pointing to the RIGHT side of the AC (right wing is pointing to earth = positive values) --> Invert and use x-axis of MPU6050
    az = accG[2];            // Z-Axis is pointing DOWNWARDS (z-axis pointing to earth = positive values) --> OK
    gx = gyroDegPerSec[1];   // Rotation around x-axis is ROLL and clockwise positive/a roll to the right is positive --> Use y-axis of MPU6050
    gy = gyroDegPerSec[0];   // Rotation around y-axis is PITCH and clockwise positive/nose up is positive --> Use x-axis of MPU6050
    gz = -gyroDegPerSec[2];  // Rotation around z-axis is YAW and clockwise positive/a turn to east is positive --> Invert for MPU6050
    mx = compMicroT[0];      // x-axis to north = positive values / to east/west = near 0 / to south = negative --> OK
    my = compMicroT[1];      // y-axis to north = positive values / to east/west = near 0 / to south = negative --> OK
    mz = compMicroT[2];      // if sensor is leveled = positive values -->OK
    //Serial.print(ax);Serial.print("\t");Serial.print(ay);Serial.print("\t");Serial.print(az);Serial.print("\t");Serial.print(gx);Serial.print("\t");Serial.print(gy);Serial.print("\t");Serial.print(gz);Serial.print("\t | ");Serial.print(mx);Serial.print("\t");Serial.print(my);Serial.print("\t");Serial.print(mz);Serial.print("\t");Serial.print(mx_h);Serial.print("\t");Serial.print(my_h);Serial.println();
    
    /*** Angle Calculation and Sensor Fusion ***/
    //Angle calculation from accelerometer. x-Axis pointing to front (ATTENTION: IS THE PRINTED Y-AXIS ON THE SENSOR!)
    pitchAcc = atan2(ax, az); // pitch is angle between x-axis and z-axis
    pitchAcc = -pitchAcc * RadToDeg;      // pitch is positiv upwards (climb) --> Invert the angle! 
    
    rollAcc = atan2(ay, az); // roll is angle between y-axis and z-axis
    rollAcc = rollAcc * RadToDeg;         // roll is clockwise positiv to the right (rotation clockwise in flight direction) 
    /*
    accPitch   = atan2(-acc.x , sqrt(sq(acc.y) + sq(acc.z)));         
    accRoll    = atan2(acc.y , acc.z);       
    accPitch = scalePIangles(accPitch, ypr.pitch);
    accRoll  = scalePIangles(accRoll, ypr.roll);
    */
    // Calculation of the heading
  
    // With tilt compensation
    // Takes raw magnetometer values, pitch and roll and turns it into a tilt-compensated heading value ranging from -pi to pi (everything in this function should be in radians).
    // Basically we first "unturn" the roll-angle and then the pitch to have mx and my in the horizontal plane again
    pitchRad = pitch/RadToDeg;  //angles have to be inverted again for unturing from actual plane to horizontal plane
    rollRad = roll/RadToDeg;
    /*
    mx_h = mx*cos(pitchRad) + my*sin(rollRad)*sin(pitchRad) - mz*cos(rollRad)*sin(pitchRad);
    my_h = my*cos(rollRad) + mz*sin(rollRad);
    */
    mx_h = mx * cos(pitchRad)  + mz * sin(pitchRad);
    my_h = mx * sin(rollRad)  * sin(pitchRad) + my * cos(rollRad) - mz * sin(rollRad) * cos(pitchRad);
    yawMag = wrap(mx_h, my_h);

    //yawMag = wrap(mx,my);  //without tilt compensation
    //Serial.print(mx);Serial.print("\t");Serial.print(my);Serial.print("\t");Serial.print(mx_h);Serial.print("\t");Serial.println(my_h);Serial.println();
  
    //yawMag = yawMag * RadToDeg;
    yawMag = yawMag - (declinationAngle/180*pi); // Subtracking the 'Declination Angle' in Deg --> a positive (to east) declination angle has to be subtracked
    //float inclinationAngle = atan(mz/sqrt(mx*mx+my*my))* RadToDeg;
    //Serial.print("Inclination Angle = ");Serial.println(inclinationAngle);
    
    
    //Sensor fusion
    now = micros();
    dT = (now - lastupdate)/1000000.0;
    lastupdate=micros();
    
    pitch = a_coeff * (pitch + gy * dT) + (1-a_coeff) * scale180(pitchAcc);  // pitch is the rotation around the y-axis  
    roll  = a_coeff * (roll  + gx * dT) + (1-a_coeff) * scale180(rollAcc);  // roll is the rotation around the x-axis
    // yaw is the rotation around the z-axis. ATTENTION: Simple filtering as for pitch and roll does not work here properly due to the change betwenn 0° - 360° and vice versa
    //       e.g "yaw = a_coeff * yaw + (1-a_coeff) * yawMag;" does not work
    //yaw = yawMag;
    yaw = scalePI(yaw);
    yawMag = scalePI(yawMag);
    yawMag = scalePIangles(yawMag, yaw);
    gzRad = gz * pi/180.0;  
    if ((yaw < -pi/2) || (yawMag < -pi/2)){
      turnPiPlus = true;
      yaw += pi/2;
      yawMag += pi/2;
    }
    else if ((yaw > pi/2)  ||  (yawMag > pi/2)){
      turnPiMinus = true;
      yaw -= pi/2;
      yawMag -= pi/2;
    }
    if (((yaw < pi/4) && (yaw > -pi/4))|| ((yawMag < pi/4) && (yawMag > -pi/4))){
      turnPiSmall = true;
      yaw -= pi/2;
      yawMag -= pi/2;
    }
    yaw = Complementary2(yawMag, -gzRad, (int)(dT*1000.0), yaw); //  -??gz
    if (turnPiPlus) {
      yaw -= pi/2;
      turnPiPlus = false;
    }
    if (turnPiMinus) {
      yaw += pi/2;
      turnPiMinus = false;
    }
    if (turnPiSmall) {
      yaw += pi/2;
      turnPiSmall = false;
    }

    //if(yaw>360) yaw = yaw-360; //care for the change from 0-->360 or 360-->0 near to north
    //if(yaw<0)yaw = yaw+360;
  
    yaw = scalePI(yaw);
    //pitch = pitch/RadToDeg;
    //roll = roll/RadToDeg;
    pitchMedian.add(scale180(pitch));
    rollMedian.add(scale180(roll));
    
  }
  if (readMagDatatoggled == false){
    readMagDatatoggled =true;
    toggleReadMagData();
  }
}

// first-order complementary filter
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis
float Complementary2(float newAngle, float newRate,int looptime, float angle) {
  float k=10;
  float dtc2=float(looptime)/1000.0;
  float x1 = (newAngle -   angle)*k*k;
  float y1 = dtc2*x1 + y1;
  float x2 = y1 + (newAngle -   angle)*2*k + newRate;
  angle = dtc2*x2 + angle;
  return angle;
}

// second-order complementary filter
// a=tau / (tau + loop time)
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis
float Complementary(float newAngle, float newRate,int looptime, float angle) {
  float tau=0.075;
  float a=0.0;
  float dtC = float(looptime)/1000.0;
  a=tau/(tau+dtC);
  angle= a* (angle + newRate * dtC) + (1-a) * (newAngle);
  return angle;
}

// Kalman filter                                      
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()
float Kalman(float newAngle, float newRate,int looptime, float x_angle)
{
  float Q_angle  =  0.01; //0.001
  float Q_gyro   =  0.0003;  //0.003
  float R_angle  =  0.01;  //0.03

  float x_bias = 0;
  float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
  float  y, S;
  float K_0, K_1;

  float dt = float(looptime)/1000;
  x_angle += dt * (newRate - x_bias);
  P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
  P_01 +=  - dt * P_11;
  P_10 +=  - dt * P_11;
  P_11 +=  + Q_gyro * dt;

  y = newAngle - x_angle;
  S = P_00 + R_angle;
  K_0 = P_00 / S;
  K_1 = P_10 / S;

  x_angle +=  K_0 * y;
  x_bias  +=  K_1 * y;
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;

  return x_angle;
}

// scale setangle, so that both PI angles have the same sign    
float scalePIangles(float setAngle, float currAngle){
  if ((setAngle >= PI/2) && (currAngle <= -PI/2)) return (setAngle-2*PI);
    else if ((setAngle <= -PI/2) && (currAngle >= PI/2)) return (setAngle+2*PI);
    else return setAngle;
}

// rescale to -PI..+PI
float scalePI(float v)
{
  float d = v;
  while (d < 0) d+=2*PI;
  while (d >= 2*PI) d-=2*PI;
  if (d >= PI) return (-2*PI+d); 
  else if (d < -PI) return (2*PI+d);
  else return d;  
}

// rescale to -180..+180
float scale180(float v)
{
  float d = v;
  while (d < 0) d+=2*180;
  while (d >= 2*180) d-=2*180;
  if (d >= 180) return (-2*180+d); 
  else if (d < -180) return (2*180+d);
  else return d;  
}

void printReadings() {
  static byte printSwitch = 0;
  switch (printSwitch){
    case 0:
      Serial.print("P");
      Serial.print(pitchMedian.getAverage(), 3);
      Serial.print("\t");
      Serial.print("Y");
      Serial.print(yaw, 3);
      Serial.println("\t");
      printSwitch = 1;
      break;
    case 1:
      Serial.print("GX");
      Serial.print(gx, 3);
      Serial.print("\t");
      Serial.print("GY");
      Serial.print(gy, 3);
      Serial.print("\t");
      Serial.print("GZ");
      Serial.print(gz, 3);
      Serial.println("\t");
      printSwitch = 2;
      break;
    case 2:
      Serial.print("AX");
      Serial.print(ax, 3);
      Serial.print("\t");
      Serial.print("AY");
      Serial.print(ay, 3);
      Serial.print("\t");
      Serial.print("AZ");
      Serial.print(az, 3);
      Serial.println("\t");
      printSwitch = 3;
      break;
    case 3:
      Serial.print("R");
      Serial.print(rollMedian.getAverage(), 3);
      Serial.print("\t");
      Serial.print("Y");
      Serial.print(yaw, 3);
      Serial.println("\t");
      
      printSwitch = 4;
      break;
    case 4:
      Serial.print("MX");
      Serial.print(mx, 3);
      Serial.print("\t");
      Serial.print("MY");
      Serial.print(my, 3);
      Serial.print("\t");
      Serial.print("MZ");
      Serial.print(mz, 3);
      Serial.println("\t");
      printSwitch = 5;
      break;
    case 5:
      Serial.print("T");
      Serial.print(temperature, 3);
      Serial.print("\t");
      Serial.print("DT");
      Serial.print(dT, 3);
      Serial.println("\t");
      printSwitch = 6;
      break;
   case 6:
      Serial.print("BL");
      if (bumperLeftTrigger)Serial.print("1");
      else Serial.print("0");
      Serial.print("\t");
      Serial.print("BR");
      if (bumperRightTrigger)Serial.print("1");
      else Serial.print("0");
      Serial.println("\t");
      printSwitch = 0;
      break;
    default:
      printSwitch = 0;
      break;
  }
    if (lastBumperLeftTrigger != bumperLeftTrigger){
      lastBumperLeftTrigger = bumperLeftTrigger;
      Serial.print("BL");
      if (bumperLeftTrigger)Serial.print("1");
      else Serial.print("0");
      Serial.println("\t");
    }
    if (lastBumperRightTrigger != bumperRightTrigger){
      lastBumperRightTrigger = bumperRightTrigger;
      Serial.print("BR");
      if (bumperRightTrigger)Serial.print("1");
      else Serial.print("0");
      Serial.println("\t");
    }
}
  
