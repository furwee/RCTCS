/*
Copyright (C) 2026  Jaco Yeung
RC car traction control system using arduino

SERVO AND ESC USE uS (microseconds) as unit
accelerometer ouput = ms^-2
gyro output = rads^-1
*/

// modules to include
#include <avr/pgmspace.h>
#include <Adafruit_LSM6DS33.h>  // IMU
#include <Wire.h>               // IIC
#include "PWM.hpp"              // INPUT https://github.com/xkam1x/Arduino-PWM-Reader
#include <Servo.h>              //OUTPUT

// hard-coded, DO NOT CHANGE
#define ThrottleInPin 7
#define ServoInPin 8

#define ThrottleOutPin 9
#define ServoOutPin 5

// LED pin
int RXLED = 17; //RX led pin
bool toggle = LOW;

// IMU object
Adafruit_LSM6DS33 IMU;

// gyro offset, set 0 if calibrate at start, hardcode if stable
float IMU_OFFSET_X = -3.6;  // acx, + is forward, - is backward
float IMU_OFFSET_Y = -29;  // acy, + is right and - is left (acceleration)
float IMU_OFFSET_Z = 0;  // gyz, + is left and - is right (rotation)

// IMU calibrated datas
float acX, acY, gyZ = 0;

// setup pins for input
PWM servoIN(ServoInPin);
PWM ESCIN(ThrottleInPin);

// setup pins for output
Servo servoOUT;
Servo ESCOUT;

// millis()
unsigned long lastSampleTime = millis(); // accel to velocity usage

float vel; // velocity

// variable setting
int servoCorrectionSteps = 50; // value between 1-500
int relativeServoCorrectionSteps;
int throttleCorrectionSteps = 100; // value between 1-500

static float accFilter = 0.0f; // LPF accXY

// 0: counterSteerStrength, 1: alpha, 2: tolerance, 3: velThreshold, 4: filterAlpha
const PROGMEM float variablesFLOAT[5] = {1.5f, 0.005f, 0.2f, 6.0f, 0.15};

void setup() {
  // Serial.begin(9600);
  while (!IMU.begin_I2C()){
    delay(5000);
  }

  Wire.setClock(400000); // so fast :O

  // IMU setup
  IMU.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
  IMU.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
  IMU.setAccelDataRate(LSM6DS_RATE_3_33K_HZ);
  IMU.setGyroDataRate(LSM6DS_RATE_3_33K_HZ);


  /*pin section*/
  pinMode(RXLED, OUTPUT);  // Set RX LED as an output
  pinMode(ServoInPin, INPUT_PULLUP);
  pinMode(ThrottleInPin, INPUT_PULLUP);
  servoIN.begin(true);
  ESCIN.begin(true);

  /*output section*/
  servoOUT.attach(ServoOutPin);
  ESCOUT.attach(ThrottleOutPin);
}

void loop() {
  
  TXLED0; // just shutting the led up, dont mind it

  // gyro event
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  IMU.getEvent(&accel, &gyro, &temp);

  //offset to get calibrated data, rounded off to filter noise from 3.33k data rate
  acX = round((accel.acceleration.x + IMU_OFFSET_X) *0.1)*10;
  acY = round((accel.acceleration.y + IMU_OFFSET_Y) *0.1)*10;
  gyZ = gyro.gyro.z + IMU_OFFSET_Z; // refer to ln 25-27
  /* Serial.println(acX);
  Serial.println(acY);
  Serial.println(gyZ);
  Serial.println(); */

  // reading input from jetson nano
  int servo = constrain(servoIN.getValue(), 1000, 2000);
  int ESC = constrain(ESCIN.getValue(), 1450, 2000);

  //algorithm(s)

  // generate expected yaw rate
  float accXY = sqrt(acX*acX + acY*acY); // get resultant velocity
  accFilter = pgm_read_float(&variablesFLOAT[4])  * accXY + (1.0f - pgm_read_float(&variablesFLOAT[4])) * accFilter;

  // sample velocity
  if (millis() - lastSampleTime > 10) {
    vel += accFilter * 0.01; // v = at
    lastSampleTime = millis();
  }

  //reset velocity
  if (ESC < 1500) {
    vel = 0.0f;
  }

  // clock led
  // digitalWrite(RXLED, millis()%500>10); // https://www.reddit.com/r/arduino/comments/n1suiz/led_blink_no_delay_one_line_of_code/ <- VERY USEFUL TO TELL IF ITS WORKING

  // converted from https://www.desmos.com/calculator/n14dt2nzrk
  float kvel = 1/(1- pgm_read_float(&variablesFLOAT[1]) * vel); // ln 53
  float expectedYawRate = floor(constrain(500 - 490 * cos(0.00628 * (servo - 1500) * kvel), 0, 1000)* 0.04f)*25;
  
  //direction
  int sign = (servo < 1450) ? -1 : (servo > 1550) ? 1 : 0; // set positive or negative or a 0 if deadzone (100uS)
  relativeServoCorrectionSteps = sign * servoCorrectionSteps;

  //correction
  float diff = gyZ - expectedYawRate;
  if (diff < -1 * pgm_read_float(&variablesFLOAT[2])) { // refer to ln 54
    //understeer, reduce throttle and add steering
    servo += relativeServoCorrectionSteps; 

    ESC = (vel >= pgm_read_float(&variablesFLOAT[3])) ? (ESC - throttleCorrectionSteps) : (ESC + throttleCorrectionSteps); // refer to ln 51, handles both high speed understeer olr low speed understeer
  
  } else if (diff > pgm_read_float(&variablesFLOAT[2])) {
    // oversteer, counter-steer and maintain throttle until it is normal again
    servo -= pgm_read_float(&variablesFLOAT[0]) * relativeServoCorrectionSteps;
    ESC -= throttleCorrectionSteps * 0.2f;
  }

  // output
  servoOUT.writeMicroseconds(constrain(servo, 1000, 2000));
  ESCOUT.writeMicroseconds(constrain(ESC, 1450, 2000)); 
}
