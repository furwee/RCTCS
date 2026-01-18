/*
PCB AND CODING done by furwee

SERVO AND ESC USE uS (microseconds) as unit
*/

// modules to include
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
float IMU_OFFSET_X = -3.85;  // acx, + is forward, - is backward
float IMU_OFFSET_Y = -28.8;  // acy, + is right and - is left (acceleration)
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
float counterSteerStrength = 1.5;
float alpha = 0.005;
float tolerance = 0.2;

void setup() {
    while (!IMU.begin_I2C()){
    delay(5000);
  }
  // IMU setup
  IMU.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
  IMU.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);

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
  TXLED0;
  // gyro event
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  IMU.getEvent(&accel, &gyro, &temp);

  //offset to get calibrated data
  acX = accel.acceleration.x + IMU_OFFSET_X;
  acY = accel.acceleration.y + IMU_OFFSET_Y;
  gyZ = gyro.gyro.z + IMU_OFFSET_Z; // refer to ln 25-27

  // reading input from jetson nano
  int servo = constrain(servoIN.getValue(), 1000, 2000);
  int ESC = constrain(ESCIN.getValue(), 1500, 2000);

  //algorithm(s)

  // generate expected yaw rate
  float accXY = sqrt(acX*acX + acY*acY); // get resultant velocity

  // sample velocity
  if (millis() - lastSampleTime > 10) {
    vel += accXY * 0.01; // v = at
    lastSampleTime = millis();

    if (ESC == 1500) { // center point
      vel = 0;
    }
  }

  // clock led
  digitalWrite(RXLED, millis()%500>250); // https://www.reddit.com/r/arduino/comments/n1suiz/led_blink_no_delay_one_line_of_code/ <- VERY USEFUL TO TELL IF ITS WORKING

  // converted from https://www.desmos.com/calculator/n14dt2nzrk
  float kvel = 1/(1- alpha * vel); // ln 53
  float expectedYawRate = floor(constrain(500 - 490 * cos(3.14f * (servo - 1500) * 0.002f * kvel), 0, 1000)/25)*25;
  
  //direction
  int sign = (servo < 1450) ? -1 : (servo > 1550) ? 1 : 0; // set positive or negative or a 0 if deadzone (100uS)
  relativeServoCorrectionSteps = sign * servoCorrectionSteps;

  //correction
  float diff = gyZ - expectedYawRate;
  if (diff < -tolerance) { // refer to ln 54
    //understeer, reduce throttle and add steering
    servo += relativeServoCorrectionSteps; 
    ESC -= throttleCorrectionSteps; // refer from ln 49
  } else if (diff > tolerance) {
    // oversteer, counter-steer and maintain throttle until it is normal again
    servo -= counterSteerStrength * relativeServoCorrectionSteps;
    ESC -= throttleCorrectionSteps * 0.2f;
  }

  // output
  servoOUT.writeMicroseconds(servo);
  ESCOUT.writeMicroseconds(ESC);  
}
