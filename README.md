# RCTCS
## What is this project about?
this project started as a assist of jetracer's operation loop (https://github.com/NVIDIA-AI-IOT/jetracer) for competition but slowly outgrew its original purpose to also become a Traction Control System (TCS). Hence the name.  
![pcb product](https://github.com/furwee/RCTCS/blob/master/asset/20260118_124929.jpg)  

---

## What do i need to prepare?
- a SparkFun Pro Micro
- arduino IDE that can code the pro micro
- Adafruit LSM6DS3TR-C + LIS3MDL - Precision 9 DoF IMU (5543)
- Breadboard w/ wires and a button
- alternatively, get yourself a PCB w/ soldering equipment and a 5x5x1.2mm smd button of your own choice (for quick reset)
- USB micro USB cable (make sure it is not just a power cable)
- 4 m2.5*8mm screw with individual nuts if you are going for pcb
- a couple pin headers
  
---

## (optional) How to setup your arduino IDE so it can upload codes to the pro micro (ignore if you are using another micro-controller):
The SparkFun Pro Micro (ATmega32U4) is not natively listed in the Arduino IDE.  
Follow these steps to set it up:

1. **Install Board Definitions**
   - Open Arduino IDE → *File* → *Preferences*.
   - Add SparkFun’s JSON URL to *Additional Board Manager URLs*:  
     `https://raw.githubusercontent.com/sparkfun/Arduino_Boards/master/IDE_Board_Manager/package_sparkfun_index.json`
   - Go to *Tools* → *Board* → *Boards Manager*.
   - Search for **SparkFun AVR Boards** and install.

2. **Select the Correct Board**
   - *Tools* → *Board* → choose **SparkFun Pro Micro**.
   - *Tools* → *Processor* → select **ATmega32U4 (5V, 16MHz)** or **ATmega32U4 (3.3V, 8MHz)** depending on your hardware.
   - 
---

## How to set up your IMU (a very primitive way):
A perfect IMU is impossible to make and is totally unique. However, we can mitigate the errors with offsets and filtering  
I already rounded the datas to be simplified  
you only need to: 
  - add your own code to RCTCS_v1-x.ino to have ```Serial``` in setup()
  - print the **raw** data from sensor.getEvent()
  - adjust offset until 0 is reached

viola, there you go :)

---

## What is the control loop?
1: read the IMU's data and get a processed acX, acY and gyZ  
2: read PWM from the outputs of jetracer (steering and throttle)  
3: generate a resultant acceleration from acX and acY  
4: using v = a*t, get velocity via integration (resets to 0 if ESC is 1450uS)  
5: using [this link](https://www.desmos.com/calculator/n14dt2nzrk), generates an expected yaw rate from vel (velocity) and alpha (a), adjustable if needed.  
6: confirms steering direction and adding a 100uS deadzone on 1500uS (center point of servo pwm)  
7: using yaw rate's difference to the expected yaw rate from step 5 to do correction (tolerance is 0.2G adjustable)  
8: output the processed pwm

Flowchart coming later, sorry for the wall of text XD

---

## What is included in this repository?
- the arduino code (duh)
- the schematic of the [pcb](https://github.com/furwee/RCTCS/blob/master/asset/SCH_hidden_gyro_1-hidden_gyro_2026-01-18.svg) (unfortunately i dont feel comfortable sharing my gerber files but you may contact me to get it)

---

## What are the dependencies of this code?
- Adafruit_LSM6DS33.h
- Wire.h
- [PWM.hpp](https://github.com/xkam1x/Arduino-PWM-Reader)
- Servo.h
