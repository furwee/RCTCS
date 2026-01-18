# RCTCS
## What is this project about?
this project started as a assist of jetracer's operation loop (https://github.com/NVIDIA-AI-IOT/jetracer) for competition but slowly outgrew its original purpose to also become a Traction Control System (TCS). Hence the name.  

## What do i need to prepare?
- a SparkFun Pro Micro
- Adafruit LSM6DS3TR-C + LIS3MDL - Precision 9 DoF IMU (5543)
- Breadboard w/ wires and a button
- alternatively, get yourself a PCB w/ soldering equipment and a 5x5x1.2mm smd button of your own choice
- USB micro USB cable (make sure it is not just a power cable)  

## What is the control loop?
1: read the IMU's data  
2: read PWM from the outputs of jetracer (steering and throttle)  
3: generate a resultant acceleration from acX and acY  
4: using v = a*t, get velocity via integration (resets to 0 if ESC is 1000uS)  
5: using [this link](https://www.desmos.com/calculator/n14dt2nzrk), generates an expected yaw rate from vel (velocity) and alpha (a), adjustable if needed.  
6: confirms steering direction and adding a 100uS deadzone on 1500uS (center point of servo pwm)  
7: using yaw rate's difference to the expected yaw rate from step 5 to do correction (tolerance is 0.2G adjustable)  
8: output the processed pwm

## What is included in this repository?
- the arduino code (duh)
- the schematic and the gerber files of the pcb

## What are the dependencies of this code?
- Adafruit_LSM6DS33.h
- Wire.h
- [PWM.hpp](https://github.com/xkam1x/Arduino-PWM-Reader)
- Servo.h
