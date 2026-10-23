

# 3D-Printed Quadruped Robot

A personal robotics project built with 3D-printed parts, an Arduino Nano, and 12 servomotors controlled through a PCA9685 PWM driver.

The goal is to explore robotics, embedded electronics, and servo control.

## Overview

![Quadruped robot](images/Robot-front.jpeg)

The robot features four legs with three servomotors each.

**Hardware:**
- Arduino Nano
- PCA9685 16-channel PWM servo driver
- 4 × MG90 and 8 × SG90 servomotors
- 3D-printed mechanical structure

## Electronics

![KiCad electrical schematic](images/wiring-diagram.svg)

The Arduino communicates with the PCA9685 via I²C (A4/SDA, A5/SCL). The servos use PWM channels 0–11 and require a separate power supply.

## My Contributions

- Assembled the 3D-printed robot and installed the servomotors.
- Selected and integrated the PCA9685 driver.
- Adapted the Arduino code for the PCA9685 with AI assistance.
- Created the electrical schematic using KiCad.

##Current Status

The robot is mechanically assembled, and the Arduino firmware has been tested in Wokwi using a simulated PCA9685 and 12 servomotors.

Servo commands and movements have been observed in simulation, but physical calibration and walking functionality remain untested.

Next steps include testing with a suitable external power supply, calibrating the servos, and validating the walking gait.

## Credits

Original design by Regis Hsu, including the mechanical parts, servo configuration, and original Arduino code.
(https://www.instructables.com/DIY-Spider-RobotQuad-robot-Quadruped/).

*Personal educational project — work in progress.*
