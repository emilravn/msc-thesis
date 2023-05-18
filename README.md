# Table of Contents

- [Table of Contents](#table-of-contents)
- [MSc. Thesis](#msc-thesis)
- [Hardware](#hardware)
  - [Bill of Materials (BOM)](#bill-of-materials-bom)
  - [Project Wiring and Assembly](#project-wiring-and-assembly)
    - [Photos of the robot assembly](#photos-of-the-robot-assembly)
- [Development and Installation](#development-and-installation)
  - [Prerequisites](#prerequisites)
  - [Raspberry Pi 4 Setup](#raspberry-pi-4-setup)
  - [Workflow](#workflow)
    - [Monitor CPU/memory/temperature generate statistic reports](#monitor-cpumemorytemperature-generate-statistic-reports)
  - [Documentation from used libraries](#documentation-from-used-libraries)
- [Known issues](#known-issues)

# MSc. Thesis

<!-- ABOUT THE PROJECT -->

[![ROS 2 CI](https://github.com/emilravn/msc-thesis/actions/workflows/ros.yaml/badge.svg)](https://github.com/emilravn/msc-thesis/actions/workflows/ros.yaml)

**(Work in Progress)**

# Hardware

## Bill of Materials (BOM)

The following components were used for the robot assembly:

| Item No. | Part Name                                                                                  | Quantity | Unit Price | Total Price   |
| -------- | ------------------------------------------------------------------------------------------ | -------- | ---------- | ------------- |
| 1        | [Raspberry Pi 4 (4 GB)](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/)      | 1        | DKK529.00 | DKK529.00     |
| 2        | SanDisk SD Card (32 GB)                                                                    | 1        | DKK99.00  | DKK99.00      |
| 3        | [Two-wheel drive with tracks robot chassis kit (with DC motors and wheel encoders)](https://www.amazon.com/SZDoit-Absorption-Suspension-Raspberry-Education/dp/B096DHBTX3?th=1) | 1 | DKK674.61 | DKK674.61 |
| 4        | [L298N Motor Driver](https://elektronik-lavpris.dk/p132566/oky3195-1-l298n-stepper-motor-driver-controller-board/) | 1 | DKK149.00 | DKK149.00 |
| 5        | [HC-SR04 Ultrasonic Distance Sensor](https://let-elektronik.dk/ultrasonic-distance-sensor-hc-sr04) | 3 | DKK48.75 | DKK146.25 |
| 6        | [Raspberry Pi Camera Board v1.3 (5MP, 1080p)](https://elektronik-lavpris.dk/p145103/rpi-camera-board-raspberry-pi-kamera-modul-5mpix-v13/) | 1 | DKK99.00 | DKK99.00 |
| 7        | [SCD30 (CO<sub>2</sub>, Humidity and Temperature Sensor)](https://let-elektronik.dk/co-humidity-and-temperature-sensor-scd30) | 1 | DKK746.46 | DKK746.25 |
| 8        | [Zeee 2S Lipo Battery 5200 mAh 5200 mAh 7.4 V](https://www.amazon.de/-/en/Battery-80C-Batteries-Evader-Truggy/dp/B094Q9R1L4/ref=sr_1_16?crid=15SPCMBY8J8WU&keywords=lipo+7.4v+5200&qid=1683796021&sprefix=lipo+7.4v+5200%2Caps%2C77&sr=8-16)   | 1         | DKK172.00  | DKK172.00     |
| 9        | Powerbank for Raspberry Pi 4 (minimum output: 5V 2.4A)      | 1         | N/A  | N/A     |
| 10       | Capacitor Electrolytic 10uF 25V   | 2         | N/A  | N/A     |
| 11       | Resistor 330Ω   | 3         | N/A  | N/A     |
| 12       | Resistor 470Ω   | 3         | N/A  | N/A     |
| 13       | 5V Voltage Divider     | 1         | N/A  | N/A     |
| 14       | 3D printed mount stands for ultrasonic sensors     | 3         | N/A  | N/A     |
| 15       | 3D printed mount stand for camera     | 1         | N/A  | N/A    |
| 16       | Breadboard                                | 1         | N/A  | N/A    |
| 17       | Mounting screws                                | N/A         | N/A  | N/A    |
| 18       | Spare wires                                | N/A         | N/A  | N/A    |
| 19       | Female to Female jumper cables      | N/A         | N/A  | N/A    |
|          | **Total**                                  |          |            | **DKK2.616 + N/A** |

Additional tools or parts used in the project:

|     | Tool/Part |
| ----| --------- |
| 1   | 3D Printer |
| 2   | Tape |
| 1   | Screwdriver set |
| 1   | Double-sided tape |


## Project Wiring and Assembly

The electronic components and the complete circuit of the robot were connected as shown below in:

<p align="left">
  <img title='Electronic circuit' src=docs/img/electronic-circuit-schematic.png width="800">
</p>

The HC-SR04 ultrasonic sensors were connected to the Raspberry Pi 4 GPIO pins as indicated below, and each of them were connected between one 330Ω and one 470Ω resistor:

| HC-SR04 (Front) | GPIO.BOARD | GPIO.BCM |
| --------- | ---------- | -------- |
| VCC | 5V | 5V |
| GND | GND | GND |
| TRIG | 8 | GPIO 14 |
| ECHO | 10 | GPIO 15 |

| HC-SR04 (Middle) | GPIO.BOARD | GPIO.BCM |
| --------- | ---------- | -------- |
| VCC | 5V | 5V |
| GND | GND | GND |
| TRIG | 11 | GPIO 17 |
| ECHO | 13 | GPIO 27 |

| HC-SR04 (Rear) | GPIO.BOARD | GPIO.BCM |
| --------- | ---------- | -------- |
| VCC | 5V | 5V |
| GND | GND | GND |
| TRIG | 24 | GPIO 8 |
| ECHO | 26 | GPIO 7 |

The L298N motor driver pins were connected to the Raspberry Pi 4 as follows:

| L298N Motor Driver | GPIO.BOARD | GPIO.BCM |
| --------- | ---------- | -------- |
| VCC | 6.4V - 8.4V | 6.4 - 8.4V |
| GND | GND | GND |
| INA & INB | 16 & 18  | GPIO 24 & GPIO 23 |
| INC & IND | 29 & 31 | GPIO 5 & GPIO 6 |
| ENA | 32 | GPIO 12 PWM0 |
| ENB | 33 | GPIO 13 PWM1 |

The left and right encoder from the DC motors were connected to the Raspberry Pi 4 as follows:

| Encoder (Left) | GPIO.BOARD | GPIO.BCM |
| --------- | ---------- | -------- |
| Output A | 37 | GPIO 26 |
| Output B | 36 | GPIO 16 |

| Encoder (Right) | GPIO.BOARD | GPIO.BCM |
| --------- | ---------- | -------- |
| Output A | 22 | GPIO 25 |
| Output B | 15 | GPIO 22 |

The SCD30 sensor was connected to the Raspberry Pi 4 as follows:

| SCD30 | GPIO.BOARD | GPIO.BCM |
| --------- | ---------- | -------- |
| VIN | 3.3V | 3.3V |
| GND | GND | GND |
| TX/SCL | 5 | GPIO 3 SCL |
| RX/SDA | 3 | GPIO 2 SDA |
| SEL | GND | GND |

The DC motors are connected to the L298N motor driver as follows:

| DC motor (right) | L298N Motor Driver |
| --------- | ---------- |
| Positive | OUT A |
| Negative | OUT B |

| DC motor (left) | L298N Motor Driver |
| --------- | ---------- |
| Positive | OUT C |
| Negative | OUT D |

For reference, overview of the GPIO pins for the Raspberry Pi 4:

<p align="left">
  <a href="https://cdn.sparkfun.com/assets/learn_tutorials/1/5/9/5/GPIO.png"><img title="GPIO pins reference" src="docs/img/pin-reference.png" width="800"></a>
</p>

### Photos of the robot assembly

Front view of the robot:

<p align="left">
  <img title='Front view of the robot' src=docs/img/front_side.jpeg width="800">
</p>

Right view of the robot:

<p align="left">
  <img title='Right view of the robot' src=docs/img/right_side.jpeg width="800">
</p>

Back view of the robot:

<p align="left">
  <img title='Back view of the robot' src=docs/img/back_side.jpeg width="800">
</p>

Left view of the robot:

<p align="left">
  <img title='Left view of the robot' src=docs/img/left_side.jpeg width="800">
</p>

Top view of the robot:

<p align="left">
  <img title='Top view of the robot' src=docs/img/top.png width="800">
</p>

# Development and Installation

This section will walk you through setting up the development environment for ROS 2 using Visual Studio Code. The environment is based on the [ROS2 workspace template](https://github.com/athackst/vscode_ros2_workspace) by [athackst](https://github.com/athackst) with some minor tweaks to remove C++ as the robot is fully written in Python. Additionally, some of the settings `.devcontainer` configuration file has been altered due to recent updates to Visual Studio Code.

## Prerequisites

The following are necessary to setup the development environment:

* [Docker](https://docs.docker.com/engine/install/)
* [Visual Studio Code](https://code.visualstudio.com/)
* [Dev Containers Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

## Raspberry Pi 4 Setup

The Raspberry Pi 4 is based on the [Raspberry Pi with ROS 2 and the real-time kernel](https://github.com/ros-realtime/ros-realtime-rpi4-image) image and modified during setup with the Raspberry Pi Imager software to have different a hostname and password, and to connect it to Wi-Fi and allow SSH connections. As of May 2023, the image is based off the [ROS 2 Humble distribution](https://docs.ros.org/en/humble/index.html#) with Ubuntu 22.04.01 and the real-time kernel (PREEMPT_RT) pre-installed.

The credentials during the thesis for the robot:

| Hostname     | Username    | Password             | Access Point  | Access Point (Password) |
| ------------ | ----------- | ------------------   | ----------    | ----------------------  |
| `sfr.local`  | `sfr.local` | `sFRsuperduper@.`    | `SFR`         | `sFRsuperduper@`        |

* To enable SSH connection via hostname you should install **avahi-daemon** on the Raspberry Pi 4. 

* Disable SPI with `raspi-config` as we are not interfacing with a microcontroller. Otherwise none of the functionality won't work!

## Workflow

The [robot_control.sh](https://github.com/emilravn/msc-thesis/blob/206302bcd523076b146bc1268338234476f9b547/robot_control.sh) script is used to transferring code and to communicate with the robot at ease.
  * This becomes much simpler by copying your public SSH key to the Raspberry Pi 4 with `ssh-copy-id -i </path/to/key sfr@sfr.local`. 

If on a Windows machine you should also install the **Bonjour Print Service** from Apple which enables you to discover it by hostname instead of IP-address only.

Backup of the entire SD card is done periodically, and mostly when changes are made to the packages or the settings of the Raspberry Pi 4:
  * **Last backup date**: **16/05/2023**.

### Monitor CPU/memory/temperature generate statistic reports

Install **RPi-Monitor** with `apt` and it will serve up an interactive webpage at `localhost:8888` when on the same network as the Raspberry Pi 4.

## Documentation from used libraries

* [gpiozero](https://gpiozero.readthedocs.io/en/stable/index.html): library for controlling various GPIO elements.