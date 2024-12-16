# Wi-Fi Controlled Robot With Camera

## Introduction

  The project is a spider with 4 legs controlled through Wi-Fi. It is equiped with a camera that will transmit the images it records to the app that controls it.

## Description

  The spider will use an ESP32 microcontroller for its access to Wi-Fi. The controller will be connected through I2C to a Servo Motor Driver and a Camera Module with a FIFO buffer. The robot will be controlled through an app to which it will transmit the images the camera records. For its power source, 2 18500 Li-ion Batteries will be used. The batteries together supply 7.4V and 4800mAh. As 7.4V is to high for the Servo Driver, a Voltage Step Down Module will be used to reduce it to 5V. A switch is placed between the Batteries and the Voltage Step Down Module to turn off the robot when not in use.
  Each of its legs will have 3 Servo Motors: one at the base rotating on the Z axis, and two rotating on the X axis.
  The body and the legs of the robot are 3D printed.

## Hardware Design

### Block Diagram

![screenshot](Media/ShcemaBlocV2.png)

### Electric Diagram

![screenshot](Media/ShcemaElectricaV1.png)

### Components

| Component | Count | Place of Aquisition | Datasheet |
|-----------|-------|---------------------|---------- |
| ESP32 Development Board | 1 | Optiums Digital | https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf |
| PCA9685 Servo Driver | 1 | Optimus Digital | https://cdn-shop.adafruit.com/datasheets/PCA9685.pdf |
| SG90 Servo Motor | 12 | Optimus Digital | http://www.ee.ic.ac.uk/pcheung/teaching/DE1_EE/stores/sg90_datasheet.pdf |
| OV7670 + AL422B FIFO Camera Module | 1 | Optimus Digital | https://www.haoyuelectronics.com/Attachment/OV7670%20+%20AL422B%28FIFO%29%20Camera%20Module%28V2.0%29/OV7670%20Implementation%20Guide%20%28V1.0%29.pdf|
| Voltage Step Down Module (8A; In: 4-40V, Out: 1.25-36V) | 1 | Optimus Digital | - |
| Battery Support 2x 18500 | 1 | Optimus Digital | - |
| 18500 Li-Ion Battery 3.7V 4800mAh | 2 | - | - |
| 4.7K Resistor | 2 | - | - |
| Wires | (have yet to count them) | Optimus Digital | - |

### 3D Model

![screenshot](Media/Model3DV1.png)

## Software Design

## Results

## Conclusions

## Resources
