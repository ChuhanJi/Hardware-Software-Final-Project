
# Adaptive Ambient Fill Light with Physical Gauge

## Project Overview

This project is a two-device system that monitors ambient lighting conditions and provides automatic fill lighting when illumination is insufficient.
A sensing device measures ambient light levels, while a display device visualizes light sufficiency using a physical gauge and activates a fill light when needed.

![image alt](https://github.com/ChuhanJi/Hardware-Software-Final-Project/blob/b74c9f004e2d23e30adfb9c82967dba2a666b64c/Images/A4%20-%201.jpg)

**System Components:**

* Sensing Device: Ambient light measurement and wireless transmission
* Display Device: Physical gauge visualization and fill light control

---

## Sensor Device — Ambient Light Sensing Unit

### Description

The sensor device continuously measures ambient light intensity and evaluates whether the surrounding environment provides sufficient illumination.
Processed light data is transmitted wirelessly to the display device for visualization and control.

### Sensor Used

* **Ambient Light Sensor: BH1750**
  The BH1750 is a digital ambient light sensor that measures illuminance in lux, allowing the system to detect whether the surrounding environment provides sufficient lighting.

### How It Works

The BH1750 ambient light sensor measures real-time illuminance values, which are read by an ESP32-C3 microcontroller.
Light readings are lightly smoothed and compared against a target illumination range to estimate light sufficiency.
The processed light sufficiency value is then transmitted wirelessly to the display device via BLE.

---

## Display Device — Physical Gauge and Fill Light Controller

### Description

The display device receives light sufficiency data from the sensing device and provides physical, human-readable feedback through a stepper-motor-driven gauge.
It also controls an LED light strip to provide fill lighting when ambient illumination is insufficient.

### Sensors / Input Components Used

* **Tactile Button (Tact Switch)**
  A tactile push button is used as a simple user input to manually toggle the fill light or reset the system state.

### How It Works

The display device uses an ESP32-C3 microcontroller to receive light sufficiency data via BLE.
Based on the received data, the microcontroller controls a stepper motor to drive the physical gauge needle and adjusts the LED light strip for fill lighting and status indication.
A tactile button allows basic user interaction, such as manual override or system reset.
A custom PCB integrates the motor driver, LED control, and power management, powered by a battery suitable for continuous actuation.

---

## System Communication & Logic

### System Communication Diagram

The sensing device and display device communicate wirelessly using Bluetooth Low Energy (BLE).
Filtered ambient light data is transmitted from the sensing device to the display device, enabling real-time visualization and lighting control.

### Detailed Logic Flow

1. The sensing device reads raw ambient light levels using the BH1750 sensor.
2. Light readings are smoothed using a moving average filter.
3. The filtered light value is compared against a predefined threshold.
4. If the light level is below the threshold, a fill light activation signal is sent to the display device.
5. The display device updates the physical gauge position and turns on the LED light strip as needed.
6. A tactile button allows manual toggling or resetting of the fill light state.

---

## Datasheets

Datasheets for all major components used in this project are included in the `/datasheets` folder:

* BH1750 Ambient Light Sensor
* ESP32-C3 Microcontroller
* 28BYJ-48 Stepper Motor
* ULN2003 Stepper Motor Driver
* LED Light Strip (if applicable)


