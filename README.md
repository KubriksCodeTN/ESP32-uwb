# IoT Relative Localization and Environmental Sensing System

This project involves the development of high-performance firmware for an **ESP32-based** IoT device. The system integrates **UWB** technology for relative node localization and a suite of environmental sensors, all managed through a custom GUI on an LCD display.

## Overview

The firmware is designed to be fast, robust, and energy-efficient. It localizes surrounding nodes without a fixed global reference and renders a local map in real-time. Key data is also transmitted to a remote server via Wi-Fi for further monitoring.

## Hardware Components

- [cite_start]**MCU**: ESP-32 (4MB RAM) 
- [cite_start]**Localization**: Ultra-Wide Band (UWB) radio 
- **Sensors**:
  - [cite_start]**Magnetometer (LIS2MDL)**: For digital compass implementation.
  - [cite_start]**Accel/Gyro (LSM6DSOX)**: For movement and tilt detection.
  - [cite_start]**Temp/Humidity (SHT4X)**: For environmental monitoring.
  - [cite_start]**Brightness (TSL2591)**: For dynamic LCD backlight adjustment.
- [cite_start]**Display**: 240x240 RGB LCD (SPI-based) 
- [cite_start]**Inputs**: 3 physical buttons for UI navigation 

## Software Architecture

[cite_start]The system is built on the **ESP-IDF** framework and leverages **FreeRTOS** for concurrent task management.

- [cite_start]**Concurrency**: Separate FreeRTOS tasks for each sensor, the GUI, and Wi-Fi communication, using thread-safe queues for data exchange.
- [cite_start]**Graphics**: Custom driver-based library (**dispcolor**) utilizing DMA for efficient SPI display updates.
- [cite_start]**Connectivity**: Wi-Fi station mode sending JSON-formatted data via **UDP** to minimize overhead.
- [cite_start]**Power Optimization**: Implementation of low-power modes for sensors and task suspension based on the active UI screen.

## Power Analysis Highlights

[cite_start]A detailed power analysis was conducted using an oscilloscope and shunt resistor setup:
- [cite_start]**Main Consumers**: ESP32 (especially with Wi-Fi) and the LCD account for ~97-98% of total power.
- [cite_start]**Efficiency**: Sensor consumption is negligible due to optimized polling rates and power-down states when not in use.
