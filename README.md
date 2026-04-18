# STM32F411RE Environmental Monitoring System

This project implements an embedded environmental monitoring system based on the STM32F411RE microcontroller and the BME280 sensor connected via I2C.

## Features

### Sensor Data Acquisition
- Reads temperature, pressure, and humidity from the BME280 sensor
- Uses full factory calibration for accurate measurements

### Data Processing
- Implements integer-based compensation formulas ported from the BME280 datasheet
- Optimized for embedded systems without floating-point arithmetic

### Periodic Measurements
- RTC-based periodic sampling
- Sensor data transmitted via UART2 every 3 seconds

### User Interface (EXTI Interrupts)
- External interrupt-based mode switching
- Four display modes:
  - Temperature
  - Pressure
  - Humidity
  - All parameters

## Configuration

The BME280 sensor is configured in normal mode with:

- Humidity oversampling: 2x
- Pressure oversampling: 16x
- IIR filter coefficient: 8

## Technologies

- STM32F411RE (ARM Cortex-M4)
- C (embedded firmware)
- STM32 HAL library
- I2C communication protocol
- UART communication
- External interrupts (EXTI)
- RTC (Real-Time Clock)

## Project Focus

- Embedded firmware development in C
- Peripheral configuration on STM32
- Sensor interfacing via I2C
- Interrupt-driven architecture
- Real-time data acquisition
