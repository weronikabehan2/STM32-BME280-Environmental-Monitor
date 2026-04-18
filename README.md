STM32F411RE-based environmental monitoring system using BME280 sensor over I2C. 
Features include: raw sensor data acquisition with full factory calibration (temperature, pressure, humidity), integer-arithmetic compensation formulas ported from BME280 datasheet, RTC-stamped periodic readings sent via UART2 every 3 seconds, and EXTI interrupt handling for cycling through 4 display modes (temperature / pressure / humidity / all). 
Configured in normal mode with 2× humidity oversampling, 16× pressure oversampling, and IIR filter coefficient 8.
