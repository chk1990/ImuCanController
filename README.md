# ImuCanController
This repository provides a driver for STM32F103CBTx controller to a STM LSM9DS1 IMU and a CAN interface.

## Used software
- STM32CubeIDE 1.11.0

## Used libraries
- For the LSM9DS1 IMU chip [this](https://github.com/chk1990/interfacesHardware) library should have been used. When compiling it the binary file gets too large for the controller's flash memory.