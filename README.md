# STM32 Industrial Condition Monitor

A bare-metal embedded systems project built around the STM32L476RG and ISM330DHCX 6-axis IMU, developed as a portfolio project for industrial condition monitoring.

The goal is to build the firmware incrementally from low-level sensor interfacing to a complete condition-monitoring node, while keeping the architecture modular and the behavior measurable.


## Hardware
- STM32 NUCLEO-L476RG
- X-NUCLEO-IKS02A1 sensor expansion board
- ISM330DHCX accelerometer and gyroscope


## 1. Project intent

Build a portfolio-grade embedded product that demonstrates:

- Embedded C fundamentals and clean modular design
- STM32CubeMX and STM32CubeIDE competence
- Register-level understanding beneath HAL abstractions
- Interrupt-, DMA-, and RTOS-driven firmware
- Industrial MEMS acquisition and condition monitoring
- UART and CAN communication
- Bootloader design and remotely delivered firmware updates
