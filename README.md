# STM32-AdvancedMotorController
PWM motor controller with PID support running on an STM32 Nucleo-64 with FreeRTOS and Serial Server for PID control adjustments.
This was my final delivery project for embedded systems in semester 3 back in 2022/2023.
The PWM control is written using self made register control code. 
The servo feedback input is also custom register control code.
Incomming serial messages are also stored in the FreeRTOS message queue.

Most of this code has been tested on a STM32 Nucleo-64-F303RE

Multitasking between the pwm control and serial communication is done using multithreading in FreeRTOS.
![mutex diagram of system.](/img/mutexDiagram.png)

