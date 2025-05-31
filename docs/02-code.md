# Code Overview

The speed controller firmware is divided into two procedures: initialization and the main loop. During initialization, we pass the configuration parameters of our motor and the the PID tuning values to the SimpleFOC library. Once everything is configured we call `motor.initFOC();` which will perform the calibration procedure. On success, the main loop starts executing which can be boiled down to doing two things: read the PWM input command and call `motor.loopFOC();` and `motor.move(input);` to actuate the motor.

## Reading PWM Input
The ESC is command using a PWM signal which is a single wire carrying a digital high/low signal. PWM works by measuring the time that the signal was high compared to the period (known as duty cycle) resulting in a percentage that is passed to the SimpleFOC library. Importantly, calculating the duty cycle requires accurate timing measurements which was achieved unutilized hardware timers. These timers trigger interrupts when the signal changes updating a global variable holding the current duty cycle time. The code responsible for this was separated into a header file `pwm_input.h`.

