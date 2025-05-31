# Overview
<center>
    <img width="250" alt="flash" src="https://external-content.duckduckgo.com/iu/?u=https%3A%2F%2Fce8dc832c.cloudimg.io%2Fv7%2F_cdn_%2F45%2FFA%2FB0%2F00%2F0%2F765780_1.jpg%3Fwidth%3D640%26height%3D480%26wat%3D1%26wat_url%3D_tme-wrk_%252Ftme_new.png%26wat_scale%3D100p%26ci_sign%3D3f6a2ed8b922745361729ae06c2a10c8af578900&f=1&nofb=1&ipt=8d8aabc48a6e4d05965fac8e75034ac75f4a9629bf2af01dbf4e1198a1d286b3" />
</center>

This page is about the firmware flashed on the [B-G431B-ESC1](https://www.st.com/en/evaluation-tools/b-g431b-esc1.html) Discover Kit by ST.


## Rationale
The speed controllers power the motors using _Field Oriented Control_ (FOC) which is substantially more complex and difficult to get right than conventional [electronic speed controllers](https://en.wikipedia.org/wiki/Electronic_speed_control) (for examples those powering drones). The complexity stems from the fact that the controller actively uses the position of the rotor to compute the torque it needs to apply to reach the target velocity. Thus, the noisier and less precise the sensor is, the worse the velocity control of the motor. Furthermore, the speed controller itself uses a PID control algorithm to reach the desired speed, which needs to be tuned manually.

_So why use FOC in the first place?_

We wanted to use direct-drive hub motors to avoid having designing mechanical gears and axels. After all, we're computer scientists, so we decided to shift the mechanical complexity towards software. Typically, brushless motors are designed to spin at high speeds and lower torques (this is the common design case for spinning a propellor). Luckily, there is an open source project called [SimpleFOC](https://www.simplefoc.com/) which implements the control algorithms for us. We now "only" have to write the setup code and supply the PID values. SimpleFOC takes care of the rest. Thus, we picked a motor designed with the lowest RPM rating we could find and the highest torque. Now the only thing that is left is to flash our ESC with the SimpleFOC code and "tune" the PID algorithm so that our motors spin smoothly.

## Tuning
Finding the right PID values for the ESCs was a trial and error process and the result is nowhere near perfect. One of the biggest hurdles in tuning the Motors well, is that in order to have a sense of whether the PID values are producing a smoother (less vibration) drive is to measure the rotational velocity. This was done by the ESC itself, which meant the main loop of the control algorithm had an additional "print" call over the serial connection. This adds a significant amount of time to the FOC control loop changing its behavior. Ideally, external velocity measuring instrumentation would have been used that does not interfere with the control algorithm (this was not done due to time constraints).

## Dangers
The SimpleFOC library allows us to set current and voltage limits that are supplied to the motor. Increasing the voltage and current can easily lead to the motor overheating leading to permanent damage of it and the components around it. Thus, the maximum voltage and current were set to keep the motor within reasonable operating temperatures. **These values must never be changed**.
``` cpp
#define MAX_VOLTAGE 2.7
#define MAX_CURRENT 5.0
```

### Calibration
The firmware will perform a calibration step to verify the motor direction as well as perform some basic checks. If the calibration procedure fails, the onboard LED will flash in an endless loop and the motor will be completely unresponsive. Unfortunately, some speed controllers would exhibit an erroneous behavior even after successful calibration where the motor would lock up and start consuming the maximum amount of current (5 amps). Such ESCs were marked as faulty and were not handed out to students, even if the behavior would only occur sometimes.

