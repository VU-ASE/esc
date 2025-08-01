# Known Issues

Due to the inherent complexity of FOC Speed controllers and limited time, there are a number of issues that can crop up during the lifetime of a Rover. The following is a quick breakdown of various known issues followed by a detailed guide to solve some of them.

### *ESC does not show up as a USB device after being plugged in*
We have experienced a number of ESCs dying such that they no longer appear on a host device after being plugged in. Some even started to become incredibly hot after being plugged in which indicates it is **dead** and needs to be replaced. Unfortunately there is no other fix for this case other than replacing with a spare.

### *Suddenly after calibration, motor starts to vibrate violently and becomes very hot* 

The issue here is that (for some reason unknown) the magnetic encoder is telling the ESC that the motor is spinning in the reversed direction than it actually is. Despite there being a calibration step that detects motor direction, the sensor is simply informing the ESC of the wrong direction and must be switched. To prevent having to flash ESCs with different firmware we can actually fix this in hardware! From the [datasheet](https://www.mouser.com/pdfdocs/AMS_AS5600_Datasheet_EN.PDF) of the magnetic encoder we can see that a dedicated `DIR` pin can be used to switch the direction of values being incremented. Depending on whether the `DIR` pin is bridge to `VCC` or `GND` we can set the direction in which the values increase. By plugging in the ESC to a serial monitor we can see the logs of the calibration. If the ESC incorrectly detects the direction use the following mapping to adjust it.

* Bridge `DIR -> GND` for CW
* Bridge `DIR -> VCC` for CCW

For example, in the following case the ESC incorrectly detected the direction to be CCW so we flip the value readings by bridging `DIR -> GND` allowing smooth operation of the motor.

![AS5600 showing jumper cable bridging "DIR" pin to "GND"](https://github.com/VU-ASE/.github/blob/main/images/esc/esc-jumper.png?raw=true)


