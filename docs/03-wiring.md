# Wire Connections

There are three (red, black & yellow) wires that connect directly to the motor (known as phase wires). A convenient fact about brushless motors is that if you want to change the direction in which the motor spins, you can simply swap any two of the three wires. The ESC needs to be connected to the motor using the bullet connectors. It is important that the following color-connection between ESC and motor is followed:

* Left Motor:
    * any two motor-esc wires must **NOT** match color
    * plugs into channel 1 on the PCA board
* Right Motor:
    * all motor-esc wires must match color
    * plugs into channel 2 on the PCA board

![Back of ESC with annotations](https://github.com/VU-ASE/.github/blob/main/images/esc/esc-back.png?raw=true)

<!-- ![Front of ESC with annotations](https://github.com/VU-ASE/.github/blob/main/images/esc/esc-front.png?raw=true) -->


## Magnetic Encoder
The magnetic encoder connects over an I2C connection using the following pads:
![Back of ESC close up with annotations](https://github.com/VU-ASE/.github/blob/main/images/esc/esc-back-close-up.png?raw=true)

## PWM Input
The pwm uses a pair of wires, one data wire and one ground. They are connected as follows:
![Front of ESC close up with annotations](https://github.com/VU-ASE/.github/blob/main/images/esc/esc-front-close-up.png?raw=true)

