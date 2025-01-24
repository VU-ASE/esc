//
// ESC - software to flash on the B-G431 ESC1 devkit powering the ASE Rover.
//
// Author: Maximilian Gallup
//
// The following piece of software is provided as is and provides no guarantees
// towards quality and reliability. It is the result of a research project that
// turned into an educational tool. Relying entirely on the open source library
// "SimpleFOC" this driver code merely provides some tuning values for the PID
// control system that the library implements for us.
//
// This piece of software is implemented for an entry level cheap devkit:
//    https://www.st.com/en/evaluation-tools/b-g431b-esc1.html
//


#include <SimpleFOC.h>
#include "pwm_input.h"

// SimpleFOC library defines datastructures for the motor that we use as well
// as the drivers.
BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

// We are using an off-the-shelf AS5600 magnetic encoder for sensing shaft
// position and the devkit features a current sensor for more fine-grained control.
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003, -64.0/7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// Enable debugging over serial - set this to false when flashing for production.
#define MONITOR false

// Unfortunately the two motors of the rover are mounted in opposite directions
// and the software currently does not implement any tricks to not have
// to differentiate between them so that they spin in the right direction.
enum WhichMotor {
  LEFT_MOTOR,
  RIGHT_MOTOR
};

// Set the current motor to flash here.
enum WhichMotor this_motor = LEFT_MOTOR;

// The actual target "velocity" value of the motor.
float throttle = 0.0;

// This value caps the speed in RPM. Unfortunately, the motors don't even spin up anywhere
// near this value, but it is a high target that the PID loop tries to achieve.
#define TOP_SPEED 150.0

// During testing these values seemed to keep the motors from getting too hot.
#define MAX_VOLTAGE 2.7
#define MAX_CURRENT 5.0

// More values that would require better testing equipment to thoroughly understand.
#define LOW_PASS_FILTER 0.1
#define SENSOR_ALIGN_VOLTAGE 1.0
#define MOTION_DOWN_SAMPLE 0.0


// This code runs once during initialization only.
void setup() {

  // Setup of the status LED on the board.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  if (MONITOR) {
    Serial.begin(115200);
    SimpleFOCDebug::enable(&Serial);
    
    Serial.println("-------------");
    Serial.print("TOP_SPEED: ");
    Serial.println(TOP_SPEED);
    Serial.println("--------------");
  }

  // Initialize magnetic encoder using library.
  sensor.init();
  motor.linkSensor(&sensor);


  // Initialize driver with voltage roughly equal to fully charged voltage.
  driver.voltage_power_supply = 16;
  driver.init();
  motor.linkDriver(&driver);

  // Set predefined limits
  motor.current_limit = MAX_CURRENT;
  motor.voltage_limit = MAX_VOLTAGE;
  motor.velocity_limit = TOP_SPEED;
  motor.voltage_sensor_align = SENSOR_ALIGN_VOLTAGE;

  // Voltage control seemed to work best for the torque controller
  // and velocity control method is desired for our use-case.
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::velocity;

  // The actual tuning values of the PID controller that aims to
  // keep a target velocity set by the throttle variable.
  // These values get tuned.
  motor.PID_velocity.P = 0.15;
  motor.PID_velocity.I = 0.9;
  motor.LPF_velocity.Tf = LOW_PASS_FILTER;

  // Initialize the current sensor
  currentSense.linkDriver(&driver);
  currentSense.init();
  motor.linkCurrentSense(&currentSense);

  motor.init();
  int init_success = motor.initFOC();

  if (!init_success) {
    if (MONITOR) {
      Serial.print("---INIT FOC FAILED---");
    }

    while (true) {
      digitalWrite(LED_BUILTIN, HIGH);
      digitalWrite(LED_RED, HIGH);
      _delay(50);
      digitalWrite(LED_BUILTIN, LOW);
      digitalWrite(LED_RED, LOW);
      _delay(50);
    }
  }

  if (MONITOR) {
    Serial.print(">>> motor.zero_electric_angle:   ");
    Serial.println(motor.zero_electric_angle);
    Serial.print(">>> motor.sensor_direction:   ");
    Serial.println(motor.sensor_direction);
  }

  // This function initializes the PWM input reader to read the input voltage
  // for our control.
  pwm_input_init();

  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_RED, HIGH);
    _delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LED_RED, LOW);
    _delay(50);
  }

}


void loop() {
  // The adjusted_duty variable is a global that is read without locks
  uint32_t local_adjusted_duty = adjusted_duty;

  // Stop the motors if it is in an unreasonable range
  if (local_adjusted_duty < 100 || local_adjusted_duty > 200) {
    throttle = 0.0;
  } else {
    // Update the throttle value to the scale of [-TOP_SPEED, TOP_SPEED
    throttle = ((float) (local_adjusted_duty-100) / 100.0) * (TOP_SPEED * 2) - TOP_SPEED;
  }

  // Set the velocity through SimpleFOC
  motor.loopFOC();
  motor.move(throttle);
}


