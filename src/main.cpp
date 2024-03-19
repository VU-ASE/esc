#include <SimpleFOC.h>
#include "pwm_input.h"


BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

LowsideCurrentSense currentSense = LowsideCurrentSense(0.003, -64.0/7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);


#define MONITOR false
#define SKIP_CALIBRATION true

enum WhichMotor {
  LEFT_MOTOR,
  RIGHT_MOTOR
};

// CHANGE THIS DEPENDING ON WHICH MOTOR YOU ARE FLASHING
enum WhichMotor this_motor = LEFT_MOTOR;
// enum WhichMotor this_motor = RIGHT_MOTOR;


#define TOP_SPEED 150.0
float throttle = 0.0;

#define MAX_VOLTAGE 2.7
#define MAX_CURRENT 7.0

// Could value for Current setup
#define LOW_PASS_FILTER 0.1
#define SENSOR_ALIGN_VOLTAGE 1.0
#define MOTION_DOWN_SAMPLE 0.0

Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);
  
  Serial.println("-------------");
  Serial.print("TOP_SPEED: ");
  Serial.println(TOP_SPEED);
  Serial.println("--------------");


  // MAGNETIC SENSOR INIT
  sensor.init();
  motor.linkSensor(&sensor);


  // DRIVER INIT
  driver.voltage_power_supply = 16;
  driver.init();
  motor.linkDriver(&driver);

  // MOTOR INIT
  motor.current_limit = MAX_CURRENT;
  motor.voltage_limit = MAX_VOLTAGE;
  motor.velocity_limit = TOP_SPEED;
  motor.voltage_sensor_align = SENSOR_ALIGN_VOLTAGE;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::velocity;


  // MOTOR PID SETTINGS - TODO TUNE THESE MORE
  motor.PID_velocity.P = 0.15;
  motor.PID_velocity.I = 0.9;
  motor.LPF_velocity.Tf = LOW_PASS_FILTER;


  if (SKIP_CALIBRATION) {
    if (this_motor == LEFT_MOTOR) {
      // wires are NOT color-aligned!
      motor.sensor_direction = Direction::CCW;
      motor.zero_electric_angle = 3.60;
    } else if (this_motor == RIGHT_MOTOR) {
      // wires are color-aligned!
      motor.sensor_direction = Direction::CW;
      motor.zero_electric_angle = 3.60;
    }
  }


  // CURRENT SENSOR
  currentSense.linkDriver(&driver);
  currentSense.init();
  motor.linkCurrentSense(&currentSense);


  motor.init();
  motor.initFOC();


  // Serial.print("motor.zero_electric_angle:   ");
  // Serial.println(motor.zero_electric_angle);
  // Serial.print("motor.sensor_direction:   ");
  // Serial.println(motor.sensor_direction);
  // while (true) {}

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
  if (adjusted_duty < 100 || adjusted_duty > 200) {
    throttle = 0.0;
  } else {
    throttle = ((float) (adjusted_duty-100) / 100.0) * (TOP_SPEED * 2) - TOP_SPEED;
  }

  motor.loopFOC();
  motor.move(throttle);

  if (MONITOR) {
    Serial.print(adjusted_duty);
    Serial.print("    ");
    Serial.print(throttle);
    Serial.println("    ");
  }
}


