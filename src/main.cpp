#include <SimpleFOC.h>

#include "basic_pwn_in.h"

BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

LowsideCurrentSense currentSense = LowsideCurrentSense(0.003, -64.0/7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);


// This makes the motors warm, removing plastic from Motor shaft would make this ok.
#define MAX_VOLTAGE 2.7
#define MAX_CURRENT 7.0

// Could value for Current setup
#define LOW_PASS_FILTER 0.1
#define SENSOR_ALIGN_VOLTAGE 1.0
#define MOTION_DOWN_SAMPLE 0.0

// This can be pushed more when voltage goes up
#define TOP_SPEED 150.0


#define MONITOR true

Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {
  if (MONITOR) {
    Serial.begin(115200);
    SimpleFOCDebug::enable(&Serial);
    Serial.println("-------------");
    Serial.print("TOP_SPEED: ");
    Serial.println(TOP_SPEED);
    Serial.println("--------------");
    // motor.useMonitoring(Serial);
    // motor.monitor_downsample = MOTION_DOWN_SAMPLE;
    // command.add('M', doMotor, "motor");
    // command.add('A', doMotor, "my motor");
  }


  // INIT FOR READING PWM
  pinMode(A_PWM, INPUT);
  pwm_signal_init();


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



  // TODO These values need to be set depending which side 
  // the motor is on to prevent direction search calibration.
  // motor.sensor_direction = Direction::CCW;
  // motor.zero_electric_angle = 5.0759;


  // CURRENT SENSOR
  currentSense.linkDriver(&driver);
  currentSense.init();
  motor.linkCurrentSense(&currentSense);


  motor.init();
  motor.initFOC();

  _delay(1000);
}



// TODO test if we need smoothing window
#define SMOOTHING_WINDOW_SIZE 20
float input_window[SMOOTHING_WINDOW_SIZE];
int counter = 0;

float smooth_input(float new_value) {
  input_window[counter % SMOOTHING_WINDOW_SIZE] = new_value;
  counter++;

  float sum = 0.0;
  for (int i = 0; i < SMOOTHING_WINDOW_SIZE; i++) {
    sum += input_window[i];
  }

  // Serial.print("sum: ");
  // Serial.print(sum);
  // Serial.print("sum: ");
  // Serial.print(sum);

  return sum / ((float) SMOOTHING_WINDOW_SIZE);
}




#define DEAD_ZONE_THRESH 0.03
float target = 0.0;

void loop() {
  pwm_signal_read();

  float target = smooth_input( (float) ((int) (throttle * TOP_SPEED)) );
  // float target = throttle * TOP_SPEED;
  // float target = throttle;
  
  // target = val * TOP_SPEED;

  // TODO TUNE this
  // if (target <= DEAD_ZONE_THRESH && target >= 0.0f - DEAD_ZONE_THRESH) {
  //   target = 0.0;
  // }


  motor.loopFOC();
  motor.move(target);


  if (MONITOR) {
    Serial.print("    ");
    Serial.print(disconnect_counter);
    Serial.print("    ");
    Serial.print(time_between_pulses_micro_seconds);
    Serial.print("    ");
    Serial.println(target);

    // motor.monitor();
    // command.run();
  }
}





