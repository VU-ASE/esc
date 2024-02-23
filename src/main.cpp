#include <SimpleFOC.h>
#include "pwm_input.h"
#include "pwm_naive.h"



BLDCMotor motor = BLDCMotor(7);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

LowsideCurrentSense currentSense = LowsideCurrentSense(0.003, -64.0/7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);




 

#define MONITOR true
#define DO_CALIBRATION false

enum WhichMotor {
  LEFT_MOTOR,
  RIGHT_MOTOR
};


// CHANGE THIS DEPENDING ON WHICH MOTOR YOU ARE FLASHING
enum WhichMotor this_motor = RIGHT_MOTOR;


const long SMOOTHING_WINDOW_SIZE = 10;
long input_window[SMOOTHING_WINDOW_SIZE] = {0};
int index_num = 0;
long sum = 0;

long smooth_input(long new_value) {
  index_num = (index_num + 1) % SMOOTHING_WINDOW_SIZE;
  sum -= input_window[index_num];
  input_window[index_num] = new_value;
  sum += new_value;
  return sum / SMOOTHING_WINDOW_SIZE;
}



// Values from PCA as tuned by the servo tester
#define PULSE_LOW_END 1087
#define PULSE_HIGH_END 1860

#define TOP_SPEED 150.0

// volatile long prev_time = 0;
// volatile long current_time = 0;
// volatile long disconnect_counter = 0;
volatile long time_between_pulses_micro_seconds = 0;

float throttle = 0.0;




// uint32_t custom_timer() {
//   // timer->pause();
//   uint32_t res = timer->getCount(MICROSEC_FORMAT);
//   // timer->resume();
//   // timer->setCount(0);
//   // return micros();
//   return res;

// }
// void signal_change() {
//   disconnect_counter = 0;
//   int current_state = digitalRead(A_PWM);
//   if (current_state == HIGH) {
//     prev_time = custom_timer();
//     return;
//   }
//   current_time = custom_timer();
//   delta = current_time - prev_time;
//   // if (delta > 0 ) {
//   //   time_between_pulses_micro_seconds = smooth_input(delta);
//   // }
// }



// #define DEAD_ZONE_THRESH 1.0f

// void pwm_signal_read() {
//   if (disconnect_counter > 200) {
//     throttle = 0.0f;
//     return;
//   }

//   if (time_between_pulses_micro_seconds <= PULSE_HIGH_END && time_between_pulses_micro_seconds >= PULSE_LOW_END) {
//     throttle = (((time_between_pulses_micro_seconds - PULSE_LOW_END) / (float) (PULSE_HIGH_END - PULSE_LOW_END)) * 2.0f * TOP_SPEED) - TOP_SPEED;

//     if (throttle > TOP_SPEED) { throttle = TOP_SPEED; }
//     if (throttle < -TOP_SPEED) { throttle = -TOP_SPEED; }

//     // For now no dead_zone
//     // if ((throttle <= DEAD_ZONE_THRESH) && (throttle >= 0.0f - DEAD_ZONE_THRESH)) {
//     //   throttle = 0.0;
//     // }
//   }
//   disconnect_counter += 1;
// }


// void pwm_signal_init() {
  // pinMode(A_PWM, INPUT);
  // timer->setMode(0, TIMER_INPUT_CAPTURE_BOTHEDGE, A_PWM);
  // timer->setOverflow(10000, MICROSEC_FORMAT);
  // timer->resume();

  // USE THIS 
  // https://github.com/stm32duino/STM32Examples/blob/main/examples/Peripherals/HardwareTimer/Frequency_Dutycycle_measurement/Frequency_Dutycycle_measurement.ino
  
  
  
  // attachInterrupt(digitalPinToInterrupt(A_PWM), signal_change, CHANGE);
  
  // digitalPinToInterrupt(A_PWM) 


  // todo: attach interrupt to timer2
  // todo: set time_between_pulses_micro_seconds to be pulses

// }

// This makes the motors warm, removing plastic from Motor shaft would make this ok.
#define MAX_VOLTAGE 2.7
#define MAX_CURRENT 7.0

// Could value for Current setup
#define LOW_PASS_FILTER 0.1
#define SENSOR_ALIGN_VOLTAGE 1.0
#define MOTION_DOWN_SAMPLE 0.0

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
  }

  // INIT FOR READING PWM
  


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


  if (DO_CALIBRATION) {
    if (this_motor == LEFT_MOTOR) {
      motor.sensor_direction = Direction::CCW;

    } else if (this_motor == RIGHT_MOTOR) {
      motor.sensor_direction = Direction::CW;
    }
  }


  // CURRENT SENSOR
  currentSense.linkDriver(&driver);
  currentSense.init();
  motor.linkCurrentSense(&currentSense);


  motor.init();
  motor.initFOC();

  pwm_input_init();

  _delay(1500);
}


void loop() {
  motor.loopFOC();
  motor.move(throttle);

  if (MONITOR) {
    // Serial.print(frequency_measured);
    // Serial.print("    ");
    // Serial.print(dutycycle_measured);
    // Serial.println("    ");
    // Serial.println(throttle);
  }
}


