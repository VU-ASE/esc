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

// This value caps the speed in RPM. Unfortunately, the motors don't even spin up anywhere
// near this value, but it is a high target that the PID loop tries to achieve.
#define TOP_SPEED 250.0
// During testing these values seemed to keep the motors from getting too hot.
#define MAX_VOLTAGE 2.7
#define MAX_CURRENT 5.0
// More values that would require better testing equipment to thoroughly understand.
#define LOW_PASS_FILTER 0.1
#define SENSOR_ALIGN_VOLTAGE 1.0
#define MOTION_DOWN_SAMPLE 0.0

/**
 * 1. Sensor initialization (https://docs.simplefoc.com/code#step-1-position-sensor-setup)
 * We are using an off-the-shelf AS5600 magnetic encoder for sensing the shaft position.
 */
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

/**
 * 2. Driver initialization (https://docs.simplefoc.com/code#step-2--driver-setup)
 * Configure the drivers for our brushless motors and set up the PWM outputs for them.
 * For each phase, we specify a high side and a low side
 */
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);

/**
 * 3. Current sensor initialization (https://docs.simplefoc.com/code#step-3--current-sense-setup)
 * The devkit features a current sensor for more fine-grained control.
 */
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003, -64.0 / 7.0, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

/**
 * 4. Motor initialization (https://docs.simplefoc.com/code#step-4--motor-setup-)
 * Brushless DC motors (BLDC)
 */
BLDCMotor motor = BLDCMotor(7);

void blinkShort()
{
  digitalWrite(LED_RED, HIGH);
  _delay(200);
  digitalWrite(LED_RED, LOW);
  _delay(200);
}

void blinkLong()
{
  digitalWrite(LED_RED, HIGH);
  _delay(600);
  digitalWrite(LED_RED, LOW);
  _delay(200);
}

void blinkSOS()
{
  // S: ...
  blinkShort();
  blinkShort();
  blinkShort();

  _delay(600); // gap between letters

  // O: ---
  blinkLong();
  blinkLong();
  blinkLong();

  _delay(600); // gap between letters

  // S: ...
  blinkShort();
  blinkShort();
  blinkShort();

  _delay(1400); // wait before repeating SOS
}

/**
 * Initialization code, runs once on startup.
 */
void setup()
{
  // Set up te red built in LED
  pinMode(LED_RED, OUTPUT);


  if (MONITOR)
  {
    Serial.begin(115200);
    SimpleFOCDebug::enable(&Serial);

    Serial.println("-------------");
    Serial.print("TOP_SPEED: ");
    Serial.println(TOP_SPEED);
    Serial.println("--------------");
  }

  // 1. Initialize magnetic sensor
  sensor.init();
  motor.linkSensor(&sensor);

  // 2. Initialize driver with voltage roughly equal to fully charged voltage.
  driver.voltage_power_supply = 16;
  driver.init();
  motor.linkDriver(&driver);

  // 3. Set up motor and its predefined limits
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
  // velocity low pass filtering
  // default 5ms - try different values to see what is the best.
  // the lower the less filtered
  motor.LPF_velocity.Tf = LOW_PASS_FILTER;

  // 4. Initialize the current sensor
  currentSense.linkDriver(&driver);
  currentSense.init();
  motor.linkCurrentSense(&currentSense);

  // Indicate initialization when the red LED is powered
  digitalWrite(LED_RED, HIGH);
  motor.init();
  int init_success = motor.initFOC();
  if (!init_success)
  {
    if (MONITOR)
    {
      Serial.print("---INIT FOC FAILED---");
    }

    // Visual signal that something went wrong
    while (true)
    {
      blinkSOS();
    }
  }

  if (MONITOR)
  {
    Serial.print(">>> motor.zero_electric_angle:   ");
    Serial.println(motor.zero_electric_angle);
    Serial.print(">>> motor.sensor_direction:   ");
    Serial.println(motor.sensor_direction);
  }

  // This function initializes the PWM input reader to read the input voltage
  // for our control.
  pwm_input_init();

  // Turn off red LED to show init is done
  digitalWrite(LED_RED, LOW);
  for (int i = 0; i < 10; i++)
  {
    digitalWrite(LED_RED, HIGH);
    _delay(50);
    digitalWrite(LED_RED, LOW);
    _delay(50);
  }
}

// If the duty cycle is 11.00 or lower, the throttle should be 100.0
// If the duty cycle is between 18 and 19, the throttle should be 0.0
// If the duty cycle is 25 or higher, the throttle should be -100.0
float compute_throttle(float dutyCycle)
{
    if (dutyCycle <= 11.0f) {
        return 100.0f;
    } 
    else if (dutyCycle >= 25.0f) {
        return -100.0f;
    }
    else if (dutyCycle >= 18.0f && dutyCycle <= 19.0f) {
        return 0.0f;
    }
    else if (dutyCycle > 11.0f && dutyCycle < 18.0f) {
        // Linearly interpolate from 100 to 0
        float t = (dutyCycle - 11.0f) / (18.0f - 11.0f);
        return 100.0f * (1.0f - t);
    }
    else if (dutyCycle > 19.0f && dutyCycle < 25.0f) {
        // Linearly interpolate from 0 to -100
        float t = (dutyCycle - 19.0f) / (25.0f - 19.0f);
        return -100.0f * t;
    }

    // Fallback
    return 0.0f;
}

// The actual throttle value to persist among loop iterations
float throttle = 0.0;
uint32_t last_rising = 0;
void loop()
{
  // Read captured values
  // uint32_t t_rising = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
  // uint32_t t_falling = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);

  // // Compute pulse width (high time)
  // uint32_t pulse_width = (t_falling >= t_rising)
  //                            ? (t_falling - t_rising)
  //                            : (0xFFFF - t_rising + t_falling);

  // // Fixed period for 50 Hz PWM
  const uint32_t period = 20000; // in microseconds
  // Read pulse width locally (copy from global val)
  const uint32_t pw = pulse_width;
  // Compute duty cycle
  float duty_cycle = ((float)pw / period) * 100.0;
  float t = compute_throttle(duty_cycle);

  #if MONITOR
  Serial.printf("Got pw %u and throttle %f and duty cycle %f\n", pw, t, duty_cycle);
  #endif

  // Set the velocity through SimpleFOC
  motor.loopFOC();
  motor.move(t);
}

