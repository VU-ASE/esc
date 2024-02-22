#ifndef PWM_NAIVE_H
#define PWM_NAIVE_H

#include <SimpleFOC.h>

uint32_t channel_change = 1;
volatile long delta = 0;
volatile long current_time = 0;
volatile long prev_time = 0;
volatile long disconnect_counter = 0;
int current_state = 0;


HardwareTimer *MyTimNaive;

// #define pin LED_BUILTIN
#define pin A_PWM

long custom_timer() {
  return MyTimNaive->getCaptureCompare(channel_change, MICROSEC_COMPARE_FORMAT);
}

void signal_change_callback(void) {
  disconnect_counter = 0;
  int current_state = digitalRead(A_PWM);  
  if (current_state == HIGH) {
    prev_time = custom_timer();
    return;
  }
  current_time = custom_timer();
  delta = current_time - prev_time;
  // if (delta > 0 ) {
  //   time_between_pulses_micro_seconds = smooth_input(delta);
  // }
}

void pwm_signal_init_naive() {
  Serial.println(">>> initializing pwm input");
  

  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM);
  channel_change = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM));
  // channel_change = 1;

  Serial.print(">>> got channel: ");
  Serial.println(channel_change);

  // pinMode(pin, INPUT);
  // pinMode(LED_BUILTIN, OUTPUT);

  MyTimNaive = new HardwareTimer(Instance);


  
  Serial.print(">>> setting overflow...");
  MyTimNaive->setOverflow(1, HERTZ_FORMAT);
  Serial.print(">>> setting mode...");
  MyTimNaive->setMode(channel_change, TIMER_OUTPUT_COMPARE_PWM1, pin);
  Serial.print(">>> setting capture compare...");
  MyTimNaive->setCaptureCompare(channel_change, 50, PERCENT_COMPARE_FORMAT);



  // MyTimNaive->setMode(channel_change, TIMER_INPUT_CAPTURE_BOTHEDGE, pin);


  

  Serial.println(">>> created new timer");

  // uint32_t PrescalerFactor = 1;
  // MyTimNaive->setPrescaleFactor(PrescalerFactor);
  // MyTimNaive->setOverflow(0x10000);

  // MyTimNaive->attachInterrupt(channel_change, signal_change_callback);

  // Serial.println(">>> attached interrupts");

  // MyTimNaive->resume();

  // Serial.println(">>> resumed timer");

//   // Compute this scale factor only once
//   input_freq = MyTimNaive->getTimerClkFreq() / MyTimNaive->getPrescaleFactor();
}


#endif
