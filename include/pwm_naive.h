#ifndef PWM_NAIVE_H
#define PWM_NAIVE_H

#include <SimpleFOC.h>

uint32_t channel_change = 1;
volatile long delta = 0;
volatile long current_time = 0;
volatile long prev_time = 0;
volatile long disconnect_counter = 0;
int current_state = 0;


HardwareTimer *timer;

// #define pin LED_BUILTIN
#define pin A_PWM

long custom_timer() {
  return timer->getCaptureCompare(channel_change, MICROSEC_COMPARE_FORMAT);
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




// void signal_change() {
//   Serial.println("change: ");
//   // Serial.println(timer->getCount());
// }


#define pin A_PWM

void pwm_signal_init_naive() {

  // REQUIRES uncommenting line 131 of packages/framework-arduinoststm32/variants/STM32G4xx/G431(6-8-B)U_G441CBU/PeriperalPins_B_G431B_ESC1.c
  // specifically the symbol PinMap_TIM[] needs to have the PA_15 timer enable (which we use for PWM)

  pinMode(pin, INPUT);
  
  Serial.println(">>> initializing pwm input");

  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM);
  channelRising = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM));


  switch (channelRising) {
    case 1:
      channelFalling = 2;
      break;
    case 2:
      channelFalling = 1;
      break;
    case 3:
      channelFalling = 4;
      break;
    case 4:
      channelFalling = 3;
      break;
  }

  Serial.println(">>> got channelRising");
  Serial.println(channelRising);
  Serial.println(">>> got channelFalling");
  Serial.println(channelFalling);
  
  timer = new HardwareTimer(Instance);

  Serial.println(">>> setting prescaler...");
  timer->setPrescaleFactor(8500);
  
  Serial.println(">>> setting overflow...");
  timer->setOverflow(1, HERTZ_FORMAT);

  Serial.println(">>> setting mode");
  timer->setMode(channelRising, TIMER_INPUT_FREQ_DUTY_MEASUREMENT, pin, FILTER_NONE);

  timer->refresh();
  timer->resume();


  // Serial.println(">>> setting overflow...");
  // timer->setOverflow(1, HERTZ_FORMAT);
  
  // Serial.println(">>> testing different pins...");

  

  // uint32_t function = pinmap_function(digitalPinToPinName(A_POTENTIOMETER), PinMap_TIM);
  // uint32_t result = STM_PIN_CHANNEL(function);

  // Serial.println(">>> result:");
  // Serial.println(result);


  // Serial.println(">>> setting mode...");
  // timer->setMode(channel_change, TIMER_DISABLED, pin);
  // Serial.println(">>> setting capture compare...");
  // timer->setCaptureCompare(channel_change, 50, PERCENT_COMPARE_FORMAT);



  // timer->setMode(channel_change, TIMER_INPUT_CAPTURE_BOTHEDGE, pin);


  

  Serial.println(">>> created new timer and sitting");
  while(true){};

  // uint32_t PrescalerFactor = 1;
  // timer->setPrescaleFactor(PrescalerFactor);
  // timer->setOverflow(0x10000);

  // timer->attachInterrupt(channel_change, signal_change_callback);

  // Serial.println(">>> attached interrupts");

  // timer->resume();

  // Serial.println(">>> resumed timer");

//   // Compute this scale factor only once
//   input_freq = timer->getTimerClkFreq() / timer->getPrescaleFactor();
}


#endif
