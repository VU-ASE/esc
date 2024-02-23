#ifndef PWM_INPUT_H
#define PWM_INPUT_H

#include <SimpleFOC.h>

#define pin  A_BUTTON

uint32_t channel_rising, channel_falling;
volatile uint32_t frequency_measured, dutycycle_measured, last_period_capture = 0, current_capture, high_state_measured;
uint32_t input_freq = 0;
volatile uint32_t rollover_compare_count = 0;
HardwareTimer *timer;


void signal_rising_callback(void) {
  Serial.println("rising callback");
  current_capture = timer->getCaptureCompare(channel_rising);
  /* frequency computation */
  if (current_capture > last_period_capture) {
    frequency_measured = input_freq / (current_capture - last_period_capture);
    dutycycle_measured = (high_state_measured * 100) / (current_capture - last_period_capture);
  } else if (current_capture <= last_period_capture) {
    /* 0x1000 is max overflow value */
    frequency_measured = input_freq / (0x10000 + current_capture - last_period_capture);
    dutycycle_measured = (high_state_measured * 100) / (0x10000 + current_capture - last_period_capture);
  }

  last_period_capture = current_capture;
  rollover_compare_count = 0;
}

/* In case of timer rollover, frequency is to low to be measured set values to 0
   To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision. */
void rollover_callback(void) {
  rollover_compare_count++;

  if (rollover_compare_count > 1) {
    frequency_measured = 0;
    dutycycle_measured = 0;
  }
}


void signal_falling_callback(void) {
  /* prepare DutyCycle computation */
  current_capture = timer->getCaptureCompare(channel_falling);

  if (current_capture > last_period_capture) {
    high_state_measured = current_capture - last_period_capture;
  } else if (current_capture <= last_period_capture) {
    /* 0x1000 is max overflow value */
    high_state_measured = 0x10000 + current_capture - last_period_capture;
  }
}


void test_callback(void) {

  Serial.println("CHANGE!");
  // Serial.println(timer->getCount());

}





void pwm_input_init() {
  pinMode(pin, INPUT);


  // attachInterrupt(digitalPinToInterrupt(pin), test_callback, CHANGE);
  // return;

  // Automatically retrieve TIM instance and channel_rising associated to pin
  // This is used to be compatible with all STM32 series automatically.
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM);
  channel_rising = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM));

  // channel_risings come by pair for TIMER_INPUT_FREQ_DUTY_MEASUREMENT mode:
  // channel_rising1 is associated to channel_falling and channel_rising3 is associated with channel_rising4
  switch (channel_rising) {
    case 1:
      channel_falling = 2;
      break;
    case 2:
      channel_falling = 1;
      break;
    case 3:
      channel_falling = 4;
      break;
    case 4:
      channel_falling = 3;
      break;
  }

  Serial.println(">>> channel_rising ");
  Serial.println(channel_rising);
  Serial.println(">>> channel_falling");
  Serial.println(channel_falling);

  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
  timer = new HardwareTimer(Instance);

  // Configure rising edge detection to measure frequency
  timer->setMode(channel_rising, TIMER_INPUT_CAPTURE_BOTHEDGE, pin);

  // With a PrescalerFactor = 1, the minimum frequency value to measure is : TIM counter clock / CCR MAX
  //  = (SystemCoreClock) / 65535
  // Example on Nucleo_L476RG with systemClock at 80MHz, the minimum frequency is around 1,2 khz
  // To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision.
  // The maximum frequency depends on processing of both interruptions and thus depend on board used
  // Example on Nucleo_L476RG with systemClock at 80MHz the interruptions processing is around 10 microseconds and thus Max frequency is around 100kHz
  uint32_t PrescalerFactor = 1;
  // timer->setPrescaleFactor(PrescalerFactor);
  // timer->setOverflow(0x10000); // Max Period value to have the largest possible time to detect rising edge and avoid timer rollover
  timer->attachInterrupt(channel_rising, test_callback);
  // timer->attachInterrupt(channel_rising, signal_rising_callback);
  // timer->attachInterrupt(channel_falling, signal_falling_callback);
  // timer->attachInterrupt(rollover_callback);
  
  timer->refresh();
  timer->resume();

  // Compute this scale factor only once
  input_freq = timer->getTimerClkFreq() / timer->getPrescaleFactor();
}



#endif
