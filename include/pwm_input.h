#ifndef PWM_INPUT_H
#define PWM_INPUT_H

#include <SimpleFOC.h>

#define pin A_PWM

uint32_t channel_rising, channel_falling;
volatile uint32_t frequency_measured, adjusted_duty = 150, last_total_period = 0;
uint32_t input_freq = 0;
volatile uint32_t rollover_compare_count = 0;
HardwareTimer *timer;

volatile uint32_t first_rise = 0, first_fall = 0, second_rise = 0;

#define MAGIC_OFFSET 69 + 100

#define MAX_ALLOWED_DIFFERENCE 100

void signal_rising_callback(void)
{
  first_rise = second_rise;
  second_rise = timer->getCaptureCompare(channel_rising);

  // The first and second rise are a sliding window, only when they are both defined in increasing order we have a
  if (second_rise > first_rise && first_rise != 0 && second_rise != 0 && first_fall > first_rise && first_fall < second_rise) {
    uint32_t total_period = second_rise - first_rise;
    uint32_t high_period = first_fall - first_rise;
    

    if (total_period > MAX_ALLOWED_DIFFERENCE + last_total_period  || total_period < last_total_period - MAX_ALLOWED_DIFFERENCE) {
      last_total_period = total_period;
      return;
    }

    last_total_period = total_period;

    adjusted_duty = (high_period * 1600) / (total_period)-MAGIC_OFFSET;
  }

  rollover_compare_count = 0;
}

/* In case of timer rollover, frequency is to low to be measured set values to 0
   To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision. */
void rollover_callback(void)
{
  rollover_compare_count++;

  if (rollover_compare_count > 1)
  {
    // frequency_measured = 0;
    adjusted_duty = 0;
    first_rise = 0;
    second_rise = 0;
    first_fall = 0;
  }
}

void signal_falling_callback(void)
{
  first_fall = timer->getCaptureCompare(channel_falling);
}

void pwm_input_init()
{
  pinMode(pin, INPUT);

  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM);
  channel_rising = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM));

  // channel_risings come by pair for TIMER_INPUT_FREQ_DUTY_MEASUREMENT mode:
  // channel_rising1 is associated to channel_falling and channel_rising3 is associated with channel_rising4
  switch (channel_rising)
  {
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

  timer = new HardwareTimer(Instance);

  // Configure rising edge detection to measure frequency
  timer->setMode(channel_rising, TIMER_INPUT_FREQ_DUTY_MEASUREMENT, pin);

  uint32_t PrescalerFactor = 170;
  timer->setPrescaleFactor(PrescalerFactor);
  timer->setOverflow(0x10000); // Max Period value to have the largest possible time to detect rising edge and avoid timer rollover
  timer->attachInterrupt(channel_rising, signal_rising_callback);
  timer->attachInterrupt(channel_falling, signal_falling_callback);
  timer->attachInterrupt(rollover_callback);

  timer->refresh();
  timer->resume();

  // Compute this scale factor only once
  input_freq = timer->getTimerClkFreq() / timer->getPrescaleFactor();
}

#endif
