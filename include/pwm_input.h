#ifndef PWM_INPUT_H
#define PWM_INPUT_H

#include <SimpleFOC.h>

// volatile long delta = 0;
uint32_t channelRising = 1;
uint32_t channelFalling;
volatile uint32_t FrequencyMeasured, DutycycleMeasured, LastPeriodCapture = 0, CurrentCapture, HighStateMeasured;
uint32_t input_freq = 0;
volatile uint32_t rolloverCompareCount = 0;
HardwareTimer *MyTim;

#define pin A_PWM


void signal_rising_callback(void) {
  CurrentCapture = MyTim->getCaptureCompare(channelRising);
  /* frequency computation */
  if (CurrentCapture > LastPeriodCapture) {
    FrequencyMeasured = input_freq / (CurrentCapture - LastPeriodCapture);
    DutycycleMeasured = (HighStateMeasured * 100) / (CurrentCapture - LastPeriodCapture);
  } else if (CurrentCapture <= LastPeriodCapture){
    /* 0x1000 is max overflow value */
    FrequencyMeasured = input_freq / (0x10000 + CurrentCapture - LastPeriodCapture);
    DutycycleMeasured = (HighStateMeasured * 100) / (0x10000 + CurrentCapture - LastPeriodCapture);
  }

  LastPeriodCapture = CurrentCapture;
  rolloverCompareCount = 0;
}

/* In case of timer rollover, frequency is to low to be measured set values to 0
   To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision. */
void overflow_callback(void) {
  rolloverCompareCount++;

  if (rolloverCompareCount > 1) {
    FrequencyMeasured = 0;
    DutycycleMeasured = 0;
  }
}


void signal_falling_callback(void) {
  CurrentCapture = MyTim->getCaptureCompare(channelFalling);
  if (CurrentCapture > LastPeriodCapture) {
    HighStateMeasured = CurrentCapture - LastPeriodCapture;
  } else if (CurrentCapture <= LastPeriodCapture) {
    /* 0x1000 is max overflow value */
    HighStateMeasured = 0x10000 + CurrentCapture - LastPeriodCapture;
  }
}


void pwm_signal_init() {
  Serial.println(">>> initializing pwm input");
  // channelRisings come by pair for TIMER_INPUT_FREQ_DUTY_MEASUREMENT mode:
  // channelRising1 is associated to channelFalling and channelRising3 is associated with channelRising4
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

  MyTim = new HardwareTimer(TIM2);
  Serial.println(">>> created new timer");

  // Configure rising edge detection to measure frequency
  MyTim->setMode(channelRising, TIMER_INPUT_FREQ_DUTY_MEASUREMENT, pin);
  Serial.println(">>> set timer mode");

  // With a PrescalerFactor = 1, the minimum frequency value to measure is : TIM counter clock / CCR MAX
  //  = (SystemCoreClock) / 65535
  // Example on Nucleo_L476RG with systemClock at 80MHz, the minimum frequency is around 1,2 khz
  // To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision.
  // The maximum frequency depends on processing of both interruptions and thus depend on board used
  // Example on Nucleo_L476RG with systemClock at 80MHz the interruptions processing is around 10 microseconds and thus Max frequency is around 100kHz
  uint32_t PrescalerFactor = 1;
  MyTim->setPrescaleFactor(PrescalerFactor);
  MyTim->setOverflow(0x10000); // Max Period value to have the largest possible time to detect rising edge and avoid timer rollover
  Serial.println(">>> set prescale and overflow");

  MyTim->attachInterrupt(channelRising, signal_rising_callback);
  MyTim->attachInterrupt(channelFalling, signal_falling_callback);
  MyTim->attachInterrupt(overflow_callback);

  Serial.println(">>> attached interrupts");
  MyTim->resume();

  Serial.println(">>> resumed timer");

  // Compute this scale factor only once
  input_freq = MyTim->getTimerClkFreq() / MyTim->getPrescaleFactor();

}


#endif
