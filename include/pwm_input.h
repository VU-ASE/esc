#ifndef PWM_INPUT_H
#define PWM_INPUT_H

#include <SimpleFOC.h>
#include "HardwareTimer.h"

// Enable debugging over serial - set this to false when flashing for production.
#define MONITOR false
#define PWM_PIN A_PWM

// Global variables
HardwareTimer *MyTim2 = nullptr;
volatile uint32_t pulse_width = 0;
volatile uint32_t t_rising = 0;
volatile uint32_t t_falling = 0;

// Interrupt handler
void onCaptureCallback()
{
  t_rising = MyTim2->getCaptureCompare(1); // Rising edge time
  t_falling = MyTim2->getCaptureCompare(2); // Falling edge time

  #if MONITOR
  // Serial.printf("Rising: %lu, Falling: %lu\n", t_rising, t_falling);
  #endif

  if (t_falling >= t_rising)
    pulse_width = t_falling - t_rising;
  else
    pulse_width = 0xFFFF - t_rising + t_falling;

  #if MONITOR
  // Serial.printf("Pulse width: %lu us\n", pulse_width);
  #endif
}

void pwm_input_init()
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();

  // Manually configure PA15 for TIM2_CH1
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2; // TIM2 Alternate Function
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Now configure TIM2 for PWM Input mode
  MyTim2 = new HardwareTimer(TIM2);

  MyTim2->setPrescaleFactor(50); // 1MHz timer tick
  MyTim2->setOverflow(0xFFFF);   // Full 16-bit timer

  // HAL-level low config
  TIM_HandleTypeDef *h = MyTim2->getHandle();
  
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};

  // Input capture on Channel 1, rising edge
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(h, &sConfigIC, TIM_CHANNEL_1);

  // Input capture on Channel 2, falling edge, indirect TI
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(h, &sConfigIC, TIM_CHANNEL_2);

  // PWM Input mode (this is the magic!)
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  HAL_TIM_SlaveConfigSynchro(h, &sSlaveConfig);

  // Start input capture on both channels
  HAL_TIM_IC_Start_IT(h, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(h, TIM_CHANNEL_2);

  // Attach interrupt only to CH1
  MyTim2->attachInterrupt(1, onCaptureCallback);

  MyTim2->resume();

  #ifdef MONITOR
  Serial.println("TIM2 configured in PWM Input Mode!");
  #endif
}



#endif
