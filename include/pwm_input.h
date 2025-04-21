#ifndef PWM_INPUT_H
#define PWM_INPUT_H

#include <SimpleFOC.h>

#define PWM_PIN A_PWM

TIM_HandleTypeDef htim2;

// uint32_t channel_rising, channel_falling;
// volatile uint32_t frequency_measured, adjusted_duty = 150, last_total_period = 0;
// uint32_t input_freq = 0;
// volatile uint32_t rollover_compare_count = 0;
// HardwareTimer *timer;

// volatile uint32_t first_rise = 0, first_fall = 0, second_rise = 0;

// #define MAGIC_OFFSET 69 + 100

// #define MAX_ALLOWED_DIFFERENCE 100

// void signal_rising_callback(void)
// {
//   first_rise = second_rise;
//   second_rise = timer->getCaptureCompare(channel_rising);

//   // The first and second rise are a sliding window, only when they are both defined in increasing order we have a
//   if (second_rise > first_rise && first_rise != 0 && second_rise != 0 && first_fall > first_rise && first_fall < second_rise) {
//     uint32_t total_period = second_rise - first_rise;
//     uint32_t high_period = first_fall - first_rise;

//     if (total_period > MAX_ALLOWED_DIFFERENCE + last_total_period  || total_period < last_total_period - MAX_ALLOWED_DIFFERENCE) {
//       last_total_period = total_period;
//       return;
//     }

//     last_total_period = total_period;

//     adjusted_duty = (high_period * 1600) / (total_period)-MAGIC_OFFSET;
//   }

//   rollover_compare_count = 0;
// }

// /* In case of timer rollover, frequency is to low to be measured set values to 0
//    To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision. */
// void rollover_callback(void)
// {
//   rollover_compare_count++;

//   if (rollover_compare_count > 1)
//   {
//     // frequency_measured = 0;
//     adjusted_duty = 0;
//     first_rise = 0;
//     second_rise = 0;
//     first_fall = 0;
//   }
// }

// void signal_falling_callback(void)
// {
//   first_fall = timer->getCaptureCompare(channel_falling);
// }

volatile uint32_t high_time = 0;
volatile uint32_t period = 0;

void printTimerInstance(TIM_TypeDef *Instance)
{
  if (Instance == TIM1)
  {
    Serial.println("TIM1");
  }
  else if (Instance == TIM2)
  {
    Serial.println("TIM2");
  }
  else if (Instance == TIM3)
  {
    Serial.println("TIM3");
  }
  else if (Instance == TIM4)
  {
    Serial.println("TIM4");
  }
  else if (Instance == TIM15)
  {
    Serial.println("TIM15");
  }
  else if (Instance == TIM16)
  {
    Serial.println("TIM16");
  }
  else if (Instance == TIM17)
  {
    Serial.println("TIM17");
  }
  else
  {
    Serial.println("Unknown Timer");
  }
}

void pwm_input_init()
{
  // The PWM pin on this board should behave as input
  pinMode(PWM_PIN, INPUT);

  // Enable clocks for TIM2 and GPIOA
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // Configure PA15 as TIM2_CH1 (AF1)
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configure TIM2
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 50 - 1; // 1 MHz timer if APB1 = 72MHz
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF; // Max period for 16-bit timer
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_IC_Init(&htim2);

  TIM_IC_InitTypeDef sConfigIC = {0};

  // CH1: Rising edge capture (Direct TI)
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1);

  // CH2: Falling edge capture (Indirect TI)
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2);

  // Start input capture
  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2);

  Serial.println("Configured timer!");
}

#endif
