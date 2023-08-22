/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define ADC1_resolution                   (4096 - 1)    // 2^12
#define average_slope                        (2.5f * 0.001f) // mV to V
#define Voltage_at_25C                      0.76f
#define Temperature_at_076V         25
#define ADC1_TempRank                    5
#define Vrefint_Rank                            6
#define period                                         (htim2.Init.Period + 1)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

extern uint16_t ADC1_data[ADC1_NumberOfConversions];
static float ADC1_DataToVoltAverage;
static float Temperature_in_C;

uint16_t V_ref_Internal_Calibration; // global to make it visible for debugger
static uint16_t VREFINT_RawData;
static float VDDA;
static float VREFINT;

extern TIM_HandleTypeDef htim2;
static uint16_t PWM_DutyCycle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  static const uint16_t DMA_IRQHandler_Calls = 100;
  static uint16_t i = 0;
  static const uint16_t  amountOfAllTemperatureSamples = ADC1_TempRank * DMA_IRQHandler_Calls;

  // VREFIN_CAL Raw data acquired at temperature of 30 °C VDDA = 3.3 V 0x1FFF7A2A - 0x1FFF7A2B
  V_ref_Internal_Calibration = *(uint16_t *)0x1FFF7A2A; // uint16_t cuz  Memory addres: 0x1FFF7A2A - 0x1FFF7A2B so we wanna read two bytes (not only one -> uint8_t)
  VREFINT_RawData = ADC1_data[Vrefint_Rank - (Vrefint_Rank - ADC1_TempRank)];
  VDDA = 3.3 * V_ref_Internal_Calibration / (float)VREFINT_RawData; //3.3
  VREFINT = VDDA * VREFINT_RawData / (float)ADC1_resolution;

  while (i < DMA_IRQHandler_Calls)
  {
    i++;
    for (uint8_t j = 0; j < ADC1_TempRank; j++)
    {
      ADC1_DataToVoltAverage = ADC1_DataToVoltAverage + (VDDA * ADC1_data[j] / (float)ADC1_resolution);
    }
    return;
  }
  i = 0;

  ADC1_DataToVoltAverage = ADC1_DataToVoltAverage / (float)amountOfAllTemperatureSamples;
  Temperature_in_C = ((ADC1_DataToVoltAverage - Voltage_at_25C) / average_slope) + Temperature_at_076V;
  ADC1_DataToVoltAverage = 0;

  if (Temperature_in_C >= 33)
  {
    PWM_DutyCycle = period;
  }
  else if (Temperature_in_C >= 32 && Temperature_in_C < 32.8)
  {
    PWM_DutyCycle = period * 0.75;
  }
  else if (Temperature_in_C >= 31 && Temperature_in_C < 31.8)
  {
    PWM_DutyCycle = period * 0.5;
  }
  else if (Temperature_in_C >= 30 && Temperature_in_C < 30.8)
  {
    PWM_DutyCycle = period * 0.35;
  }
  else if (Temperature_in_C < 29.8)
  {
    PWM_DutyCycle = 0;
  }
  TIM2->CCR1 = PWM_DutyCycle; //__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_DutyCycle);

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
