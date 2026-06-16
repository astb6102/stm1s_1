
/**
  ******************************************************************************
  * @file    stm32_mc_common_it.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt
  *          service routine related to Motor Control for the STM32 Family
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup STM32F4xx_IRQ_Handlers
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_config.h"
#include "mc_type.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "motorcontrol.h"
#include "mcp_config.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/** @addtogroup MCSDK
  * @{
  */
/** @addtogroup STM32F4xx_IRQ_Handlers STM32F4xx IRQ Handlers
  * @{
  */

/* USER CODE BEGIN PRIVATE */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTICK_DIVIDER (SYS_TICK_FREQUENCY/1000U)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
bool bdmaActivTc = false;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* USER CODE END PRIVATE */

void HardFault_Handler(void);
void SysTick_Handler(void);

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  */
void HardFault_Handler(void)
{
 /* USER CODE BEGIN HardFault_IRQn 0 */

 /* USER CODE END HardFault_IRQn 0 */

  TSK_HardwareFaultTask();

  /* Go to infinite loop when Hard Fault exception occurs */
  while (true)
  {
	  NVIC_SystemReset();
    /* Nothing to do */
  }

 /* USER CODE BEGIN HardFault_IRQn 1 */

 /* USER CODE END HardFault_IRQn 1 */
}

void SysTick_Handler(void)
{
#ifdef MC_HAL_IS_USED
static uint8_t SystickDividerCounter = SYSTICK_DIVIDER;
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  if (SystickDividerCounter == SYSTICK_DIVIDER)
  {
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
    SystickDividerCounter = 0;
  }
  else
  {
    /* Nothing to do */
  }

  SystickDividerCounter ++;
#endif /* MC_HAL_IS_USED */
  /* Buffer is ready by the HW layer to be processed */
  /* NO DMA interrupt */
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */

    MC_RunMotorControlTasks();

  /* USER CODE BEGIN SysTick_IRQn 2 */

  /* USER CODE END SysTick_IRQn 2 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/

