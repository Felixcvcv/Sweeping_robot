/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f1xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "motor.h"
#include <stdio.h>
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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2; //!修改
extern UART_HandleTypeDef huart3; //!修改
extern int16_t aaa;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
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
 * @brief This function handles Prefetch fault, memory access fault.
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
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/
volatile uint32_t TimeCounter, time=0;
extern uint8_t wave_flag;
/**
 * @brief This function handles TIM2 global interrupt.
 */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
  TimeCounter++;
  /* USER CODE END TIM2_IRQn 1 */
}

/**
 * @brief This function handles TIM4 global interrupt.
 */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  //  printf("3");
  if (TIM4 == htim->Instance)
  {
    if (flag_time == 0) // 标志捕获到上升沿
    {
      __HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING); // 这里使用的是通道3
      TimeCounter = 0;                                                                        // 先清零TIM4计时
      flag_time = 1;                                                                          // 下次进入下降沿
    }
    else // 标志捕获到下降沿
    {
      wave_flag = 1; // 区分第一次flag_time=0
      __HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
      flag_time = 0;      // 标志捕获到下降沿，下次进入上升沿
      time = TimeCounter; // 记录此时高电平时间（单位为10us）
      TimeCounter = 0;    // 清零此时的时间
    }
  }
}
/* USER CODE BEGIN 1 */
/* USER CODE BEGIN 1 */
extern char RxBuff1[], DataBuff[8];
extern uint8_t auto_flag;
void USART1_IRQHandler(void) // 清除必要标志位
{

  HAL_UART_IRQHandler(&huart1);
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void) //!修改
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void) //!修改
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/// @brief 串口回调函数，判断接收到的数据
/// @param huart
///  前0x11   后0x22   左0x33   右0x44   停止0xAA  快速0x55  中等0x66  慢速0x77
///  扫把0x89  风扇0x99
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET); // 测试
  // if ((DataBuff[0] == 0x33) || (DataBuff[0] == 0x44))
  //   Motor_SetDirection(1);
  switch (RxBuff1[0])
  {
  case 0x01:
    auto_flag = 0;
    wave_flag = 0;
    stop();
    besom_stop();
    // fan_stop();
    break;
  case 0x02:
    Motor_SetDirection(1);
    auto_flag = 0;
    wave_flag = 0;
//	  aaa=1;
    break;
  case 0x03:
    Motor_SetDirection(2);
    auto_flag = 0;
    wave_flag = 0;
    break;
  case 0x04:
    Motor_SetDirection(3);
    left();
    auto_flag = 0;
    wave_flag = 0;
    break;
  case 0x05:
    Motor_SetDirection(4);
    right();
    auto_flag = 0;
    wave_flag = 0;
    break;
  case 0x06:
    quickly();
    auto_flag = 0;
    wave_flag = 0;
    break;
  case 0x07:
    medium();
    auto_flag = 0;
    wave_flag = 0;
    break;
  case 0x08:
    slow();
    auto_flag = 0;
    wave_flag = 0;
    break;
  case 0x09:
    besom_run(); // 吸尘器
    auto_flag = 0;
    wave_flag = 0;
    break;
  case 0x10:
    auto_flag = 1; // 设置自动模式
    besom_run();   // 同时打开计时器
    break;
  default:
    break;
  }
  // DataBuff[0] = RxBuff1[0]; // 记录上一次输入的值
  RxBuff1[0] = 0;
  HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuff1, 1); // 把Size重新设置为1
  HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuff1, 1); // 把Size重新设置为1 
  HAL_UART_Receive_IT(&huart3, (uint8_t *)RxBuff1, 1); // 把Size重新设置为1 
}




/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
