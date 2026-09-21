
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "motor.h"
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
char RxBuff1[1], RxBuff2[1], RxBuff3[1], DataBuff[8];
uint8_t auto_flag, flag_time = 0, wave_flag = 0;
uint32_t Distance;                          // 距离
uint32_t HalTime1, HalTime2;                // 临时时间变量
extern volatile uint32_t TimeCounter, time; // 时间计数，单位10us
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

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
void SystemClock_Config(void);
uint32_t Distance_Calculate(uint32_t count);
void Delay_us(unsigned long i);
void sent(void);
/* USER CODE BEGIN PFP */
int16_t aaa=0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart2, (uint8_t *)"OK", sizeof("OK") - 1, 100); // 蓝牙模块开机测试
  printf("\r\n");                                                     // 蓝牙模块开机测试

  HAL_UART_Receive_IT(&huart1, (uint8_t *)RxBuff1, 1); // 串口中断
  HAL_UART_Receive_IT(&huart2, (uint8_t *)RxBuff1, 1); // 串口中断 
  HAL_UART_Receive_IT(&huart3, (uint8_t *)RxBuff1, 1); // 串口中断
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);// 扫把部分

  Motor_SetSpeed(0, 0);
  Motor_SetSpeed(1, 0);

  HAL_TIM_Base_Start_IT(&htim2);              // 计时
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3); // 输入捕获

  /* USER CODE END 2 */
  // besom_run();
//  Motor_SetDirection(1);
//  medium();
//  HAL_Delay(3000);
//  Motor_SetDirection(2);
//  HAL_Delay(3000);
//  besom_run();
//  stop();
//  HAL_Delay(3000);
//  besom_stop();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) // set dir and then speed
  {
//    medium();// test
//		if(aaa==1){
//			HAL_UART_Transmit(&huart2, (uint8_t *)"aaa", sizeof("aaa") - 1, 100);
//		}
    sent();
    Distance = Distance_Calculate(time);
    printf("%d\r\n", Distance);
    HAL_Delay(1000);
    if (auto_flag == 1) // 自动模式，//!可能在delay中进入中断
    {
      besom_run();
      besom_run();
      sent();
      medium();
      Distance = Distance_Calculate(time);
      printf("%d\r\n", Distance);
      /* USER CODE END WHILE */
      if ((flag_time == 0) && (wave_flag == 1)) // 此时收到下降沿
      {
        if (Distance <= 25)
        {
          Motor_SetDirection(2);
          quickly();
          HAL_Delay(3000);
          Motor_SetDirection(3); // 左
          left();
          HAL_Delay(5000);
        }
      }
    }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/// @brief 当超出测量范围时，传入时间为33ms时，计算得到距离约为561cm，对应测量距离最大值5.6m
/// @param count 时间计数值（单位10us）
/// @return
uint32_t Distance_Calculate(uint32_t count)
{
  uint32_t Distance = 0;
  Distance = (uint32_t)(((float)count * 17) / 100); // 距离单位cm，声速340M/s,时间*速度/2=距离
  return Distance;
}

// 等级us级别
void Delay_us(unsigned long i)
{
  unsigned long j;
  for (; i > 0; i--)
  {
    for (j = 5; j > 0; j--)
      ;
  }
}

void sent(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); //!修改
  HAL_Delay(5);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); //!修改
  Delay_us(40);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); //!修改
  Delay_us(40);
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
