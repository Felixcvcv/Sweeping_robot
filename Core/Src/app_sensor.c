/*
 * app_sensor.c —— 超声波测距任务（HC-SR04）
 *
 * 硬件连接（与原工程一致）：
 *   PB7  -> HC-SR04 的 Trig，输出 40us 高电平触发一次测距
 *   PB8  -> HC-SR04 的 Echo，接 TIM4_CH3 输入捕获
 *
 * 测距原理：
 *   触发后模块发出 8 个 40kHz 脉冲，Echo 引脚输出一段高电平，其宽度与距离成正比。
 *   TIM4 的分频为 72-1，计数频率 1MHz（1 个计数 = 1us），因此直接读取捕获到的
 *   计数值即可得到回波时间（us），距离 = 回波时间 * 声速 / 2。
 *
 * 与裸机版本的区别：
 *   裸机版本用 TIM2 每 10us 中断一次来累计时间（100kHz 中断，约占 20%~30% CPU），
 *   这里改用 TIM4 捕获寄存器的时间戳直接相减，保留 1us 分辨率且没有中断开销。
 */
#include "app_sensor.h"
#include "app_rtos.h"
#include "app_config.h"
#include "tim.h"

/* HC-SR04 触发引脚 */
#define TRIG_PORT       GPIOB
#define TRIG_PIN        GPIO_PIN_7

/* 回波捕获：TIM4 通道 3 */
#define ECHO_TIM        htim4
#define ECHO_TIM_CH     TIM_CHANNEL_3

#define ECHO_TIMER_FULL (0x10000UL) /* TIM4 为 16 位计数器，用于处理翻转 */

/* 捕获状态机：0 = 等待回波上升沿，1 = 等待回波下降沿 */
static volatile uint8_t  s_capture_state = 0;
static volatile uint32_t s_echo_start = 0;  /* 上升沿时刻，单位 us */
static volatile uint32_t s_echo_us = 0;     /* 回波高电平时间，单位 us */
static volatile uint32_t s_last_distance = DISTANCE_OUT_OF_RANGE;

/**
 * @brief  基于 TIM4 计数的微秒级延时（1 计数 = 1us）
 * @note   带循环上限保护，防止定时器未启动时死等
 */
static void Sensor_DelayUs(uint32_t us)
{
    uint32_t start = (uint32_t)ECHO_TIM.Instance->CNT;
    uint32_t guard = 0;

    while ((uint16_t)((uint32_t)ECHO_TIM.Instance->CNT - start) < us)
    {
        if (++guard > 100000UL)
        {
            break;
        }
    }
}

/**
 * @brief  输出一次 Trig 触发脉冲（>=10us 高电平）
 */
static void Sensor_Trigger(void)
{
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    Sensor_DelayUs(5);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    Sensor_DelayUs(40);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  把捕获状态复位到“等待上升沿”
 * @note   上一次测量如果没有回波（超时），捕获会停在“等待下降沿”的状态，
 *         每次测量前统一复位，避免一次失败之后再也测不到距离
 */
static void Sensor_ArmCapture(void)
{
    __HAL_TIM_SET_CAPTUREPOLARITY(&ECHO_TIM, ECHO_TIM_CH, TIM_INPUTCHANNELPOLARITY_RISING);
    s_capture_state = 0;
}

/**
 * @brief  回波时间换算距离，单位 cm
 * @param  time_us 回波高电平时间，单位 us
 * @note   声速 340m/s = 0.034cm/us，声波往返所以除以 2：距离 = 时间 * 0.017
 */
uint32_t Distance_Calculate(uint32_t time_us)
{
    return (uint32_t)(((float)time_us * 17) / 1000);
}

/**
 * @brief  获取最近一次测距结果（单位 cm）
 */
uint32_t App_SensorGetLastDistance(void)
{
    return s_last_distance;
}

/**
 * @brief  完成一次测距：触发 -> 等待回波信号量 -> 换算距离 -> 写入距离信箱
 */
static void Sensor_Measure(void)
{
    uint32_t distance;

    Sensor_ArmCapture();
    (void)xSemaphoreTake(g_echoSem, 0); /* 清掉上一轮可能残留的信号量 */

    Sensor_Trigger();

    if (xSemaphoreTake(g_echoSem, pdMS_TO_TICKS(SENSOR_ECHO_TIMEOUT_MS)) == pdTRUE)
    {
        distance = Distance_Calculate(s_echo_us);
    }
    else
    {
        distance = DISTANCE_OUT_OF_RANGE; /* 超时：前方无遮挡或超出量程 */
    }

    s_last_distance = distance;
    (void)xQueueOverwrite(g_distQueue, &distance); /* 信箱只保留最新值 */
}

/**
 * @brief  测距任务：周期 100ms（HC-SR04 建议测量间隔 >= 60ms）
 */
void App_SensorTask(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();

    (void)argument;

    for (;;)
    {
        Sensor_Measure();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}

/**
 * @brief  定时器输入捕获回调（TIM4_CH3 中断）
 * @note   上升沿记录回波起点，下降沿得到回波宽度并唤醒测距任务；
 *         本函数运行在中断中，通过 portYIELD_FROM_ISR 请求任务切换
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t captured;

    if (htim->Instance != TIM4)
    {
        return;
    }

    captured = HAL_TIM_ReadCapturedValue(htim, ECHO_TIM_CH);

    if (s_capture_state == 0) /* 捕获到上升沿：回波开始 */
    {
        s_echo_start = captured;
        s_capture_state = 1;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, ECHO_TIM_CH, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else /* 捕获到下降沿：回波结束，一次测距完成 */
    {
        if (captured >= s_echo_start)
        {
            s_echo_us = captured - s_echo_start;
        }
        else
        {
            /* 回波最宽约 38ms，小于 TIM4 的 65.535ms 计数周期，最多翻转一次 */
            s_echo_us = (ECHO_TIMER_FULL - s_echo_start) + captured;
        }

        s_capture_state = 0;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, ECHO_TIM_CH, TIM_INPUTCHANNELPOLARITY_RISING);

        xSemaphoreGiveFromISR(g_echoSem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
