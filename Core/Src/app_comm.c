/*
 * app_comm.c —— 通信任务与距离上报任务
 *
 * 三路串口分工（与原工程一致，仅波特率沿用 9600）：
 *   USART1 (PA9/PA10)  ：HC-08 蓝牙模块
 *   USART2 (PA2/PA3)   ：SU-03T 离线语音模块
 *   USART3 (PB10/PB11) ：串口触摸屏
 *
 * 数据流（中断只做“搬运”，不在中断里操作电机）：
 *   串口接收中断 --> g_cmdQueue --> 通信任务校验 --> g_ctrlQueue --> 控制任务执行
 *
 * 这里修复了裸机版本的两个问题：
 *   1. 三路串口共用同一个 RxBuff1，且任意一路收到数据都会把三路的中断重新挂载一遍，
 *      导致丢字节/重复触发（现象就是“有时候要按两次停止才停”）；
 *      现在每路串口各有独立缓冲，且只重新挂载自己这一路。
 *   2. 裸机版本直接在串口中断里调用电机函数，中断里做了大量 GPIO 操作；
 *      现在中断只投递一个字节，动作由任务完成。
 */
#include "app_comm.h"
#include "app_rtos.h"
#include "app_config.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define COMM_UART_COUNT 3

/* 每路串口一个独立的接收缓冲 */
static uint8_t s_rx_byte[COMM_UART_COUNT];
static UART_HandleTypeDef *const s_uart[COMM_UART_COUNT] = {&huart1, &huart2, &huart3};

/**
 * @brief  由串口句柄查找对应的串口序号
 */
static int8_t Comm_UartIndex(UART_HandleTypeDef *huart)
{
    uint8_t i;

    for (i = 0; i < COMM_UART_COUNT; i++)
    {
        if (s_uart[i] == huart)
        {
            return (int8_t)i;
        }
    }
    return -1;
}

/**
 * @brief  判断指令码是否合法（防止串口噪声字节占满指令队列）
 */
uint8_t App_CommIsValidCommand(uint8_t cmd)
{
    switch (cmd)
    {
    case CMD_STOP:
    case CMD_FORWARD:
    case CMD_BACKWARD:
    case CMD_LEFT:
    case CMD_RIGHT:
    case CMD_FAST:
    case CMD_MEDIUM:
    case CMD_SLOW:
    case CMD_VACUUM_ON:
    case CMD_AUTO_MODE:
        return 1;
    default:
        return 0;
    }
}

/**
 * @brief  启动三路串口的中断接收，每次收 1 个字节
 */
void App_CommStartReceive(void)
{
    uint8_t i;

    for (i = 0; i < COMM_UART_COUNT; i++)
    {
        (void)HAL_UART_Receive_IT(s_uart[i], &s_rx_byte[i], 1);
    }
}

/**
 * @brief  串口接收完成回调（中断上下文）
 * @note   只做两件事：把收到的字节投递到指令队列、重新挂载本路接收。
 *         注意：控制台发送不使用 HAL 的阻塞发送，所以这里重新挂载接收时
 *         不会因为 huart->Lock 被占用而失败（详见 app_rtos.c 的说明）。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    int8_t index = Comm_UartIndex(huart);

    if (index < 0)
    {
        return;
    }

    (void)xQueueSendFromISR(g_cmdQueue, &s_rx_byte[index], &xHigherPriorityTaskWoken);

    /* 重新挂载“这一路”串口的接收，准备接收下一个字节 */
    (void)HAL_UART_Receive_IT(huart, &s_rx_byte[index], 1);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief  串口错误回调（过载 ORE / 帧错误 FE / 噪声 NE 等，中断上下文）
 * @note   电机是大干扰源，串口收到噪声字节时 HAL 会中止本次接收。
 *         这里把接收重新挂上，否则这一路串口会从此收不到数据；
 *         若 HAL 内部仍处于接收中（BUSY），Receive_IT 返回 HAL_BUSY，不会有副作用。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    int8_t index = Comm_UartIndex(huart);

    if (index < 0)
    {
        return;
    }

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    (void)HAL_UART_Receive_IT(huart, &s_rx_byte[index], 1);
}

/**
 * @brief  通信任务：阻塞等待指令字节，校验后转发给控制任务
 */
void App_CommTask(void *argument){
    uint8_t cmd;

    (void)argument;

    for (;;)
    {
        if (xQueueReceive(g_cmdQueue, &cmd, portMAX_DELAY) == pdTRUE)
        {
            if (App_CommIsValidCommand(cmd) != 0)
            {
                /* 转发给控制任务；最多等 100ms（5 个控制周期），
                   避免控制任务异常时通信任务被永久阻塞 */
                if (xQueueSend(g_ctrlQueue, &cmd, pdMS_TO_TICKS(100)) != pdPASS)
                {
                    /* 队列满：丢弃本条指令 */
                }
#if (APP_LOG_COMMANDS == 1)
                App_LogText("cmd:");
                App_LogDistance(cmd);
#endif
            }
        }
    }
}

/**
 * @brief  上报任务：周期 1s 输出最新距离（与原工程 printf 输出位置、格式一致）
 */
void App_TelemetryTask(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t distance;

    (void)argument;

    for (;;)
    {
        if (xQueuePeek(g_distQueue, &distance, 0) == pdTRUE)
        {
            App_LogDistance(distance);
        }
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}
