/*
 * app_rtos.c —— FreeRTOS 对象的创建、任务启动以及控制台输出
 *
 * 说明：控制台（USART1）的发送不再使用 HAL_UART_Transmit()，原因是该函数会
 *       长时间占用 huart->Lock，而中断回调里重新挂载接收要用 HAL_UART_Receive_IT()，
 *       一旦发送期间有数据到达，重挂载会因 HAL_BUSY 失败，导致该串口再也收不到数据
 *       （裸机版本中“要按两次停止才停”的现象就与它有关）。
 *       这里改成直接轮询 USART 的 TXE 标志发送，不碰 HAL 的锁，
 *       从根本上避免“发送把接收堵死”的问题。
 */
#include "app_rtos.h"
#include "app_config.h"
#include "app_control.h"
#include "app_sensor.h"
#include "app_comm.h"
#include "motor.h"
#include "tim.h"
#include <stdio.h>
#include <string.h>

/*---------------------------------------------------------------------------
 * 全局 RTOS 对象句柄
 *-------------------------------------------------------------------------*/
QueueHandle_t     g_cmdQueue  = NULL; /* 原始指令字节：串口中断 -> 通信任务 */
QueueHandle_t     g_ctrlQueue = NULL; /* 已校验指令  ：通信任务 -> 控制任务 */
QueueHandle_t     g_distQueue = NULL; /* 距离信箱    ：测距任务 -> 控制/上报任务 */
SemaphoreHandle_t g_echoSem   = NULL; /* 回波完成信号：TIM4 中断 -> 测距任务 */

static SemaphoreHandle_t s_consoleMutex = NULL;

/* 控制台串口：与原工程 printf 重定向一致，使用 USART1 */
#define CONSOLE_UART    USART1

/*---------------------------------------------------------------------------
 * 控制台输出
 *-------------------------------------------------------------------------*/
/**
 * @brief  向控制台（USART1）发送一段数据，整串加锁，可被多个任务调用
 * @note   采用寄存器级轮询发送，不占用 HAL 的 huart->Lock，避免影响串口接收
 */
void App_ConsoleSend(const char *text, uint16_t len)
{
    uint16_t i;
    BaseType_t locked = pdFALSE;

    if ((s_consoleMutex != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
    {
        if (xSemaphoreTake(s_consoleMutex, portMAX_DELAY) == pdTRUE)
        {
            locked = pdTRUE;
        }
    }

    for (i = 0; i < len; i++)
    {
        while ((CONSOLE_UART->SR & USART_SR_TXE) == 0U) /* 等待发送数据寄存器空 */
        {
        }
        CONSOLE_UART->DR = (uint8_t)text[i];
    }
    while ((CONSOLE_UART->SR & USART_SR_TC) == 0U) /* 等待最后一个字节发送完成 */
    {
    }

    if (locked == pdTRUE)
    {
        xSemaphoreGive(s_consoleMutex);
    }
}

/**
 * @brief  输出一行文本
 */
void App_LogText(const char *text)
{
    App_ConsoleSend(text, (uint16_t)strlen(text));
}

/**
 * @brief  输出一行距离数据，格式与原工程 printf("%d\r\n", Distance) 一致
 */
void App_LogDistance(uint32_t distance_cm)
{
    char buf[16];
    int len = sprintf(buf, "%lu\r\n", (unsigned long)distance_cm);

    if (len > 0)
    {
        App_ConsoleSend(buf, (uint16_t)len);
    }
}

/*---------------------------------------------------------------------------
 * 致命错误处理：先把执行器停下来，再关中断死循环（便于调试器定位）
 * 本函数可能在中断上下文中被调用，因此不能使用任何阻塞或内核 API
 *-------------------------------------------------------------------------*/
static void App_FatalStop(void)
{
    htim3.Instance->CCR1 = 0; /* 左轮停 */
    htim3.Instance->CCR2 = 0; /* 右轮停 */
    htim3.Instance->CCR3 = 0; /* 吸尘器停 */
    htim3.Instance->CCR4 = 0;
    HAL_GPIO_WritePin(Besom_fan_gpio, Besom_pin1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Besom_fan_gpio, Besom_pin2, GPIO_PIN_RESET);

    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}

/**
 * @brief  栈溢出钩子（configCHECK_FOR_STACK_OVERFLOW = 2）
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    App_FatalStop();
}

/**
 * @brief  动态内存分配失败钩子（configUSE_MALLOC_FAILED_HOOK = 1）
 */
void vApplicationMallocFailedHook(void)
{
    App_FatalStop();
}

/*---------------------------------------------------------------------------
 * 系统初始化：创建内核对象并启动全部任务
 *-------------------------------------------------------------------------*/
void App_RtosInit(void)
{
    /* 1. 创建任务间通信对象 */
    g_cmdQueue  = xQueueCreate(8, sizeof(uint8_t));
    g_ctrlQueue = xQueueCreate(8, sizeof(uint8_t));
    g_distQueue = xQueueCreate(1, sizeof(uint32_t)); /* 深度 1：只保留最新距离 */
    g_echoSem   = xSemaphoreCreateBinary();
    s_consoleMutex = xSemaphoreCreateMutex();

    if ((g_cmdQueue == NULL) || (g_ctrlQueue == NULL) || (g_distQueue == NULL) ||
        (g_echoSem == NULL) || (s_consoleMutex == NULL))
    {
        App_FatalStop(); /* 堆内存不足 */
    }

    /* 2. 创建任务（优先级定义见 app_config.h） */
    if (xTaskCreate(App_ControlTask, "control", CONTROL_STACK_WORDS, NULL,
                    PRIO_CONTROL, NULL) != pdPASS)
    {
        App_FatalStop();
    }
    if (xTaskCreate(App_SensorTask, "sensor", SENSOR_STACK_WORDS, NULL,
                    PRIO_SENSOR, NULL) != pdPASS)
    {
        App_FatalStop();
    }
    if (xTaskCreate(App_CommTask, "comm", COMM_STACK_WORDS, NULL,
                    PRIO_COMM, NULL) != pdPASS)
    {
        App_FatalStop();
    }
    if (xTaskCreate(App_TelemetryTask, "report", TELEMETRY_STACK_WORDS, NULL,
                    PRIO_TELEMETRY, NULL) != pdPASS)
    {
        App_FatalStop();
    }

    /* 3. 内核对象就绪后再开串口接收中断，确保中断里可以安全地写队列 */
    App_CommStartReceive();
}
