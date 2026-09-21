/*
 * app_rtos.h —— FreeRTOS 对象（队列 / 信号量）与系统启动接口
 *
 * 任务划分：
 *   sensor   测距任务   ：触发 HC-SR04，读取回波时间，换算距离后写入距离信箱
 *   control  控制任务   ：唯一的“执行器”任务，解析指令队列并运行自动模式状态机
 *   comm     通信任务   ：接收 3 路串口指令，校验后转发给控制任务
 *   report   上报任务   ：周期性把最新距离通过 USART1 输出
 *
 * 任务间通信：
 *   g_cmdQueue   : (ISR -> comm)    串口收到的原始指令字节
 *   g_ctrlQueue  : (comm -> control) 已校验的指令
 *   g_distQueue  : (sensor -> control/report) 长度 1 的信箱，只保留最新距离
 *   g_echoSem    : (TIM4 中断 -> sensor) 回波下降沿完成信号
 */
#ifndef __APP_RTOS_H
#define __APP_RTOS_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* 队列 / 信号量句柄（由 App_RtosInit() 创建） */
extern QueueHandle_t     g_cmdQueue;  /* 原始指令字节，深度 8 */
extern QueueHandle_t     g_ctrlQueue; /* 已校验指令，深度 8 */
extern QueueHandle_t     g_distQueue; /* 距离信箱，深度 1（xQueueOverwrite） */
extern SemaphoreHandle_t g_echoSem;   /* 超声波回波完成信号 */

/* 创建 RTOS 对象与全部任务；必须在 vTaskStartScheduler() 之前调用 */
void App_RtosInit(void);

/* USART1 控制台：整串输出，内部加锁，可在任意任务中调用（不能在中断里调用） */
void App_ConsoleSend(const char *text, uint16_t len);
void App_LogText(const char *text);
void App_LogDistance(uint32_t distance_cm);

#endif /* __APP_RTOS_H */
