/*
 * app_comm.h —— 通信任务（3 路串口指令接收）与距离上报任务
 */
#ifndef __APP_COMM_H
#define __APP_COMM_H

#include "main.h"

/**
 * @brief  启动 USART1/2/3 的接收中断
 * @note   必须在 RTOS 队列创建完成之后调用，因为接收中断会往指令队列里写数据
 */
void App_CommStartReceive(void);

/**
 * @brief  通信任务：从指令队列取字节，校验后转发给控制任务
 */
void App_CommTask(void *argument);

/**
 * @brief  上报任务：周期性把最新距离输出到 USART1
 */
void App_TelemetryTask(void *argument);

/**
 * @brief  判断指令码是否合法
 * @retval 1 合法，0 非法
 */
uint8_t App_CommIsValidCommand(uint8_t cmd);

#endif /* __APP_COMM_H */
