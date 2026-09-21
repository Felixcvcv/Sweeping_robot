/*
 * app_control.h —— 运动控制任务（电机、自动模式状态机）
 */
#ifndef __APP_CONTROL_H
#define __APP_CONTROL_H

#include "main.h"

/* 机器人工作模式 */
typedef enum
{
    MODE_MANUAL = 0,   /* 手动：由蓝牙/语音/触摸屏指令直接控制 */
    MODE_AUTO_FORWARD, /* 自动：中速前进并持续避障 */
    MODE_AUTO_BACKUP,  /* 自动：检测到障碍，快速后退中 */
    MODE_AUTO_TURN     /* 自动：后退结束，左转转向中 */
} RobotMode_t;

/**
 * @brief  控制任务：解析指令队列、运行自动模式状态机
 * @note   所有电机动作（PWM / 方向 / 吸尘器）都只在本任务中执行，
 *         这样执行器只有一个“主人”，不会出现两个任务同时改电机状态的问题
 */
void App_ControlTask(void *argument);

/**
 * @brief  获取当前工作模式
 */
RobotMode_t App_GetMode(void);

/**
 * @brief  获取当前模式名称（调试用）
 */
const char *App_GetModeName(void);

#endif /* __APP_CONTROL_H */
