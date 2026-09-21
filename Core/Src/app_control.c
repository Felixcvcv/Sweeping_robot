/*
 * app_control.c —— 运动控制任务
 *
 * 职责：
 *   1. 接收通信任务转发过来的指令，执行对应的电机动作（手动模式）；
 *   2. 运行自动模式（超声波避障）状态机；
 *   3. 作为唯一的执行器任务，所有电机/吸尘器操作都在这里发生。
 *
 * 自动模式流程（与原裸机版本一致，但改用“时间戳 + 状态机”实现，不再阻塞延时）：
 *   MODE_AUTO_FORWARD 中速前进
 *        └─ 距离 <= 25cm ──> MODE_AUTO_BACKUP 快速后退 3s
 *                              └─ 3s 到 ──> MODE_AUTO_TURN 左转 5s
 *                                            └─ 5s 到 ──> 回到 MODE_AUTO_FORWARD
 *
 * 裸机版本在自动模式里用 HAL_Delay(3000)/HAL_Delay(5000) 阻塞等待，这期间
 * 主循环完全停摆（连停止指令都无法响应）。改成状态机后，等待期间控制任务
 * 依然在 20ms 周期地跑，随时可以响应新的指令。
 */
#include "app_control.h"
#include "app_rtos.h"
#include "app_config.h"
#include "motor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

static RobotMode_t s_mode = MODE_MANUAL; /* 当前工作模式 */
static TickType_t s_phase_start = 0;     /* 自动模式当前阶段开始时刻 */

/**
 * @brief  执行一条手动指令
 * @note   除自动模式指令外，任何指令都会让机器人退出自动模式（与原工程一致）
 */
static void Control_ApplyCommand(uint8_t cmd)
{
    switch (cmd)
    {
    case CMD_STOP: /* 停止：退出自动模式，关电机与吸尘器 */
        s_mode = MODE_MANUAL;
        stop();
        besom_stop();
        break;

    case CMD_FORWARD:
        s_mode = MODE_MANUAL;
        Motor_SetDirection(1);
        break;

    case CMD_BACKWARD:
        s_mode = MODE_MANUAL;
        Motor_SetDirection(2);
        break;

    case CMD_LEFT: /* 左转：差速转向，先设方向再设速度 */
        s_mode = MODE_MANUAL;
        Motor_SetDirection(3);
        left();
        break;

    case CMD_RIGHT:
        s_mode = MODE_MANUAL;
        Motor_SetDirection(4);
        right();
        break;

    case CMD_FAST:
        s_mode = MODE_MANUAL;
        quickly();
        break;

    case CMD_MEDIUM:
        s_mode = MODE_MANUAL;
        medium();
        break;

    case CMD_SLOW:
        s_mode = MODE_MANUAL;
        slow();
        break;

    case CMD_VACUUM_ON: /* 吸尘器（扫把/风扇）单独启动 */
        s_mode = MODE_MANUAL;
        besom_run();
        break;

    case CMD_AUTO_MODE: /* 自动模式：开吸尘器 + 中速前进，进入避障状态机 */
        s_mode = MODE_AUTO_FORWARD;
        s_phase_start = xTaskGetTickCount();
        besom_run();
        Motor_SetDirection(1);
        medium();
        break;

    default:
        break; /* 未知指令直接忽略 */
    }
}

/**
 * @brief  自动模式状态机，每 CONTROL_PERIOD_MS 调用一次
 */
static void Control_UpdateAutoMode(void)
{
    uint32_t distance;
    TickType_t now = xTaskGetTickCount();

    if (s_mode == MODE_MANUAL)
    {
        return;
    }

    if (s_mode == MODE_AUTO_FORWARD)
    {
        /* 读取测距信箱里的最新距离（只读取不取走，信箱始终保留最新值） */
        if (xQueuePeek(g_distQueue, &distance, 0) == pdTRUE)
        {
            if (distance <= OBSTACLE_DISTANCE_CM)
            {
                /* 前方有障碍：快速后退 */
                Motor_SetDirection(2);
                quickly();
                s_mode = MODE_AUTO_BACKUP;
                s_phase_start = now;
            }
        }
    }
    else if (s_mode == MODE_AUTO_BACKUP)
    {
        if ((now - s_phase_start) >= pdMS_TO_TICKS(AVOID_BACKWARD_MS))
        {
            /* 后退结束：左转转向 */
            Motor_SetDirection(3);
            left();
            s_mode = MODE_AUTO_TURN;
            s_phase_start = now;
        }
    }
    else /* MODE_AUTO_TURN */
    {
        if ((now - s_phase_start) >= pdMS_TO_TICKS(AVOID_TURN_MS))
        {
            /* 转向结束：恢复中速前进，继续避障 */
            Motor_SetDirection(1);
            medium();
            s_mode = MODE_AUTO_FORWARD;
            s_phase_start = now;
        }
    }
}

/**
 * @brief  控制任务：20ms 周期，先清空指令队列再跑状态机
 */
void App_ControlTask(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();
    uint8_t cmd;

    (void)argument;

    for (;;)
    {
        /* 一次把所有等待中的指令都处理完（队列元素不会堆积） */
        while (xQueueReceive(g_ctrlQueue, &cmd, 0) == pdTRUE)
        {
            Control_ApplyCommand(cmd);
        }

        Control_UpdateAutoMode();

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

/**
 * @brief  获取当前工作模式
 */
RobotMode_t App_GetMode(void)
{
    return s_mode;
}

/**
 * @brief  获取当前模式名称
 */
const char *App_GetModeName(void)
{
    switch (s_mode)
    {
    case MODE_AUTO_FORWARD:
        return "auto-forward";
    case MODE_AUTO_BACKUP:
        return "auto-backup";
    case MODE_AUTO_TURN:
        return "auto-turn";
    case MODE_MANUAL:
    default:
        return "manual";
    }
}
