/*
 * app_config.h —— 应用层统一配置
 * 指令码、任务参数、避障阈值等集中在这里，便于统一调整
 */
#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

/*---------------------------------------------------------------------------
 * 串口指令码（蓝牙 USART1 / 语音 USART2 / 串口屏 USART3 共用同一套协议）
 * 与原裸机工程 HAL_UART_RxCpltCallback() 中的判断值完全一致
 *-------------------------------------------------------------------------*/
#define CMD_STOP            0x01 /* 停止（同时退出自动模式、关吸尘器） */
#define CMD_FORWARD         0x02 /* 前进 */
#define CMD_BACKWARD        0x03 /* 后退 */
#define CMD_LEFT            0x04 /* 左转 */
#define CMD_RIGHT           0x05 /* 右转 */
#define CMD_FAST            0x06 /* 快速 */
#define CMD_MEDIUM          0x07 /* 中速 */
#define CMD_SLOW            0x08 /* 慢速 */
#define CMD_VACUUM_ON       0x09 /* 打开吸尘器（扫把/风扇） */
#define CMD_AUTO_MODE       0x10 /* 自动模式（自动前进 + 超声波避障） */

/*---------------------------------------------------------------------------
 * 任务优先级（FreeRTOS 中数值越大优先级越高）
 * 控制 > 测距 > 通信 > 上报，configMAX_PRIORITIES 必须大于最大值
 *-------------------------------------------------------------------------*/
#define PRIO_CONTROL        4
#define PRIO_SENSOR         3
#define PRIO_COMM           2
#define PRIO_TELEMETRY      1

/*---------------------------------------------------------------------------
 * 任务周期（ms）与任务栈深度（单位：word，1 word = 4 字节）
 *-------------------------------------------------------------------------*/
#define SENSOR_PERIOD_MS    100  /* 超声波测距周期，HC-SR04 建议 >= 60ms */
#define CONTROL_PERIOD_MS   20   /* 运动控制周期（50Hz 状态机时基） */
#define TELEMETRY_PERIOD_MS 1000 /* 距离上报周期，与裸机版本一致 */

#define SENSOR_STACK_WORDS    128
#define CONTROL_STACK_WORDS   160
#define COMM_STACK_WORDS      192
#define TELEMETRY_STACK_WORDS 192

/*---------------------------------------------------------------------------
 * 自动模式（避障）参数
 *-------------------------------------------------------------------------*/
#define OBSTACLE_DISTANCE_CM   25   /* 障碍物判定阈值（cm） */
#define AVOID_BACKWARD_MS      3000 /* 检测到障碍后快速后退时长 */
#define AVOID_TURN_MS          5000 /* 后退结束后左转时长 */

/*---------------------------------------------------------------------------
 * 超声波参数
 *-------------------------------------------------------------------------*/
#define SENSOR_ECHO_TIMEOUT_MS 60    /* 等待回波超时（HC-SR04 最远回波约 38ms） */
#define DISTANCE_OUT_OF_RANGE  561   /* 无回波时的距离值（对应原工程 33ms 上限） */

/*---------------------------------------------------------------------------
 * 调试开关
 * APP_LOG_COMMANDS = 1 时，通信任务会把收到的指令打印到 USART1，便于联调；
 * 默认关闭，避免与手机 APP 的距离数据流混淆。
 *-------------------------------------------------------------------------*/
#define APP_LOG_COMMANDS       0

#endif /* __APP_CONFIG_H */
