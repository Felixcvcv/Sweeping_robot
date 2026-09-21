#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define motor_pin1_1 GPIO_PIN_3
#define motor_pin1_2 GPIO_PIN_4
#define motor_pin2_1 GPIO_PIN_5
#define motor_pin2_2 GPIO_PIN_6
#define motor_gpio GPIOB

#define Besom_pin1 GPIO_PIN_12
#define Besom_pin2 GPIO_PIN_13
// #define fan_pin1 GPIO_PIN_14
// #define fan_pin2 GPIO_PIN_15
#define Besom_fan_gpio GPIOB

    /* 注意：FreeRTOS 版本中电机的所有动作只在控制任务（app_control.c）里执行，
       保证执行器只有一个“主人”；上电初始化时 main() 也可以直接调用。 */
    // 电机部分
    void Motor_SetSpeed(uint8_t ch, int8_t Speed);
    void Motor_SetDirection(uint8_t dir); // dir 1:前进 2：后退 3：左转 4：右转
    void stop(void);                      // 停止
    void quickly(void);                   // 快速
    void slow(void);                      // 慢速
    void medium(void);                    // 中速
    void left(void);                      // 左转
    void right(void);                     // 右转
    // 扫把+风扇
    void besom_run(void);  // 风扇启动
    void besom_stop(void); // 风扇停止
    // void fan_run(void);
    // void fan_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */
