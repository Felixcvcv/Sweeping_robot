#include "motor.h"
#include "tim.h"
#include "math.h"

/**
 * 函    数：直流电机0/1设置速度
 * 参    数：Speed 要设置的速度，范围：0~100，传入的ch为0/1(0是左边的轮子,1是右边的轮子)
 * 返 回 值：无
 */
void Motor_SetSpeed(uint8_t ch, int8_t Speed)
{
    if (Speed >= 0)
    {
        if (ch == 0)
        {
            HAL_GPIO_WritePin(motor_gpio, motor_pin1_1, GPIO_PIN_SET);   // pin1置高电平
            HAL_GPIO_WritePin(motor_gpio, motor_pin1_2, GPIO_PIN_RESET); // pin2置低电平，设置方向为正转
            TIM3->CCR1 = Speed;                                          // PWM设置为速度值
        }
        else
        {
            HAL_GPIO_WritePin(motor_gpio, motor_pin2_1, GPIO_PIN_SET);   // pin1置高电平
            HAL_GPIO_WritePin(motor_gpio, motor_pin2_2, GPIO_PIN_RESET); // pin2置低电平，设置方向为正转
            TIM3->CCR2 = Speed;                                          // PWM设置为速度值
        }
    }
    else
    {
        if (ch == 0)
        {
            HAL_GPIO_WritePin(motor_gpio, motor_pin1_1, GPIO_PIN_RESET); // pin1置高电平
            HAL_GPIO_WritePin(motor_gpio, motor_pin1_2, GPIO_PIN_SET);   // pin2置低电平，设置方向为正转
            TIM3->CCR1 = -Speed;                                         // PWM设置为速度值
        }
        else
        {
            HAL_GPIO_WritePin(motor_gpio, motor_pin2_1, GPIO_PIN_RESET); // pin1置高电平
            HAL_GPIO_WritePin(motor_gpio, motor_pin2_2, GPIO_PIN_SET);   // pin2置低电平，设置方向为正转
            TIM3->CCR2 = -Speed;                                         // PWM设置为速度值
        }
    }
}

uint8_t speed_now0, speed_now1;

// void stop(void)
//{
//     Motor_SetSpeed(1, 0);
//     Motor_SetSpeed(0, 0);
// }

// void quickly(void)
//{
//     Motor_SetSpeed(1, 90);
//     Motor_SetSpeed(0, 90);
// }

// void slow(void)
//{
//     Motor_SetSpeed(1, 40);
//     Motor_SetSpeed(0, 40);
// }

// void medium(void)
//{
//     Motor_SetSpeed(1, 70);
//     Motor_SetSpeed(0, 70);
// }

void stop(void)
{
    speed_now0 = 0;
    speed_now1 = 0;
    TIM3->CCR1 = speed_now0;
    TIM3->CCR2 = speed_now1;
}

void quickly(void)
{
    speed_now0 = 100;
    speed_now1 = 100;
    TIM3->CCR1 = speed_now0;
    TIM3->CCR2 = speed_now1;
}

void slow(void)
{
    speed_now0 = 60;
    speed_now1 = 60;
    TIM3->CCR1 = speed_now0;
    TIM3->CCR2 = speed_now1;
}

void medium(void)
{
    speed_now0 = 80;
    speed_now1 = 80;
    TIM3->CCR1 = speed_now0;
    TIM3->CCR2 = speed_now1;
}

void left(void)
{
    speed_now0 = 40;
    speed_now1 = 100;  //!
    TIM3->CCR1 = speed_now0;
    TIM3->CCR2 = speed_now1;
}

void right(void)
{
    speed_now0 = 100;
    speed_now1 = 40;
    TIM3->CCR1 = speed_now0;
    TIM3->CCR2 = speed_now1;
}
/**
 * 直流电机设置方向,设置方向后要设置速度
 * ch:1-4对应四个电机，dir：1前 2后 3左 4右
 * motor_gpio：左边电机 Besom_fan_gpio：右边电机
 */
void Motor_SetDirection(uint8_t dir)
{
    switch (dir)
    {
    case 1:
        HAL_GPIO_WritePin(motor_gpio, motor_pin1_1, GPIO_PIN_SET);   // pin1置低电平
        HAL_GPIO_WritePin(motor_gpio, motor_pin1_2, GPIO_PIN_RESET); // pin2置低电平，电机停止
        HAL_GPIO_WritePin(motor_gpio, motor_pin2_1, GPIO_PIN_SET);   // pin1置低电平
        HAL_GPIO_WritePin(motor_gpio, motor_pin2_2, GPIO_PIN_RESET); // pin2置低电平，电机停止
        break;
    case 2:
        HAL_GPIO_WritePin(motor_gpio, motor_pin1_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_gpio, motor_pin1_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_gpio, motor_pin2_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_gpio, motor_pin2_2, GPIO_PIN_SET);
        break;
    case 3:
        HAL_GPIO_WritePin(motor_gpio, motor_pin1_1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_gpio, motor_pin1_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_gpio, motor_pin2_1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_gpio, motor_pin2_2, GPIO_PIN_RESET); // 左转：右轮前进 左轮不动
        break;
    default:
        HAL_GPIO_WritePin(motor_gpio, motor_pin1_1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_gpio, motor_pin1_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_gpio, motor_pin2_1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_gpio, motor_pin2_2, GPIO_PIN_RESET); // 右转：左轮前进 右轮后退
        break;
    }
}

void besom_run(void)
{
    HAL_GPIO_WritePin(Besom_fan_gpio, Besom_pin1, GPIO_PIN_SET);   // pin1置高电平
    HAL_GPIO_WritePin(Besom_fan_gpio, Besom_pin2, GPIO_PIN_RESET); // pin2置低电平，设置方向为正转
    TIM3->CCR3 = 100;                                               // PWM设置为速度值
}

void besom_stop(void)
{
    HAL_GPIO_WritePin(Besom_fan_gpio, Besom_pin1, GPIO_PIN_RESET); // pin1置高电平
    HAL_GPIO_WritePin(Besom_fan_gpio, Besom_pin2, GPIO_PIN_RESET); // pin2置低电平，设置方向为正转
}

// void fan_run(void)
// {
//     HAL_GPIO_WritePin(Besom_fan_gpio, fan_pin1, GPIO_PIN_SET);   // pin1置高电平
//     HAL_GPIO_WritePin(Besom_fan_gpio, fan_pin2, GPIO_PIN_RESET); // pin2置低电平，设置方向为正转
//     TIM3->CCR4 = 90;                                             // PWM设置为速度值
// }

// void fan_stop(void)
// {
//     HAL_GPIO_WritePin(Besom_fan_gpio, fan_pin1, GPIO_PIN_RESET); // pin1置高电平
//     HAL_GPIO_WritePin(Besom_fan_gpio, fan_pin2, GPIO_PIN_RESET); // pin2置低电平，设置方向为正转
// }


