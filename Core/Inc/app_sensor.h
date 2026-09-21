/*
 * app_sensor.h —— 超声波测距（HC-SR04）
 */
#ifndef __APP_SENSOR_H
#define __APP_SENSOR_H

#include "main.h"

/**
 * @brief  测距任务：周期触发超声波并更新距离信箱（g_distQueue）
 * @param  argument 未使用
 */
void App_SensorTask(void *argument);

/**
 * @brief  获取最近一次测距结果
 * @retval 距离，单位 cm；超出量程时返回 DISTANCE_OUT_OF_RANGE
 */
uint32_t App_SensorGetLastDistance(void);

/**
 * @brief  由回波时间（单位 10us）换算距离，单位 cm
 * @note   声速 340m/s：距离 = 时间 * 17 / 100
 */
uint32_t Distance_Calculate(uint32_t count);

#endif /* __APP_SENSOR_H */
