/**
 * @file WT901C.h
 * @author 早上坏 (star32349@outlook.com)
 * @brief
 * @version 1.0
 * @date 2024-10-01
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef __WT901C_H
#define __WT901C_H

#include "stm32f4xx_hal.h"
#include "usart.h"
#include "Chassis.h"
#include "IMU.h"
#define WT901C_huart huart2 /*!< 陀螺仪使用串口2 */
/**
 * @struct WT901C_Time
 * @brief 时间结构体
 * 
 */
typedef struct
{
    uint8_t Year;
    uint8_t Month;
    uint8_t Day;
    uint8_t Hour;
    uint8_t Minute;
    uint8_t Second;
    uint16_t ms;
} WT901C_Time;
/**
 * @struct WT901C_Acc
 * @brief 加速度结构体
 * 
 */
typedef struct
{
    float Acc_x;
    float Acc_y;
    float Acc_z;
    float Temp;
} WT901C_Acc;
/**
 * @struct WT901C_Gyro
 * @brief 角速度结构体
 *
 */
typedef struct
{
    float Gyro_x;
    float Gyro_y;
    float Gyro_z;
    // float Vol;
} WT901C_Gyro;
/**
 * @struct WT901C_Angle
 * @brief 角度结构体
 *
 */
typedef struct
{
    float Yaw;
    float Pitch;
    float Roll;
    float Version;
} WT901C_Angle;
/**
 * @struct WT901C_Magnetic
 * @brief 磁场结构体
 *
 */
typedef struct
{
    float Hx;
    float Hy;
    float Hz;
    float Temp;
} WT901C_Magnetic;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

extern uint8_t Rx_buf;
extern WT901C_Angle IMU_angle;
#endif
