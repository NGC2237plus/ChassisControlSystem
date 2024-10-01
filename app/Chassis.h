/**
 * @file Chassis.h
 * @author 早上坏 (star32349@outlook.com)
 * @brief
 * @version 1.0
 * @date 2024-10-01
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef __CHASSIS_H
#define __CHASSIS_H
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "main.h"
#include "math.h"
#include "usart.h"
#include "WT901C.h"
#include "pid.h"
#include "CRC.h"
#define Motor_PI 3.14159265358979323846f /*!< 圆周率 */

#define Motor1_PWM_H htim2.Instance->CCR1 /*!< 定时器2 PWM通道1 */
#define Motor1_PWM_L htim2.Instance->CCR2 /*!< 定时器2 PWM通道2 */

#define Motor2_PWM_H htim2.Instance->CCR3 /*!< 定时器2 PWM通道3 */
#define Motor2_PWM_L htim2.Instance->CCR4 /*!< 定时器2 PWM通道4 */

#define Motor3_PWM_H htim3.Instance->CCR1 /*!< 定时器3 PWM通道1 */
#define Motor3_PWM_L htim3.Instance->CCR2 /*!< 定时器3 PWM通道2 */

#define Motor4_PWM_H htim3.Instance->CCR3 /*!< 定时器3 PWM通道3 */
#define Motor4_PWM_L htim3.Instance->CCR4 /*!< 定时器3 PWM通道4 */

#define Packet_Head 0x05 /*!< 帧头 */
#define Packet_End 0x22  /*!< 帧尾 */
#define MaxSpeed 1000    /*!< 速度最大值 */
#define MinSpeed 100     /*!< 速度最小值 */
/**
 * @struct Remote_control
 * @brief 控制数据结构体
 *
 */
typedef struct
{
    int16_t Vx;
    int16_t Vy;
    int16_t Vz;
} Remote_control;
/**
 * @struct Motor_t
 * @brief 电机结构体
 *
 */
typedef struct
{
    int16_t Set_speed;          /*!< 设置速度 */
    __IO uint16_t Encode_Count; /*!< 编码器计数 */
    float rps;                  /*!< 转速n/s */
    float speed;                /*!< 速度m/s */
    int8_t dir;                 /*!< 电机旋转方向 */
    PID_TypeDef pid;            /*!< pid速度环 */
} Motor_t;
/**
 * @struct Chassis_t
 * @brief 底盘数据结构体
 *
 */
typedef struct
{
    Motor_t Motor1;
    Motor_t Motor2;
    Motor_t Motor3;
    Motor_t Motor4;
    float set_angle;       /*!< 设定角度 */
    float real_angle;      /*!< 实际角度 */
    PID_TypeDef Angle_pid; /*!< pid角度环 */
} Chassis_t;

extern Remote_control control;
extern uint8_t Usart_Head;
extern Chassis_t chassis;
void PWM_Init(TIM_HandleTypeDef *timHandle);
void Motor_PWM(void);
void Motor_Speed(int16_t vx, int16_t vy, int16_t RotateV);

void Chassis_UART_RxCpltCallback(void);
#endif
