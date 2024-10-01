/**
 * @file Chassis.h
 * @author ���ϻ� (star32349@outlook.com)
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
#define Motor_PI 3.14159265358979323846f /*!< Բ���� */

#define Motor1_PWM_H htim2.Instance->CCR1 /*!< ��ʱ��2 PWMͨ��1 */
#define Motor1_PWM_L htim2.Instance->CCR2 /*!< ��ʱ��2 PWMͨ��2 */

#define Motor2_PWM_H htim2.Instance->CCR3 /*!< ��ʱ��2 PWMͨ��3 */
#define Motor2_PWM_L htim2.Instance->CCR4 /*!< ��ʱ��2 PWMͨ��4 */

#define Motor3_PWM_H htim3.Instance->CCR1 /*!< ��ʱ��3 PWMͨ��1 */
#define Motor3_PWM_L htim3.Instance->CCR2 /*!< ��ʱ��3 PWMͨ��2 */

#define Motor4_PWM_H htim3.Instance->CCR3 /*!< ��ʱ��3 PWMͨ��3 */
#define Motor4_PWM_L htim3.Instance->CCR4 /*!< ��ʱ��3 PWMͨ��4 */

#define Packet_Head 0x05 /*!< ֡ͷ */
#define Packet_End 0x22  /*!< ֡β */
#define MaxSpeed 1000    /*!< �ٶ����ֵ */
#define MinSpeed 100     /*!< �ٶ���Сֵ */
/**
 * @struct Remote_control
 * @brief �������ݽṹ��
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
 * @brief ����ṹ��
 *
 */
typedef struct
{
    int16_t Set_speed;          /*!< �����ٶ� */
    __IO uint16_t Encode_Count; /*!< ���������� */
    float rps;                  /*!< ת��n/s */
    float speed;                /*!< �ٶ�m/s */
    int8_t dir;                 /*!< �����ת���� */
    PID_TypeDef pid;            /*!< pid�ٶȻ� */
} Motor_t;
/**
 * @struct Chassis_t
 * @brief �������ݽṹ��
 *
 */
typedef struct
{
    Motor_t Motor1;
    Motor_t Motor2;
    Motor_t Motor3;
    Motor_t Motor4;
    float set_angle;       /*!< �趨�Ƕ� */
    float real_angle;      /*!< ʵ�ʽǶ� */
    PID_TypeDef Angle_pid; /*!< pid�ǶȻ� */
} Chassis_t;

extern Remote_control control;
extern uint8_t Usart_Head;
extern Chassis_t chassis;
void PWM_Init(TIM_HandleTypeDef *timHandle);
void Motor_PWM(void);
void Motor_Speed(int16_t vx, int16_t vy, int16_t RotateV);

void Chassis_UART_RxCpltCallback(void);
#endif
