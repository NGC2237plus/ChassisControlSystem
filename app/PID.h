/**
 * @file PID.h
 * @author ���ϻ� (star32349@outlook.com)
 * @brief
 * @version 1.0
 * @date 2024-10-01
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef __PID_H
#define __PID_H

#include "tim.h"
#include "gpio.h"
#include "math.h"
/**
 * @struct PID_TypeDef
 * @brief pid�ṹ��
 *
 */
typedef struct
{
    float KP;              /*!< p���� */
    float KI;              /*!< i���� */
    float KD;              /*!< d���� */
    float Error;           /*!< ��ǰ��� */
    float Last_Error;      /*!< �ϴ���� */
    float Last_Last_Error; /*!< ���ϴ���� */
    float Sum_Error;       /*!< �ۻ���� */
    float Max_KI;          /*!< i�޷� */
    float Max_Output;      /*!< ����޷� */
    float Output;          /*!< ���ֵ */
    float pout;            /*!< p��� */
    float iout;            /*!< i��� */
    float dout;            /*!< d��� */
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float p, float i, float d, float maxKI, float maxOut);
void PID_Calc(PID_TypeDef *pid, float reference, float feedback);
void PID_Add_Calc(PID_TypeDef *pid, float SetValue, float FeedBack);
void PID_clear(PID_TypeDef *pid);

#endif
