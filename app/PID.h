/**
 * @file PID.h
 * @author 早上坏 (star32349@outlook.com)
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
 * @brief pid结构体
 *
 */
typedef struct
{
    float KP;              /*!< p参数 */
    float KI;              /*!< i参数 */
    float KD;              /*!< d参数 */
    float Error;           /*!< 当前误差 */
    float Last_Error;      /*!< 上次误差 */
    float Last_Last_Error; /*!< 上上次误差 */
    float Sum_Error;       /*!< 累积误差 */
    float Max_KI;          /*!< i限幅 */
    float Max_Output;      /*!< 输出限幅 */
    float Output;          /*!< 输出值 */
    float pout;            /*!< p输出 */
    float iout;            /*!< i输出 */
    float dout;            /*!< d输出 */
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float p, float i, float d, float maxKI, float maxOut);
void PID_Calc(PID_TypeDef *pid, float reference, float feedback);
void PID_Add_Calc(PID_TypeDef *pid, float SetValue, float FeedBack);
void PID_clear(PID_TypeDef *pid);

#endif
