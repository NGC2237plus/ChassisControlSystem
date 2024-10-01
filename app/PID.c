/**
 * @file PID.c
 * @author 早上坏 (star32349@outlook.com)
 * @brief PID算法板级支持包
 * @version 1.0
 * @date 2024-10-01
 *
 * @copyright Copyright (c) 2024
 *
 * @par 修改日志:
 * <table>
 * <tr><th>日期         <th>版本  <th>作者    <th>描述
 * <tr><td>22024-10-01  <td>1.0   <td>早上坏  <td>初始版本
 * </table>
 */
#include "pid.h"
/**
 * @brief PID参数初始化
 *
 * @param pid pid结构体指针
 * @param p p参数
 * @param i i参数
 * @param d d参数
 * @param maxKI i限幅
 * @param maxOut 输出限幅
 */
void PID_Init(PID_TypeDef *pid, float p, float i, float d, float maxKI, float maxOut)
{
    pid->KP = p;
    pid->KI = i;
    pid->KD = d;
    pid->Max_KI = maxKI;
    pid->Max_Output = maxOut;
    pid->Error = pid->Last_Error = pid->Last_Last_Error = pid->Sum_Error = pid->pout = pid->iout = pid->dout = pid->Output = 0;
}
/**
 * @brief 位置式pid计算
 *
 * @param pid pid结构体指针
 * @param SetValue 设置值
 * @param FeedBack 实际值
 */
void PID_Calc(PID_TypeDef *pid, float SetValue, float FeedBack)
{
    pid->Error = SetValue - FeedBack;
    pid->Sum_Error += pid->Error;
    pid->pout = pid->KP * pid->Error;
    pid->iout = pid->KI * pid->Sum_Error;
    pid->dout = pid->KD * (pid->Error - pid->Last_Error);
    // 积分限幅
    if (pid->iout > pid->Max_KI)
        pid->iout = pid->Max_KI;
    else if (pid->iout < -pid->Max_KI)
        pid->iout = -pid->Max_KI;
    // // 输出限幅
    pid->Output = pid->pout + pid->iout + pid->dout;
    if (pid->Output > pid->Max_Output)
        pid->Output = pid->Max_Output;
    else if (pid->Output < -pid->Max_Output)
        pid->Output = -pid->Max_Output;

    pid->Last_Error = pid->Error;
}
/**
 * @brief 增量式pid计算
 *
 * @param pid pid结构体指针
 * @param SetValue 设置值
 * @param FeedBack 实际值
 */
void PID_Add_Calc(PID_TypeDef *pid, float SetValue, float FeedBack)
{
    pid->Error = SetValue - FeedBack;
    if (pid->Error >= 180)
        pid->Error -= 360;
    else if (pid->Error <= -180)
        pid->Error += 360;
    if (pid->Error <= 2.5f && pid->Error >= -2.5f)
        pid->Error = 0;
    pid->pout = pid->KP * (pid->Error - pid->Last_Error);
    pid->iout = pid->KI * pid->Error;
    pid->dout = pid->KD * (pid->Error - 2 * pid->Last_Error + pid->Last_Last_Error);

    pid->Output = pid->pout + pid->iout + pid->dout;
    if (pid->Output > pid->Max_Output)
        pid->Output = pid->Max_Output;
    else if (pid->Output < -pid->Max_Output)
        pid->Output = -pid->Max_Output;

    pid->Last_Last_Error = pid->Last_Error;
    pid->Last_Error = pid->Error;
}
/**
 * @brief 清空pid，除参数外赋值为0
 *
 * @param pid pid结构体指针
 */
void PID_clear(PID_TypeDef *pid)
{
    pid->Error = pid->Last_Error = pid->Last_Last_Error = pid->Sum_Error = pid->pout = pid->iout = pid->dout = pid->Output = 0;
}
