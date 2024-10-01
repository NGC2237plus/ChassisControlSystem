/**
 * @file PID.c
 * @author ���ϻ� (star32349@outlook.com)
 * @brief PID�㷨�弶֧�ְ�
 * @version 1.0
 * @date 2024-10-01
 *
 * @copyright Copyright (c) 2024
 *
 * @par �޸���־:
 * <table>
 * <tr><th>����         <th>�汾  <th>����    <th>����
 * <tr><td>22024-10-01  <td>1.0   <td>���ϻ�  <td>��ʼ�汾
 * </table>
 */
#include "pid.h"
/**
 * @brief PID������ʼ��
 *
 * @param pid pid�ṹ��ָ��
 * @param p p����
 * @param i i����
 * @param d d����
 * @param maxKI i�޷�
 * @param maxOut ����޷�
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
 * @brief λ��ʽpid����
 *
 * @param pid pid�ṹ��ָ��
 * @param SetValue ����ֵ
 * @param FeedBack ʵ��ֵ
 */
void PID_Calc(PID_TypeDef *pid, float SetValue, float FeedBack)
{
    pid->Error = SetValue - FeedBack;
    pid->Sum_Error += pid->Error;
    pid->pout = pid->KP * pid->Error;
    pid->iout = pid->KI * pid->Sum_Error;
    pid->dout = pid->KD * (pid->Error - pid->Last_Error);
    // �����޷�
    if (pid->iout > pid->Max_KI)
        pid->iout = pid->Max_KI;
    else if (pid->iout < -pid->Max_KI)
        pid->iout = -pid->Max_KI;
    // // ����޷�
    pid->Output = pid->pout + pid->iout + pid->dout;
    if (pid->Output > pid->Max_Output)
        pid->Output = pid->Max_Output;
    else if (pid->Output < -pid->Max_Output)
        pid->Output = -pid->Max_Output;

    pid->Last_Error = pid->Error;
}
/**
 * @brief ����ʽpid����
 *
 * @param pid pid�ṹ��ָ��
 * @param SetValue ����ֵ
 * @param FeedBack ʵ��ֵ
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
 * @brief ���pid���������⸳ֵΪ0
 *
 * @param pid pid�ṹ��ָ��
 */
void PID_clear(PID_TypeDef *pid)
{
    pid->Error = pid->Last_Error = pid->Last_Last_Error = pid->Sum_Error = pid->pout = pid->iout = pid->dout = pid->Output = 0;
}
