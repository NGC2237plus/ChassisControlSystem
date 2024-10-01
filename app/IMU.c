/**
 * @file IMU.c
 * @author 早上坏 (star32349@outlook.com)
 * @brief IMU惯导计算控制
 * @version 1.0
 * @date 2024-10-01
 *
 * @copyright Copyright (c) 2024
 *
 * @deprecated 该文件已废弃，主要用于小车调试
 */
#include "IMU.h"
// float set_angle, real_angle, angel_falg;
// uint8_t pp[7];

/**
 * @brief IMU FreeRTOS任务
 *
 */
void IMU_Task(void const *argument)
{

    // HAL_Delay(1000);

    while (1)
    {

        // PID_Add_Calc(&Angle_pid, set_angle, real_angle);
        // Motor_Speed(0, 0, -Angle_pid.Output*50);
        // printf("%f,%f\r\n", chassis.Motor1.Set_speed * 0.001, chassis.Motor1.speed);
    }
}
void test(void)
{
    // if (Usart_Head == 0x77 && angel_falg == 0)
    // {
    //     HAL_UART_Receive_IT(&huart1, pp, 7);
    //     angel_falg = 1;
    //     return;
    // }
    // if (angel_falg == 1)
    // {
    //     Angle_pid.KP = pp[0] + (float)pp[1] / 10.0f;
    //     Angle_pid.KI = pp[2] + (float)pp[3] / 10.0f;
    //     Angle_pid.KD = pp[4] + (float)pp[5] / 10.0f;
    //     set_angle = pp[6];
    //     angel_falg = 0;
    // }
    // HAL_UART_Receive_IT(&huart1, &Usart_Head, 1);
}
