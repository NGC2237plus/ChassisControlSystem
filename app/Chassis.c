/**
 * @mainpage  智能车底盘控制程序
 * <table>
 * <tr><th>工程  <td>F407_Chassis2.0
 * <tr><th>作者   <td>早上坏 (star32349@outlook.com)
 * </table>
 * @section   详细描述
 * 智能车底盘控制程序，根据串口数据控制移动
 *
 * @section   功能描述
 * - STM32F407VET6 + STM32 HAL库 + FreeRTOS开发
 * - 底盘采用麦克纳姆轮
 * - 陀螺仪校正运动误差
 *
 * @section   用法描述
 * - 具体串口协议请参考文档
 *
 * @section   固件更新
 * <table>
 * <tr><th>日期        <th>版本    <th>作者                             <th>Description  </tr>
 * <tr><td>2023-12-05  <td>1.0     <td>早上坏 (star32349@outlook.com)   <td>初始版本 </tr>
 * <tr><td>2024-04-01  <td>2.0     <td>早上坏 (star32349@outlook.com)   <td>算法优化，代码完善 </tr>
 * </table>
 */
/**
 * @file Chassis.c
 * @author 早上坏 (star32349@outlook.com)
 * @brief 底盘运动控制
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
#include "chassis.h"
#define Wheel_Diameter 0.06f         /*!< 轮子直径，单位m */
#define Speed_Factor 0.001f          /*!<  速度系数 */
#define Pulse_Num 390.0f             /*!<  旋转一圈编码器输出的脉冲数 */
#define DIR 1                        /*!<  旋转方向，编号小的PWM输入引脚高电平时，顺时针旋转为1，逆时针为-1 */
__IO uint16_t time_count, time, sec; /* 计时 */
uint8_t data[9];                     /* 接收的控制数据 */
uint8_t Usart_Head;                  /* 帧头 */
uint16_t CRC16;                      /* CRC校验 */
/**
 * @brief 数据有效标志
 * @note
 * - value = 0：数据有效
 * - value = 1：数据CRC校验错误
 * - value = 2：未接收到帧尾
 */
uint8_t Data_Flag = 0;
/**
 * @brief 接收状态标志
 * @note
 * - value = 0：等待帧头
 * - value = 1：已接收到帧头，开始接收剩下的数据
 */
uint8_t Usart_Flag = 0;
Remote_control control; /*!< 控制数据 */
Chassis_t chassis;      /*!< 底盘数据 */
/**
 * @brief 最大值限制
 * @param value 要限制的值
 * @param Max 限制的最大值
 * @retval 值范围[-Max, Max]
 */
#define Limit_Max(value, Max)  \
    {                          \
        if (value > Max)       \
            value = Max;       \
        else if (value < -Max) \
            value = -Max;      \
    }
/**
 * @brief 最小值限制
 * @param value 要限制的值
 * @param Min 限制的最小值
 * @retval 值范围[-Min, Min]
 */
#define Limit_Min(value, Min)              \
    {                                      \
        if (value <= Min && value >= -Min) \
            value = 0;                     \
    }
/**
 * @brief 底盘FreeRTOS任务
 */
void Chassis_Task(void const *argument)
{
    uint16_t count_buf1, count_buf2, count_buf3, count_buf4; /*!< 缓冲，防止计算时数值变化 */
    /* 开启定时器中断 */
    HAL_TIM_Base_Start_IT(&htim4);
    /* PWM定时器初始化 */
    PWM_Init(&htim2);
    PWM_Init(&htim3);
    /* 底盘控制串口接收中断 */
    HAL_UART_Receive_IT(&huart1, &Usart_Head, 1);
    /* 陀螺仪串口接收中断 */
    HAL_UART_Receive_IT(&huart2, &Rx_buf, 1);
    // PID_Init(&chassis.Angle_pid, 2, 1, 1, 100, 100);//角度环
    // chassis.set_angle = IMU_angle.Yaw; // 角度环pid
    /* PID参数初始化 */
    PID_Init(&chassis.Motor1.pid, 5000, 10, 100, 1000, 1000);
    PID_Init(&chassis.Motor2.pid, 5000, 10, 100, 1000, 1000);
    PID_Init(&chassis.Motor3.pid, 5000, 10, 100, 1000, 1000);
    PID_Init(&chassis.Motor4.pid, 5000, 10, 100, 1000, 1000);
    control.Vx = control.Vy = control.Vz = 0;
    while (1)
    {

        // if (Data_Flag)
        // chassis.real_angle = IMU_angle.Yaw;
        // chassis.set_angle += control.Vz * 0.001;
        // if (chassis.set_angle >= 180)
        //     chassis.set_angle -= 360;
        // else if (chassis.set_angle <= -180)
        //     chassis.set_angle += 360;
        // PID_Add_Calc(&chassis.Angle_pid, chassis.set_angle, chassis.real_angle);
        // Motor_Speed(control.Vx, control.Vy, -chassis.Angle_pid.Output * 50);
        /* 计算设置速度 */
        Motor_Speed(control.Vx, control.Vy, control.Vz);

        /* 计算实际速度 */
        time_count = 0;
        chassis.Motor1.Encode_Count = chassis.Motor2.Encode_Count = chassis.Motor3.Encode_Count = chassis.Motor4.Encode_Count = 0;
        while (time_count < 10)
            ;
        count_buf1 = chassis.Motor1.Encode_Count;
        count_buf2 = chassis.Motor2.Encode_Count;
        count_buf3 = chassis.Motor3.Encode_Count;
        count_buf4 = chassis.Motor4.Encode_Count;

        chassis.Motor1.rps = count_buf1 / Pulse_Num * 100;
        chassis.Motor1.speed = chassis.Motor1.rps * Motor_PI * Wheel_Diameter * chassis.Motor1.dir;

        chassis.Motor2.rps = count_buf2 / Pulse_Num * 100;
        chassis.Motor2.speed = chassis.Motor2.rps * Motor_PI * Wheel_Diameter * chassis.Motor2.dir;

        chassis.Motor3.rps = count_buf3 / Pulse_Num * 100;
        chassis.Motor3.speed = chassis.Motor3.rps * Motor_PI * Wheel_Diameter * chassis.Motor3.dir;

        chassis.Motor4.rps = count_buf4 / Pulse_Num * 100;
        chassis.Motor4.speed = chassis.Motor4.rps * Motor_PI * Wheel_Diameter * chassis.Motor4.dir;

        /* PID计算 */
        PID_Calc(&chassis.Motor1.pid, chassis.Motor1.Set_speed * Speed_Factor, chassis.Motor1.speed);
        PID_Calc(&chassis.Motor2.pid, chassis.Motor2.Set_speed * Speed_Factor, chassis.Motor2.speed);
        PID_Calc(&chassis.Motor3.pid, chassis.Motor3.Set_speed * Speed_Factor, chassis.Motor3.speed);
        PID_Calc(&chassis.Motor4.pid, chassis.Motor4.Set_speed * Speed_Factor, chassis.Motor4.speed);
        Motor_PWM();
    }
}

/**
 * @brief 定时器中断回调函数
 * @note TIM4定时时间1ms
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        HAL_IncTick();
    }
    if (htim->Instance == TIM4)
    {
        time_count++;
        time++;
    }
    if (time >= 1000)
    {
        sec++;
        time = 0;
    }
}
/**
 * @brief GPIO中断回调函数
 * @note A相触发中断，B相判断方向，函数用于计算电机速度
 *
 * @param GPIO_Pin GPIO引脚号
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_1)
    {
        chassis.Motor1.Encode_Count++;
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0))
            chassis.Motor1.dir = -1;
        else
            chassis.Motor1.dir = 1;
    }
    else if (GPIO_Pin == GPIO_PIN_9)
    {
        chassis.Motor2.Encode_Count++;
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8))
            chassis.Motor2.dir = -1;
        else
            chassis.Motor2.dir = 1;
    }
    else if (GPIO_Pin == GPIO_PIN_7)
    {
        chassis.Motor3.Encode_Count++;
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))
            chassis.Motor3.dir = -1;
        else
            chassis.Motor3.dir = 1;
    }
    else if (GPIO_Pin == GPIO_PIN_5)
    {
        chassis.Motor4.Encode_Count++;
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3))
            chassis.Motor4.dir = -1;
        else
            chassis.Motor4.dir = 1;
    }
}
/**
 * @brief 启动定时器PWM输出
 *
 * @param timHandle 定时器句柄
 */
void PWM_Init(TIM_HandleTypeDef *timHandle)
{
    HAL_TIM_PWM_Start(timHandle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(timHandle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(timHandle, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(timHandle, TIM_CHANNEL_4);
}
/**
 * @brief 设置占空比，控制电机旋转
 *
 */
void Motor_PWM(void)
{
    // 1??
    if (chassis.Motor1.pid.Output > MinSpeed)
    {
        Motor1_PWM_H = chassis.Motor1.pid.Output;
        Motor1_PWM_L = 0;
    }
    else if (chassis.Motor1.pid.Output < -MinSpeed)
    {
        Motor1_PWM_H = 0;
        Motor1_PWM_L = -chassis.Motor1.pid.Output;
    }
    else
    {
        Motor1_PWM_H = Motor1_PWM_L = MaxSpeed;
    }
    // 2??
    if (chassis.Motor2.pid.Output > MinSpeed)
    {
        Motor2_PWM_H = chassis.Motor2.pid.Output;
        Motor2_PWM_L = 0;
    }
    else if (chassis.Motor2.pid.Output < -MinSpeed)
    {
        Motor2_PWM_H = 0;
        Motor2_PWM_L = -chassis.Motor2.pid.Output;
    }
    else
    {
        Motor2_PWM_H = Motor2_PWM_L = MaxSpeed;
    }
    // 3??
    if (chassis.Motor3.pid.Output > MinSpeed)
    {
        Motor3_PWM_H = chassis.Motor3.pid.Output;
        Motor3_PWM_L = 0;
    }
    else if (chassis.Motor3.pid.Output < -MinSpeed)
    {
        Motor3_PWM_H = 0;
        Motor3_PWM_L = -chassis.Motor3.pid.Output;
    }
    else
    {
        Motor3_PWM_H = Motor3_PWM_L = MaxSpeed;
    }
    // 4??
    if (chassis.Motor4.pid.Output > MinSpeed)
    {
        Motor4_PWM_H = chassis.Motor4.pid.Output;
        Motor4_PWM_L = 0;
    }
    else if (chassis.Motor4.pid.Output < -MinSpeed)
    {
        Motor4_PWM_H = 0;
        Motor4_PWM_L = -chassis.Motor4.pid.Output;
    }
    else
    {
        Motor4_PWM_H = Motor4_PWM_L = MaxSpeed;
    }
}
/**
 * @brief 计算设置速度
 *
 * @param vx X轴方向速度
 * @param vy Y轴方向速度
 * @param RotateV 旋转速度，顺时针为负值，逆时针为正值
 */
void Motor_Speed(int16_t vx, int16_t vy, int16_t RotateV)
{
    vx = vx / 4;
    vy = vy / 4;
    RotateV /= 4;

    chassis.Motor1.Set_speed = -vy + -vx + -RotateV;
    chassis.Motor2.Set_speed = -vy + vx + -RotateV;
    chassis.Motor3.Set_speed = vy + vx + -RotateV;
    chassis.Motor4.Set_speed = vy + -vx + -RotateV;

    Limit_Max(chassis.Motor1.Set_speed, MaxSpeed);
    Limit_Max(chassis.Motor2.Set_speed, MaxSpeed);
    Limit_Max(chassis.Motor3.Set_speed, MaxSpeed);
    Limit_Max(chassis.Motor4.Set_speed, MaxSpeed);
}
/**
 * @brief 底盘串口回调函数
 * @note 串口接收底盘运动控制信息
 *
 */
void Chassis_UART_RxCpltCallback(void)
{
    if (Usart_Head == Packet_Head && Usart_Flag == 0)
    {
        Usart_Flag = 1;
        HAL_UART_Receive_IT(&huart1, data, 9);
        return;
    }
    if (Usart_Flag == 1 && data[8] == Packet_End)
    {
        Usart_Flag = 0;
        CRC16 = CRC16_X25(&data[0], 6);
        if (CRC16 / 256 == data[6] & CRC16 % 256 == data[7])
        {
            control.Vx = data[0] << 8 | data[1]; // x
            control.Vy = data[2] << 8 | data[3]; // y
            control.Vz = data[4] << 8 | data[5]; // rotate
            Data_Flag = 0;
        }
        else
        {
            Data_Flag = 1;
        }
    }
    else if (Usart_Flag)
    {
        Usart_Flag = 0;
        Data_Flag = 2;
    }
    HAL_UART_Receive_IT(&huart1, &Usart_Head, 1);
}
