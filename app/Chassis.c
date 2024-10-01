/**
 * @mainpage  ���ܳ����̿��Ƴ���
 * <table>
 * <tr><th>����  <td>F407_Chassis2.0
 * <tr><th>����   <td>���ϻ� (star32349@outlook.com)
 * </table>
 * @section   ��ϸ����
 * ���ܳ����̿��Ƴ��򣬸��ݴ������ݿ����ƶ�
 *
 * @section   ��������
 * - STM32F407VET6 + STM32 HAL�� + FreeRTOS����
 * - ���̲��������ķ��
 * - ������У���˶����
 *
 * @section   �÷�����
 * - ���崮��Э����ο��ĵ�
 *
 * @section   �̼�����
 * <table>
 * <tr><th>����        <th>�汾    <th>����                             <th>Description  </tr>
 * <tr><td>2023-12-05  <td>1.0     <td>���ϻ� (star32349@outlook.com)   <td>��ʼ�汾 </tr>
 * <tr><td>2024-04-01  <td>2.0     <td>���ϻ� (star32349@outlook.com)   <td>�㷨�Ż����������� </tr>
 * </table>
 */
/**
 * @file Chassis.c
 * @author ���ϻ� (star32349@outlook.com)
 * @brief �����˶�����
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
#include "chassis.h"
#define Wheel_Diameter 0.06f         /*!< ����ֱ������λm */
#define Speed_Factor 0.001f          /*!<  �ٶ�ϵ�� */
#define Pulse_Num 390.0f             /*!<  ��תһȦ����������������� */
#define DIR 1                        /*!<  ��ת���򣬱��С��PWM�������Ÿߵ�ƽʱ��˳ʱ����תΪ1����ʱ��Ϊ-1 */
__IO uint16_t time_count, time, sec; /* ��ʱ */
uint8_t data[9];                     /* ���յĿ������� */
uint8_t Usart_Head;                  /* ֡ͷ */
uint16_t CRC16;                      /* CRCУ�� */
/**
 * @brief ������Ч��־
 * @note
 * - value = 0��������Ч
 * - value = 1������CRCУ�����
 * - value = 2��δ���յ�֡β
 */
uint8_t Data_Flag = 0;
/**
 * @brief ����״̬��־
 * @note
 * - value = 0���ȴ�֡ͷ
 * - value = 1���ѽ��յ�֡ͷ����ʼ����ʣ�µ�����
 */
uint8_t Usart_Flag = 0;
Remote_control control; /*!< �������� */
Chassis_t chassis;      /*!< �������� */
/**
 * @brief ���ֵ����
 * @param value Ҫ���Ƶ�ֵ
 * @param Max ���Ƶ����ֵ
 * @retval ֵ��Χ[-Max, Max]
 */
#define Limit_Max(value, Max)  \
    {                          \
        if (value > Max)       \
            value = Max;       \
        else if (value < -Max) \
            value = -Max;      \
    }
/**
 * @brief ��Сֵ����
 * @param value Ҫ���Ƶ�ֵ
 * @param Min ���Ƶ���Сֵ
 * @retval ֵ��Χ[-Min, Min]
 */
#define Limit_Min(value, Min)              \
    {                                      \
        if (value <= Min && value >= -Min) \
            value = 0;                     \
    }
/**
 * @brief ����FreeRTOS����
 */
void Chassis_Task(void const *argument)
{
    uint16_t count_buf1, count_buf2, count_buf3, count_buf4; /*!< ���壬��ֹ����ʱ��ֵ�仯 */
    /* ������ʱ���ж� */
    HAL_TIM_Base_Start_IT(&htim4);
    /* PWM��ʱ����ʼ�� */
    PWM_Init(&htim2);
    PWM_Init(&htim3);
    /* ���̿��ƴ��ڽ����ж� */
    HAL_UART_Receive_IT(&huart1, &Usart_Head, 1);
    /* �����Ǵ��ڽ����ж� */
    HAL_UART_Receive_IT(&huart2, &Rx_buf, 1);
    // PID_Init(&chassis.Angle_pid, 2, 1, 1, 100, 100);//�ǶȻ�
    // chassis.set_angle = IMU_angle.Yaw; // �ǶȻ�pid
    /* PID������ʼ�� */
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
        /* ���������ٶ� */
        Motor_Speed(control.Vx, control.Vy, control.Vz);

        /* ����ʵ���ٶ� */
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

        /* PID���� */
        PID_Calc(&chassis.Motor1.pid, chassis.Motor1.Set_speed * Speed_Factor, chassis.Motor1.speed);
        PID_Calc(&chassis.Motor2.pid, chassis.Motor2.Set_speed * Speed_Factor, chassis.Motor2.speed);
        PID_Calc(&chassis.Motor3.pid, chassis.Motor3.Set_speed * Speed_Factor, chassis.Motor3.speed);
        PID_Calc(&chassis.Motor4.pid, chassis.Motor4.Set_speed * Speed_Factor, chassis.Motor4.speed);
        Motor_PWM();
    }
}

/**
 * @brief ��ʱ���жϻص�����
 * @note TIM4��ʱʱ��1ms
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
 * @brief GPIO�жϻص�����
 * @note A�ഥ���жϣ�B���жϷ��򣬺������ڼ������ٶ�
 *
 * @param GPIO_Pin GPIO���ź�
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
 * @brief ������ʱ��PWM���
 *
 * @param timHandle ��ʱ�����
 */
void PWM_Init(TIM_HandleTypeDef *timHandle)
{
    HAL_TIM_PWM_Start(timHandle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(timHandle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(timHandle, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(timHandle, TIM_CHANNEL_4);
}
/**
 * @brief ����ռ�ձȣ����Ƶ����ת
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
 * @brief ���������ٶ�
 *
 * @param vx X�᷽���ٶ�
 * @param vy Y�᷽���ٶ�
 * @param RotateV ��ת�ٶȣ�˳ʱ��Ϊ��ֵ����ʱ��Ϊ��ֵ
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
 * @brief ���̴��ڻص�����
 * @note ���ڽ��յ����˶�������Ϣ
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
