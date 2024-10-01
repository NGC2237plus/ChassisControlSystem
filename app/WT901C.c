/**
 * @file WT901C.c
 * @author ���ϻ� (star32349@outlook.com)
 * @brief WT901C�����ǣ����ղ���������������
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
 *
 * @see https://wit-motion.yuque.com/wumwnr/ltst03/vl3tpy?#%20%E3%80%8AWIT%E7%A7%81%E6%9C%89%E5%8D%8F%E8%AE%AE%E3%80%8B
 */
#include "WT901C.h"
#define WT901C_HEAD 0X55   /*!< Э��ͷ */
#define TIME_TYPE 0X50     /*!< ʱ������ͷ */
#define ACC_TYPE 0X51      /*!< ���ٶ�����ͷ */
#define GYRO_TYPE 0X52     /*!< ���ٶ�����ͷ */
#define ANGLE_TYPE 0X53    /*!< �Ƕ�����ͷ */
#define MAGNETIC_TYPE 0x54 /*!< �ų�����ͷ */

uint8_t Rx_buf, Rx_flag;
uint8_t buffer[9];

WT901C_Time IMU_time;         /*!< ʱ������ */
WT901C_Acc IMU_acc;           /*!< ���ٶ����� */
WT901C_Gyro IMU_gyro;         /*!< ���ٶ����� */
WT901C_Angle IMU_angle;       /*!< �Ƕ�����ͷ */
WT901C_Magnetic IMU_magnetic; /*!< �ų�����ͷ */

uint8_t WT901C_SumCRC(uint8_t Type, uint8_t *data);
void Time_Dispose(WT901C_Time *time);
void Acc_Dispose(WT901C_Acc *acc);
void Gyro_Dispose(WT901C_Gyro *gyro);
void Magnetic_Dispose(WT901C_Magnetic *magnetic);
void Angle_Dispose(WT901C_Angle *angle);
/**
 * @brief ���ڽ����жϻص�����
 * @note ������HAL����жϻص����ڲ����жϴ��ں��Ե��ò�ͬ�Ĵ��ڽ��մ���
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == WT901C_huart.Instance)
    {
        if (Rx_buf == WT901C_HEAD && Rx_flag == 0)
        {
            Rx_flag = 1;
        }
        else if (Rx_flag == 1 && Rx_buf >= 0x50 && Rx_buf <= 0x5A)
        {
            Rx_flag = Rx_buf;
            HAL_UART_Receive_IT(&WT901C_huart, buffer, 9);
            return;
        }
        else if (Rx_flag >= 0x50 && Rx_buf <= 0x5A)
        {
            if (WT901C_SumCRC(Rx_flag, buffer) == buffer[8])
            {
                switch (Rx_flag)
                {
                case TIME_TYPE:
                    // Time_Dispose(&IMU_time);
                    break;
                case ACC_TYPE:
                    // Acc_Dispose(&IMU_acc);
                    break;
                case GYRO_TYPE:
                    // Gyro_Dispose(&IMU_gyro);
                    break;
                case ANGLE_TYPE:
                    Angle_Dispose(&IMU_angle);
                    break;
                case MAGNETIC_TYPE:
                    // Magnetic_Dispose(&IMU_magnetic);
                    break;
                default:
                    break;
                }
            }
            Rx_flag = 0;
        }
        else
            Rx_flag = 0;
        HAL_UART_Receive_IT(&WT901C_huart, &Rx_buf, 1);
    }
    else if (huart->Instance == USART1)
    {
        Chassis_UART_RxCpltCallback();
    }
}
/**
 * @brief CRCУ���
 *
 * @param Type ��������
 * @param data ����
 * @return uint8_t CRCУ���
 */
uint8_t WT901C_SumCRC(uint8_t Type, uint8_t *data)
{
    uint8_t i;
    uint8_t Sum = WT901C_HEAD + Type;
    for (i = 0; i < 8; i++)
    {
        Sum += data[i];
    }
    return Sum;
}
/**
 * @brief ʱ�����ݽ���
 *
 * @param time ʱ������
 */
void Time_Dispose(WT901C_Time *time)
{
    time->Year = buffer[0];
    time->Month = buffer[1];
    time->Day = buffer[2];
    time->Hour = buffer[3];
    time->Minute = buffer[4];
    time->Second = buffer[5];
    time->ms = (uint16_t)buffer[7] << 8 | buffer[6];
}
/**
 * @brief ���ٶ����ݽ���
 *
 * @param time ���ٶ�����
 */
void Acc_Dispose(WT901C_Acc *acc)
{
    acc->Acc_x = (int16_t)((int16_t)buffer[1] << 8 | buffer[0]) / 32768.0 * 16;
    acc->Acc_y = (int16_t)((int16_t)buffer[3] << 8 | buffer[2]) / 32768.0 * 16;
    acc->Acc_z = (int16_t)((int16_t)buffer[5] << 8 | buffer[4]) / 32768.0 * 16;
    acc->Temp = (((int16_t)buffer[7] << 8) | buffer[6]) / 100.0;
}
/**
 * @brief ���ٶ����ݽ���
 *
 * @param time ���ٶ�����
 */
void Gyro_Dispose(WT901C_Gyro *gyro)
{
    gyro->Gyro_x = (int16_t)((int16_t)buffer[1] << 8 | buffer[0]) / 32768.0 * 2000;
    gyro->Gyro_y = (int16_t)((int16_t)buffer[3] << 8 | buffer[2]) / 32768.0 * 2000;
    gyro->Gyro_z = (int16_t)((int16_t)buffer[5] << 8 | buffer[4]) / 32768.0 * 2000;
    // gyro->Vol = (((int16_t)buffer[7] << 8) | buffer[6]) / 100.0;
}
/**
 * @brief �Ƕ����ݽ���
 *
 * @param time �Ƕ�����
 */
void Angle_Dispose(WT901C_Angle *angle)
{
    angle->Roll = (int16_t)((int16_t)buffer[1] << 8 | buffer[0]) / 32768.0 * 180;
    angle->Pitch = (int16_t)((int16_t)buffer[3] << 8 | buffer[2]) / 32768.0 * 180;
    angle->Yaw = (int16_t)((int16_t)buffer[5] << 8 | buffer[4]) / 32768.0 * 180;
    angle->Version = ((uint16_t)buffer[7] << 8) | buffer[6];
}
/**
 * @brief �ų����ݽ���
 *
 * @param time �ų�����
 */
void Magnetic_Dispose(WT901C_Magnetic *magnetic)
{
    magnetic->Hx = (int16_t)((int16_t)buffer[1] << 8 | buffer[0]);
    magnetic->Hy = (int16_t)((int16_t)buffer[3] << 8 | buffer[2]);
    magnetic->Hz = (int16_t)((int16_t)buffer[5] << 8 | buffer[4]);
    magnetic->Temp = (((int16_t)buffer[7] << 8) | buffer[6]) / 100.0;
}
