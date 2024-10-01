/**
 * @file CRC.c
 * @author ���ϻ� (star32349@outlook.com)
 * @brief CRCУ���㷨�弶֧�ְ�
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
#include "crc.h"

/**
 * @brief ��תһ���ֽڵ�λ˳��
 * 
 * @param DesBuf ������������洢��ת����ֽ�
 * @param SrcBuf ���뻺����������Ҫ��ת���ֽ�
 * 
 * @note �˺����������ֽڵ�λ˳��ת���洢�������������
 */
void InvertUint8(uint8_t *DesBuf, uint8_t *SrcBuf)
{
    int i;
    uint8_t temp = 0;

    for (i = 0; i < 8; i++)
    {
        if (SrcBuf[0] & (1 << i))
        {
            temp |= 1 << (7 - i);
        }
    }
    DesBuf[0] = temp;
}
/**
 * @brief ��תһ���ֵ�λ˳��
 * 
 * @param DesBuf ������������洢��ת�����
 * @param SrcBuf ���뻺����������Ҫ��ת����
 * 
 * @note �˺����������ֵ�λ˳��ת���洢�������������
 */
void InvertUint16(uint16_t *DesBuf, uint16_t *SrcBuf)
{
    int i;
    uint16_t temp = 0;

    for (i = 0; i < 16; i++)
    {
        if (SrcBuf[0] & (1 << i))
        {
            temp |= 1 << (15 - i);
        }
    }
    DesBuf[0] = temp;
}
/**
 * @brief ���� CRC-16/X.25 У����
 * 
 * @param puchMsg ָ���������ݵ�ָ��
 * @param usDataLen �������ݵĳ���
 * @return uint16_t ����õ��� CRC У����
 * 
 * @note �˺���ʵ�� CRC-16/X25 У���㷨���������ݷ�ת����� CRC ֵ��
 */
uint16_t CRC16_X25(uint8_t *puchMsg, unsigned int usDataLen)
{
    uint16_t wCRCin = 0xFFFF;
    uint16_t wCPoly = 0x1021;
    uint8_t wChar = 0;

    while (usDataLen--)
    {
        wChar = *(puchMsg++);
        InvertUint8(&wChar, &wChar);
        wCRCin ^= (wChar << 8);

        for (int i = 0; i < 8; i++)
        {
            if (wCRCin & 0x8000)
            {
                wCRCin = (wCRCin << 1) ^ wCPoly;
            }
            else
            {
                wCRCin = wCRCin << 1;
            }
        }
    }
    InvertUint16(&wCRCin, &wCRCin);
    return (wCRCin ^ 0xFFFF);
}
