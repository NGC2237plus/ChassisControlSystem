/**
 * @file CRC.c
 * @author 早上坏 (star32349@outlook.com)
 * @brief CRC校验算法板级支持包
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
#include "crc.h"

/**
 * @brief 反转一个字节的位顺序
 * 
 * @param DesBuf 输出缓冲区，存储反转后的字节
 * @param SrcBuf 输入缓冲区，包含要反转的字节
 * 
 * @note 此函数将输入字节的位顺序反转并存储到输出缓冲区。
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
 * @brief 反转一个字的位顺序
 * 
 * @param DesBuf 输出缓冲区，存储反转后的字
 * @param SrcBuf 输入缓冲区，包含要反转的字
 * 
 * @note 此函数将输入字的位顺序反转并存储到输出缓冲区。
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
 * @brief 计算 CRC-16/X.25 校验码
 * 
 * @param puchMsg 指向输入数据的指针
 * @param usDataLen 输入数据的长度
 * @return uint16_t 计算得到的 CRC 校验码
 * 
 * @note 此函数实现 CRC-16/X25 校验算法，输入数据反转后计算 CRC 值。
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
