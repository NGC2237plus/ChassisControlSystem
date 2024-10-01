/**
 * @file CRC.h
 * @author 早上坏 (star32349@outlook.com)
 * @brief 
 * @version 1.0
 * @date 2024-10-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef __CRC_H
#define __CRC_H

#include "stm32f4xx_hal.h"

void InvertUint8(uint8_t *DesBuf, uint8_t *SrcBuf);
void InvertUint16(uint16_t *DesBuf, uint16_t *SrcBuf);
uint16_t CRC16_X25(uint8_t *puchMsg, unsigned int usDataLen);

#endif
