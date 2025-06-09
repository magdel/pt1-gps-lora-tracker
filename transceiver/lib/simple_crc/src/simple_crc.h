#ifndef _SIMPLE_CRC_H
#define _SIMPLE_CRC_H

#include <inttypes.h>

uint8_t calcCheckSum(const uint8_t *data, uint16_t length);
  
#endif // _SIMPLE_CRC_H