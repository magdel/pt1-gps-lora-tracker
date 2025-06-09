#include "simple_crc.h"

uint8_t calcCheckSum(const uint8_t *data, uint16_t length) {
    uint8_t sum = 0b10101101;  //some value to avoid valid empty zero
    for (uint16_t i = 0; i < length; i++) {
      sum = sum ^ data[i];
    }
    return sum;
  }
  