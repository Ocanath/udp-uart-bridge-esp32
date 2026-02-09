#ifndef CHECKSUM_H
#define CHECKSUM_H

#include <stdint.h>

uint32_t fletchers_checksum32(uint32_t* arr, int size);
uint16_t fletchers_checksum16(uint16_t* arr, int size);
uint8_t fletchers_checksum8(uint8_t* arr, int size);


#endif