#ifndef ARRAY_PROCESS_H
#define ARRAY_PROCESS_H
#include <stdint.h>
#include <strings.h>


void process_32bit_to_16bit(unsigned char * parr, size_t arr_size, int32_t div, size_t * newlen);


#endif