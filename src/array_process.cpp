#include "array_process.h"

/*
non-unit tested helper function
*/
void process_32bit_to_16bit(unsigned char * parr, size_t arr_size, int32_t div, size_t * newlen)
{
	size_t bidx_16b = 0;
	size_t bidx_32b = 0;
	for(int i = 0; i < arr_size / sizeof(int32_t); i++)
	{
		int32_t * pi32 = (int32_t*)(parr + bidx_32b) ;
		bidx_32b += sizeof(int32_t);
		//this is a convenient place to plug a gain calculation in
		int32_t val_div32 = (*pi32 / div);
		if(val_div32 > 32767) //clip to avoid overflow wrapping
		{
			val_div32 = 32767;
		}
		if(val_div32 < -32768)
		{
			val_div32 = -32768;
		}
		
		int16_t * pi16 = (int16_t*)(parr + bidx_16b);
		bidx_16b += sizeof(int16_t);
		*pi16 = (int16_t)val_div32;
	}
	*newlen = (arr_size*sizeof(int16_t))/sizeof(int32_t);
}
