#include "checksum.h"

/*
Generic hex checksum calculation.
TODO: use this in the psyonic API
*/
uint32_t fletchers_checksum32(uint32_t* arr, int size)
{
	int32_t checksum = 0;
	int32_t fchk = 0;
	for (int i = 0; i < size; i++)
	{
		checksum += (int32_t)arr[i];
		fchk += checksum;
	}
	return fchk;
}

/*
Generic hex checksum calculation.
TODO: use this in the psyonic API
*/
uint16_t fletchers_checksum16(uint16_t* arr, int size)
{
	int16_t checksum = 0;
	int16_t fchk = 0;
	for (int i = 0; i < size; i++)
	{
		checksum += (int16_t)arr[i];
		fchk += checksum;
	}
	return fchk;
}

/*
Generic hex checksum calculation.
TODO: use this in the psyonic API
*/
uint8_t fletchers_checksum8(uint8_t* arr, int size)
{
	int8_t checksum = 0;
	int8_t fchk = 0;
	for (int i = 0; i < size; i++)
	{
		checksum += (int8_t)arr[i];
		fchk += checksum;
	}
	return fchk;
}
