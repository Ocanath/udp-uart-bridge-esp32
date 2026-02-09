#include <stdio.h>
#include "array_process.h"
#include "unity.h"


void test_array_process(void)
{
	int32_t array[3] = {0x12340000,0x12350000,0x12360000};
	for(int i  = 0; i < 3; i++)
	{
		printf("%ld\n", array[i]);
	}
	size_t len = 0;
	process_32bit_to_16bit((unsigned char *)array, sizeof(array), &len);	
	int16_t * p16_array = (int16_t*)(&array);
	TEST_ASSERT_EQUAL(3*sizeof(int16_t),len);
	
	for(int i = 0; i < 3; i++)
	{
		printf("%d\n", p16_array[i]);
	}
	TEST_ASSERT_EQUAL(4660, p16_array[0]);
	TEST_ASSERT_EQUAL(4661, p16_array[1]);
	TEST_ASSERT_EQUAL(4662, p16_array[2]);
}