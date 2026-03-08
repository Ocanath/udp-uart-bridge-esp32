#ifndef CONTROL_INTERFACE_H
#define CONTROL_INTERFACE_H
#include <stdint.h>

enum {NO_ACTION, RESTART};

/*Dartt interface */
typedef struct dartt_turret_control_t
{
	int32_t s0_us;
	int32_t s1_us;
	uint32_t ms;
	uint32_t action_flag;
}dartt_turret_control_t;



#endif