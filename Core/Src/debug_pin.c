#include "debug_pin.h"

#include "main.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

void debug_pin_init(void)
{
	debug_0(0);
	debug_1(0);
}
