
#ifndef __DEBUG_PIN_H__
#define __DEBUG_PIN_H__

#include <stdint.h>


#define debug_0(val) 		HAL_GPIO_WritePin(DEBUG0, val);
#define debug_1(val) 		HAL_GPIO_WritePin(DEBUG1, val);

void debug_pin_init(void);


#endif /* INC_DEBUG_PIN_H_ */
