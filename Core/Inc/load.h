
#ifndef __LOAD_H__
#define __LOAD_H__
#include <stdint.h>

void load_rotation_isr(uint16_t GPIOPin);

void load_tick0(void);
void load_tick1(void);

void load_poll(void);

void load_init(void);

#endif /* INC_LOAD_H_ */
