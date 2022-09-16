
#ifndef __INC_SERIAL_H__
#define __INC_SERIAL_H__

#include <stdint.h>

int serial_send(const uint8_t *buffer, int buffer_length);
int serial_send_str(const char *str);

#endif /* INC_SERIAL_H_ */
