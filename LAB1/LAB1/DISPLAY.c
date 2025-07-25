#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <stdint.h>
#include <avr/io.h>

void display_init(void);
void display_num(uint8_t numero);
void display_off(void);

#endif /* DISPLAY_H_ */