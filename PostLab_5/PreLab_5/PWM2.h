/*
 * PWM2.h
 *
 * Created: 8/04/2025 14:21:00
 *  Author: valen
 */ 


#ifndef PWM2_H_
#define PWM2_H_


#include <avr/io.h>

#define NO_INVERTING	1
#define INVERTING		0


void PWM2_init(void);
void PWM2_dca(uint8_t dc, uint8_t modo);
void PWM2_dcb(uint8_t dc, uint8_t modo);

#endif /* PWM2_H_ */