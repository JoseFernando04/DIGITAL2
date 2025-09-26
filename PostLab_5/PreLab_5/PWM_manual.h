// PWM.h
#ifndef PWM_H
#define PWM_H

#include <avr/io.h>
#include <avr/interrupt.h>

void PWM_init(uint8_t pin);
void PWM_setDutyCycle(uint8_t dutyCycle);
void PWM_start();
void PWM_stop();

#endif // PWM_H