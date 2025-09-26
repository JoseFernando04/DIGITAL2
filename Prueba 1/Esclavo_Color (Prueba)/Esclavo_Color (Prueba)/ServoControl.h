/*
 * servo_control.h
 *
 * Created: 8/04/2025 06:49:33
 *  Author: valen
 */ 


#ifndef SERVO_CONTROL_H_
#define SERVO_CONTROL_H_


#include <avr/io.h>

void PWM1_init(void);
void servo_writeA(float valADC);
void servo_writeB(float valADC);
float map(float x, float in_min, float in_max, float out_min, float out_max);


#endif /* SERVO_CONTROL_H_ */