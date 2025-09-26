/*
 * PWM2.c
 *
 * Created: 8/04/2025 06:53:07
 *  Author: valen (modificado)
 */ 
#include "PWM2.h"

void PWM2_init(void)
{
	// Modo Fast PWM
	TCCR2B &= ~(1<<WGM22);
	TCCR2A |= (1<<WGM21);
	TCCR2A |= (1<<WGM20);
	
	// Prescalador 1024
	TCCR2B |= (1<<CS22);
	TCCR2B &= ~(1<<CS21);
	TCCR2B |= (1<<CS20);
}

void PWM2_dca(uint8_t dc, uint8_t modo)
{
	if(modo == 1)
	{
		TCCR2A |= (1<<COM2A1);
		TCCR2A &= ~(1<<COM2A0);
	}
	else
	{
		TCCR2A |= (1<<COM2A1);
		TCCR2A |= (1<<COM2A0);
	}
	OCR2A = (dc * 255) / 100;
}

void PWM2_dcb(uint8_t dc, uint8_t modo)
{
	if(modo == 1)
	{
		TCCR2A |= (1<<COM2B1);
		TCCR2A &= ~(1<<COM2B0);
	}
	else
	{
		TCCR2A |= (1<<COM2B1);
		TCCR2A |= (1<<COM2B0);
	}
	OCR2B = (dc * 255) / 100;
}