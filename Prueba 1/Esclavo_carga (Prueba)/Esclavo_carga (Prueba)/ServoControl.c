/*
 * ServoControl.c
 *
 * Created: 8/04/2025 06:50:55
 *  Author: valen
 */ 

#include "ServoControl.h"

void PWM1_init(void){
	// PB1 | PB2
	DDRB |= ( 1<< PB1 )|(1<<PB2);
	TCNT1 = 0; // reset
	ICR1 = 39999; // TOP
	TCCR1A =  (1 << COM1A1) | (1 << COM1B1) | (0 << COM1A0) ; // low --> Compare Match
	TCCR1A |=  (1 << WGM11) | (0 << WGM10) ; // Fast PWM TOP ICR1
	TCCR1B = (1 << WGM13) | (1 << WGM12); // Fast PWM TOP ICR1
	TCCR1B |= (0 << CS12) | (1 << CS11) | ( 0 << CS10 ); // Prescaler 8
}
void servo_writeA(float adc_Value){
	OCR1A = map(adc_Value, 0, 1024, 1000, 4800);
}
void servo_writeB(float adc_Value){
	OCR1B = map(adc_Value, 0, 1024, 1000, 4800);
}
float map(float x, float in_min, float in_max, float out_min, float out_max){
	return ((x - in_min)*(out_max - out_min)/(in_max - in_min)) + out_min;
}