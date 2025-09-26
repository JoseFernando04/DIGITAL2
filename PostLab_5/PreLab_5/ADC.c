/*
 * ADC.c
 *
 * Created: 8/04/2025 06:52:27
 *  Author: valen
 */ 

#include "ADC.h"


void ADC_init(void)
{
	ADMUX |=  (1<<REFS0);		//Seleccionar el voltaje de referencia
	ADMUX &=~ (1<<REFS1);
	ADMUX &=~ (1<<ADLAR);		//Ajustar el resultado

	ADCSRA |= (1<<ADPS0);		// divisor = 128  16000/128 = 125 KHz
	ADCSRA |= (1<<ADPS1);
	ADCSRA |= (1<<ADPS2);
	ADCSRA |= (1<<ADEN);		// Encendemos en ADC
}


uint16_t ADC_read(uint8_t canal)
{
	canal &= 0b00000111;            // Limitar la entrada a canales 0-7 (la mayoría de AVR tienen 8 canales ADC)
	ADMUX = (ADMUX & 0xF0) | canal; // Conservar los 4 bits superiores (config) y establecer los bits de selección de canal
	ADCSRA |= (1<<ADSC);            // Inicia la conversión
	while(ADCSRA & (1<<ADSC));      // Esperar hasta que se complete la conversión
	return ADC;                     // Devolver el valor del ADC
}