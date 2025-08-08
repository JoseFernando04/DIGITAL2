#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "adc.h"

// Inicializar el ADC
void adc_init(void) {
	// Configurar referencia de voltaje (AVCC con capacitor externo en AREF)
	// REFS1 = 0, REFS0 = 1 -> AVCC como referencia
	ADMUX = (1 << REFS0);
	
	// Habilitar ADC y configurar prescaler
	// Para 16MHz, prescaler de 128 da frecuencia ADC de 125kHz (óptima)
	// ADEN = 1 (habilitar ADC)
	// ADPS2:0 = 111 (prescaler 128)
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	
	// Realizar una conversión dummy para estabilizar el ADC
	adc_read(0);
}

// Leer valor del ADC en el canal especificado
uint16_t adc_read(uint8_t channel) {
	// Verificar que el canal sea válido (0-7)
	if(channel > 7) return 0;
	
	// Seleccionar canal manteniendo la configuración de referencia
	ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
	
	// Esperar un poco para que se estabilice la selección del canal
	_delay_us(10);
	
	// Iniciar conversión
	ADCSRA |= (1 << ADSC);
	
	// Esperar a que termine la conversión
	while(ADCSRA & (1 << ADSC));
	
	// Retornar el resultado (registro ADC de 16 bits, pero solo 10 bits útiles)
	return ADC;
}

// Convertir valor ADC a voltaje en milivoltios
uint16_t adc_to_voltage_mv(uint16_t adc_value) {
	// Conversión: (adc_value * Vref_mV) / resolución_ADC
	// Para evitar overflow, usar aritmética de 32 bits
	uint32_t voltage = ((uint32_t)adc_value * ADC_VREF_MV) / ADC_RESOLUTION;
	return (uint16_t)voltage;
}

// Convertir valor ADC a porcentaje (0-100%)
uint8_t adc_to_percentage(uint16_t adc_value) {
	// Conversión: (adc_value * 100) / (resolución_ADC - 1)
	uint32_t percentage = ((uint32_t)adc_value * 100) / (ADC_RESOLUTION - 1);
	if(percentage > 100) percentage = 100;
	return (uint8_t)percentage;
}