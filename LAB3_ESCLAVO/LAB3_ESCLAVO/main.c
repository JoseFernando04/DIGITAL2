/*
 * LAB3_ESCLAVO.c
 *
 * JOSÉ FERNANDO GORDILLO FLORES
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "spi_simple.h"
#include "adc.h"

// Función para mostrar número en LEDs (Puerto D completo)
void display_leds_slave(uint8_t number) {
	PORTD = number;
}

// Función para inicializar LEDs del esclavo
void leds_init_slave(void) {
	DDRD = 0xFF;  // Puerto D completo como salida para LEDs
	PORTD = 0x00; // Inicializar LEDs apagados
}

// Patrón de inicio para verificar funcionamiento
void startup_pattern(void) {
	for(uint8_t i = 0; i < 3; i++) {
		display_leds_slave(0xFF);
		_delay_ms(200);
		display_leds_slave(0x00);
		_delay_ms(200);
	}
}

int main(void) {
	uint8_t command, data;
	
	// Inicializar periféricos
	spi_slave_init();
	adc_init();
	leds_init_slave();
	
	// Patrón de inicio
	startup_pattern();
	
	while(1) {
		// Esperar comando del maestro
		if(spi_data_ready()) {
			command = SPDR;  // Leer comando
			
			switch(command) {
				case 0x01:  // Comando: Modo LED
				// Esperar el dato (número para mostrar en LEDs)
				while(!spi_data_ready());
				data = SPDR;
				display_leds_slave(data);
				break;
				
				case 0x02:  // Comando: Leer potenciómetros
				{
					// Leer potenciómetros
					uint16_t pot1 = adc_read(ADC_CHANNEL_0);  // A0
					uint16_t pot2 = adc_read(ADC_CHANNEL_1);  // A1
					
					// Esperar a que el maestro solicite los datos
					// El maestro enviará bytes dummy para recibir los datos
					
					// Enviar POT1 (high byte)
					while(!spi_data_ready());
					SPDR = (pot1 >> 8) & 0xFF;
					while(!spi_data_ready());
					
					// Enviar POT1 (low byte)
					SPDR = pot1 & 0xFF;
					while(!spi_data_ready());
					
					// Enviar POT2 (high byte)
					SPDR = (pot2 >> 8) & 0xFF;
					while(!spi_data_ready());
					
					// Enviar POT2 (low byte)
					SPDR = pot2 & 0xFF;
					
					// Mostrar valores en LEDs (promedio de ambos potenciómetros)
					uint8_t avg = ((pot1 + pot2) / 2) >> 2;  // Escalar a 8 bits
					display_leds_slave(avg);
				}
				break;
				
				default:
				// Comando desconocido - no hacer nada
				break;
			}
		}
		
		_delay_ms(10);  // Pequeña pausa
	}
	
	return 0;
}