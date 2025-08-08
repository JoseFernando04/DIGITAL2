#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "adc.h"
#include "SPI2.h"

// Variables globales
volatile uint16_t pot1_value = 0;
volatile uint16_t pot2_value = 0;
volatile uint8_t command_received = 0;

// Función para inicializar SPI en modo esclavo
void spi_slave_init(void) {
	// Configurar pines SPI
	DDRB |= (1 << DDB4);  // MISO como salida (obligatorio)
	DDRB &= ~((1 << DDB3) | (1 << DDB5) | (1 << DDB2));  // MOSI, SCK, SS como entradas
	
	// Habilitar SPI, interrupciones, modo esclavo, MSB primero
	SPCR = (1 << SPE) | (1 << SPIE) | (0 << MSTR);
	
	// Opcional: habilitar pull-up en SS si es necesario
	PORTB |= (1 << PORTB2);
	
	// Preparar primer byte (dummy)
	SPDR = 0x00;  // Inicializar con 0 es más seguro que 0xAA
}

// Interrupción SPI
ISR(SPI_STC_vect) {
	static uint8_t command = 0;
	
	// Leer el dato recibido
	uint8_t received_data = SPDR;
	
	// Si SS está bajo (activo), procesamos el comando
	if (!(PINB & (1 << PINB2))) {
		// Interpretar como comando si es el primer byte
		if (command == 0) {
			command = received_data;
		}
		
		// Responder según el comando recibido
		switch(command) {
			case 0x01:  // Parte alta de pot1
			SPDR = (uint8_t)(pot1_value >> 8);
			command = 0;  // Resetear comando
			break;
			
			case 0x02:  // Parte baja de pot1
			SPDR = (uint8_t)(pot1_value & 0xFF);
			command = 0;
			break;
			
			case 0x03:  // Parte alta de pot2
			SPDR = (uint8_t)(pot2_value >> 8);
			command = 0;
			break;
			
			case 0x04:  // Parte baja de pot2
			SPDR = (uint8_t)(pot2_value & 0xFF);
			command = 0;
			break;
			
			default:
			SPDR = 0xFF;  // Respuesta de error
			command = 0;
			break;
		}
		
		command_received = 1;
		} else {
		// SS está alto, preparar para nueva transacción
		command = 0;
		SPDR = 0x00;
	}
}

int main(void) {
	// Inicializar periféricos
	adc_init();
	spi_slave_init();
	sei();
	
	// LED de debug
	DDRD |= (1 << DDD7);
	PORTD &= ~(1 << PORTD7);  // Apagar LED inicialmente
	
	while(1) {
		// Leer potenciómetros continuamente
		pot1_value = adc_read(ADC_CHANNEL_0);
		pot2_value = adc_read(ADC_CHANNEL_1);
		
		// Indicador visual de actividad
		if(command_received) {
			PORTD ^= (1 << PORTD7);  // Alternar LED
			command_received = 0;
			_delay_ms(50);  // Pequeño retardo para ver el LED
		}
		
		_delay_ms(20);  // Retardo entre lecturas ADC
	}
	
	return 0;
}