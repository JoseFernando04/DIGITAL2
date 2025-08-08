/*
 * LAB3_MAESTRO.c
 *
 * JOSÉ FERNANDO GORDILLO FLORES
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "spi_simple.h"

// Función para mostrar número en LEDs del maestro
// Bits 0-3: Puerto C (PC0-PC3)
// Bits 4-7: Puerto D (PD4-PD7)
void display_leds_master(uint8_t number) {
	// Limpiar los bits que vamos a usar
	PORTC &= 0xF0;  // Mantener bits altos de Puerto C
	PORTD &= 0x0F;  // Mantener bits bajos de Puerto D
	
	// Colocar bits 0-3 en Puerto C (PC0-PC3)
	PORTC |= (number & 0x0F);
	
	// Colocar bits 4-7 en Puerto D (PD4-PD7)
	PORTD |= (number & 0xF0);
}

// Función para inicializar LEDs del maestro
void leds_init_master(void) {
	// Configurar PC0-PC3 como salidas (parte baja de Puerto C)
	DDRC |= 0x0F;
	
	// Configurar PD4-PD7 como salidas (parte alta de Puerto D)
	DDRD |= 0xF0;
	
	// Inicializar LEDs apagados
	PORTC &= 0xF0;  // Apagar PC0-PC3
	PORTD &= 0x0F;  // Apagar PD4-PD7
}

// Función para mostrar menú
void show_menu(void) {
	uart_string("\r\n=== MENU PRINCIPAL ===\r\n");
	uart_string("1. Modo LEDs (enviar numero 0-255)\r\n");
	uart_string("2. Modo Potenciometros (leer valores del esclavo)\r\n");
	uart_string("Seleccione opcion (1 o 2): ");
}

// Función para leer número de 3 dígitos con ceros
uint8_t read_three_digit_number(void) {
	char buffer[4] = {0};
	uint8_t i;
	
	uart_string("Ingrese numero de 3 digitos (ej: 007, 123, 255): ");
	
	// Leer exactamente 3 dígitos
	for(i = 0; i < 3; i++) {
		while(!uart_data_available());
		uint8_t c = uart_receive();
		
		if(c >= '0' && c <= '9') {
			buffer[i] = c;
			uart_transmit(c);  // Echo
			} else {
			i--;  // Si no es dígito, repetir
		}
	}
	
	uart_string("\r\n");
	
	// Convertir a número
	uint16_t result = (buffer[0] - '0') * 100 + (buffer[1] - '0') * 10 + (buffer[2] - '0');
	if(result > 255) result = 255;
	
	return (uint8_t)result;
}

// Modo LEDs
void led_mode(void) {
	uart_string("\r\n=== MODO LEDS ===\r\n");
	
	while(1) {
		uint8_t number = read_three_digit_number();
		
		// Mostrar en LEDs del maestro
		display_leds_master(number);
		
		uart_string("Numero: ");
		uart_print_number(number);
		uart_string(" - Enviando al esclavo...\r\n");
		
		// Enviar al esclavo por SPI
		PORTB &= ~(1 << PORTB2);  // SS bajo
		_delay_us(10);
		spi_transceive(0x01);     // Comando: modo LED
		_delay_us(10);
		spi_transceive(number);   // Dato
		_delay_us(10);
		PORTB |= (1 << PORTB2);   // SS alto
		
		uart_string("Enviado! Presione 'q' para volver al menu o cualquier tecla para continuar: ");
		
		while(!uart_data_available());
		uint8_t c = uart_receive();
		uart_transmit(c);
		uart_string("\r\n");
		
		if(c == 'q' || c == 'Q') break;
	}
}

// Modo Potenciómetros
void potentiometer_mode(void) {
	uart_string("\r\n=== MODO POTENCIOMETROS ===\r\n");
	uart_string("Leyendo valores cada 2 segundos...\r\n");
	uart_string("Presione cualquier tecla para volver al menu\r\n\r\n");
	
	while(1) {
		// Solicitar lectura de potenciómetros al esclavo
		PORTB &= ~(1 << PORTB2);  // SS bajo
		_delay_us(10);
		spi_transceive(0x02);     // Comando: leer potenciómetros
		_delay_us(10);
		
		// Leer POT1 (2 bytes: high, low)
		uint8_t pot1_high = spi_transceive(0x00);
		_delay_us(10);
		uint8_t pot1_low = spi_transceive(0x00);
		_delay_us(10);
		
		// Leer POT2 (2 bytes: high, low)
		uint8_t pot2_high = spi_transceive(0x00);
		_delay_us(10);
		uint8_t pot2_low = spi_transceive(0x00);
		_delay_us(10);
		
		PORTB |= (1 << PORTB2);   // SS alto
		
		// Reconstruir valores de 16 bits
		uint16_t pot1_value = (pot1_high << 8) | pot1_low;
		uint16_t pot2_value = (pot2_high << 8) | pot2_low;
		
		// Convertir a voltaje (asumiendo 5V referencia, 10 bits ADC)
		uint16_t pot1_voltage = (pot1_value * 5000UL) / 1024;
		uint16_t pot2_voltage = (pot2_value * 5000UL) / 1024;
		
		// Mostrar resultados
		uart_string("POT1: ");
		uart_print_number(pot1_value);
		uart_string(" (");
		uart_print_number(pot1_voltage);
		uart_string("mV) | POT2: ");
		uart_print_number(pot2_value);
		uart_string(" (");
		uart_print_number(pot2_voltage);
		uart_string("mV)\r\n");
		
		// Verificar si hay tecla presionada
		if(uart_data_available()) {
			uart_receive();  // Leer y descartar
			break;
		}
		
		_delay_ms(2000);  // Esperar 2 segundos
	}
}

int main(void) {
	// Inicializar periféricos
	uart_init();
	spi_master_init();
	leds_init_master();
	
	uart_string("=== LABORATORIO SPI - MAESTRO ===\r\n");
	uart_string("Sistema iniciado correctamente\r\n");
	
	while(1) {
		show_menu();
		
		// Esperar selección
		while(!uart_data_available());
		uint8_t option = uart_receive();
		uart_transmit(option);
		uart_string("\r\n");
		
		switch(option) {
			case '1':
			led_mode();
			break;
			case '2':
			potentiometer_mode();
			break;
			default:
			uart_string("Opcion invalida. Intente de nuevo.\r\n");
			break;
		}
	}
	
	return 0;
}