#define F_CPU 16000000UL
#include <avr/io.h>
#include <stdio.h>
#include "uart.h"

// Inicializar UART
void uart_init(void) {
	// Configurar baud rate
	UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
	UBRR0L = (uint8_t)UBRR_VALUE;

	// Habilitar transmisión y recepción
	UCSR0B = (1 << TXEN0) | (1 << RXEN0);

	// Configurar formato: 8 bits de datos, 1 bit de stop, sin paridad
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Transmitir un byte
void uart_transmit(uint8_t data) {
	// Esperar a que el buffer de transmisión esté vacío
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = data;
}

// Recibir un byte
uint8_t uart_receive(void) {
	// Esperar a que llegue un dato
	while (!(UCSR0A & (1 << RXC0)));
	return UDR0;
}

// Verificar si hay datos disponibles
uint8_t uart_data_available(void) {
	return (UCSR0A & (1 << RXC0));
}

// Enviar cadena de caracteres
void uart_string(const char* str) {
	while(*str) {
		uart_transmit(*str++);
	}
}

// Enviar voltaje formateado
void uart_print_voltage(uint16_t voltage_mv, const char* label) {
	char buffer[20];
	sprintf(buffer, "%s: %d.%02dV\r\n", label, voltage_mv/1000, (voltage_mv%1000)/10);
	uart_string(buffer);
}


void uart_print_number(uint16_t num) {
	char buffer[6];
	sprintf(buffer, "%u", num);
	uart_string(buffer);
}