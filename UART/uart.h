#ifndef UART_H
#define UART_H

#include <avr/io.h>
#include <stdio.h>

// Configuración UART
#define BAUD_RATE 9600
#define UBRR_VALUE ((F_CPU / (16UL * BAUD_RATE)) - 1)

// Prototipos de funciones
void uart_init(void);
void uart_transmit(uint8_t data);
uint8_t uart_receive(void);
uint8_t uart_data_available(void);
void uart_string(const char* str);
void uart_print_voltage(uint16_t voltage_mv, const char* label);

#endif