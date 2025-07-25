/*
 * LAB2.c
 *
 * JOSE FERNANDO GORDILLO FLORES
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "lcd.h"
#include "adc.h"
#include "uart.h"

int main(void) {
	lcd_init();
	adc_init();
	uart_init();

	uint16_t adc_pot1, adc_pot2;
	uint16_t volt_pot1, volt_pot2;
	char buffer[9];
	int16_t contador_s3 = 0;  // Contador S3
	uint8_t received_char;

	lcd_clear();
	lcd_set_cursor(0, 1);
	lcd_string("P1");
	lcd_set_cursor(0, 8);
	lcd_string("P2");
	lcd_set_cursor(0, 14);
	lcd_string("S3");

	// Mensaje inicial por UART
	uart_string("=== LAB2 - LCD y UART ===\r\n");
	uart_string("Comandos: '+' incrementar, '-' decrementar\r\n\r\n");

	while (1) {
		// Leer los dos potenciómetros
		adc_pot1 = adc_read(ADC_CHANNEL_0); // A0
		adc_pot2 = adc_read(ADC_CHANNEL_1); // A1

		// Convertir a milivoltios
		volt_pot1 = adc_to_voltage_mv(adc_pot1);
		volt_pot2 = adc_to_voltage_mv(adc_pot2);

		// Mostrar voltajes en LCD
		lcd_set_cursor(1, 0);
		sprintf(buffer, "%1d.%02dV", volt_pot1/1000, (volt_pot1%1000)/10);
		lcd_string(buffer);

		lcd_set_cursor(1, 7);
		sprintf(buffer, "%1d.%02dV", volt_pot2/1000, (volt_pot2%1000)/10);
		lcd_string(buffer);

		// Mostrar contador S3 en LCD
		lcd_set_cursor(1, 14);
		sprintf(buffer, "%2d", contador_s3);
		lcd_string(buffer);

		// Enviar voltajes por UART
		char uart_line[40];
		sprintf(uart_line, "P1: %1d.%02dV  P2: %1d.%02dV  S3: %2d      \r",
		volt_pot1/1000, (volt_pot1%1000)/10,
		volt_pot2/1000, (volt_pot2%1000)/10,
		contador_s3);
		uart_string(uart_line);

		// Verificar si hay comandos recibidos por UART
		if (uart_data_available()) {
			received_char = uart_receive();

			if (received_char == '+') {
				contador_s3++;
				uart_string("S3 incrementado\r\n");
			}
			else if (received_char == '-') {
				contador_s3--;
				uart_string("S3 decrementado\r\n");
			}
		}

		_delay_ms(500); // Actualiza cada 500 ms
	}
	return 0;
}