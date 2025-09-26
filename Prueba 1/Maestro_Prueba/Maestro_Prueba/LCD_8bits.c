/*
 * LCD_8bits.c
 *
 * Created: 1/08/2025 07:23:22
 *  Author: valen
 */ 

#define F_CPU 16000000
#include <util/delay.h>
#include "LCD_8bits.h"

// Nueva configuración de pines:
// D0-D5: PD2-PD7 (6 bits de datos)
// D6-D7: PB0-PB1 (2 bits de datos restantes)
// RS: PB2
// E: PB3

void initLCD8(void){
	// Configurar pines de datos PD2-PD7 como salidas
	DDRD |= 0b11111100; // PD2, PD3, PD4, PD5, PD6, PD7
	
	// Configurar pines PB0, PB1 (datos), PB2 (RS), PB3 (E) como salidas
	DDRB |= 0b00001111; // PB0, PB1, PB2, PB3
	
	_delay_ms(15);
	
	// Function SET (8 bits, 2 líneas, 5x8 dots)
	LCD8_CMD(0b00111000);
	_delay_ms(5);
	
	// Display ON/OFF (Display ON, Cursor OFF, Blink OFF)
	LCD8_CMD(0b00001100);
	_delay_ms(2);
	
	// Entry mode (Increment cursor, no shift)
	LCD8_CMD(0b00000110);
	_delay_ms(2);
	
	// Clear display
	LCD8_CMD(0b00000001);
	_delay_ms(2);
}

void LCD8_PORT(uint8_t data) {
	// Limpiar RS (modo comando)
	PORTB &= ~(1 << 2);
	
	// Enviar datos
	// D0-D5 van a PD2-PD7
	PORTD = (PORTD & 0b00000011) | (data << 2);
	
	// D6-D7 van a PB0-PB1
	PORTB = (PORTB & 0b11111100) | ((data >> 6) & 0b00000011);
	
	// Pulso de Enable
	PORTB |= (1 << 3);   // E = 1
	_delay_ms(1);
	PORTB &= ~(1 << 3);  // E = 0
	_delay_ms(1);
}

void LCD8_CMD(uint8_t data){
	// Limpiar RS (modo comando)
	PORTB &= ~(1 << 2);
	
	// Enviar datos
	// D0-D5 van a PD2-PD7
	PORTD = (PORTD & 0b00000011) | (data << 2);
	
	// D6-D7 van a PB0-PB1
	PORTB = (PORTB & 0b11111100) | ((data >> 6) & 0b00000011);
	
	// Pulso de Enable
	PORTB |= (1 << 3);   // E = 1
	_delay_ms(1);
	PORTB &= ~(1 << 3);  // E = 0
	_delay_ms(1);
}

void LCD8_Write_Char(char c){
	// Establecer RS = 1 (modo datos)
	PORTB |= (1 << 2);
	
	// Enviar datos del carácter
	// D0-D5 van a PD2-PD7
	PORTD = (PORTD & 0b00000011) | (c << 2);
	
	// D6-D7 van a PB0-PB1
	PORTB = (PORTB & 0b11111100) | ((c >> 6) & 0b00000011) | (1 << 2); // Mantener RS = 1
	
	// Pulso de Enable
	PORTB |= (1 << 3);   // E = 1
	_delay_ms(1);
	PORTB &= ~(1 << 3);  // E = 0
	_delay_ms(1);
}

void LCD8_Write_String(char *a){
	for(int i = 0; a[i] != '\0'; i++){
		LCD8_Write_Char(a[i]);
	}
}

void LCD8_Set_Cursor(uint8_t col, uint8_t row){
	uint8_t address = 0;  
	if (row == 0) {
		address = 0x80 + col;  // Primera fila
		} else if (row == 1) {
		address = 0xC0 + col;  // Segunda fila
	}
	LCD8_CMD(address);
}

void LCD8_Clear(void){
	LCD8_CMD(0x01); // Comando para limpiar pantalla
	_delay_ms(2);
}

void LCD8_Variable(float v, uint8_t n){
	char str[10];
	float_to_string(v, str, n);
	LCD8_Write_String(str);
}

void LCD8_Variable_U(uint8_t v){
	char str[4];
	uint8_to_string(v, str);
	LCD8_Write_String(str);
}

void float_to_string(float num, char *buffer, uint8_t decimales) {
	int parte_entera = (int)num;
	int parte_decimal = (int)((num - parte_entera) * 100);
	
	if (parte_decimal < 0) parte_decimal *= -1;
	
	int i = 0;
	if (parte_entera == 0) {
		buffer[i++] = '0';
		} else {
		int temp = parte_entera;
		char temp_buffer[10];
		int j = 0;
		while (temp > 0) {
			temp_buffer[j++] = (temp % 10) + '0';
			temp /= 10;
		}
		while (j > 0) {
			buffer[i++] = temp_buffer[--j];
		}
	}
	
	buffer[i++] = '.';
	buffer[i++] = (parte_decimal / 10) + '0';
	buffer[i++] = (parte_decimal % 10) + '0';
	buffer[i] = '\0';
}

void uint8_to_string(uint8_t num, char *buffer) {
	uint8_t i = 0;
	char temp[4];
	uint8_t j = 0;
	
	if (num == 0) {
		buffer[i++] = '0';
		} else {
		while (num > 0) {
			temp[j++] = (num % 10) + '0';
			num /= 10;
		}
		while (j > 0) {
			buffer[i++] = temp[--j];
		}
	}
	buffer[i] = '\0';
}