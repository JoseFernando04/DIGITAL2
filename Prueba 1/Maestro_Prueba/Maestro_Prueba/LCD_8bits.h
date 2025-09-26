/*
 * LCD_8bits.h
 *
 * Created: 1/08/2025 07:46:36
 *  Author: valen
 */ 


#ifndef LCD_8BITS_H_
#define LCD_8BITS_H_


#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>

void initLCD8(void);

void LCD8_PORT(uint8_t data);

void LCD8_CMD(uint8_t data);

void LCD8_Write_Char(char c);

void LCD8_Write_String(char *a);

void LCD8_Set_Cursor(uint8_t col, uint8_t row);

void LCD8_Clear(void);

void LCD8_Variable(float v, uint8_t n);

void LCD8_Variable_U(uint8_t v);

void float_to_string(float num, char *buffer, uint8_t decimales);

void uint8_to_string(uint8_t num, char *buffer);




#endif /* LCD_8BITS_H_ */