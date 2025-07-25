#ifndef LCD_H
#define LCD_H

#include <avr/io.h>
#include <stdio.h>

// Pines de datos LCD
#define LCD_D0_PORT PORTD
#define LCD_D0_DDR  DDRD
#define LCD_D0_PIN  PD2

#define LCD_D1_PORT PORTD
#define LCD_D1_DDR  DDRD
#define LCD_D1_PIN  PD3

#define LCD_D2_PORT PORTD
#define LCD_D2_DDR  DDRD
#define LCD_D2_PIN  PD4

#define LCD_D3_PORT PORTD
#define LCD_D3_DDR  DDRD
#define LCD_D3_PIN  PD5

#define LCD_D4_PORT PORTD
#define LCD_D4_DDR  DDRD
#define LCD_D4_PIN  PD6

#define LCD_D5_PORT PORTD
#define LCD_D5_DDR  DDRD
#define LCD_D5_PIN  PD7

#define LCD_D6_PORT PORTB
#define LCD_D6_DDR  DDRB
#define LCD_D6_PIN  PB2

#define LCD_D7_PORT PORTB
#define LCD_D7_DDR  DDRB
#define LCD_D7_PIN  PB3

// Pines de control
#define LCD_CTRL_PORT PORTB
#define LCD_CTRL_DDR  DDRB
#define LCD_RS        PB0
#define LCD_E         PB1

// Comandos LCD HD44780
#define LCD_CLEAR           0x01
#define LCD_HOME            0x02
#define LCD_ENTRY_MODE      0x06
#define LCD_DISPLAY_ON      0x0C
#define LCD_DISPLAY_OFF     0x08
#define LCD_CURSOR_ON       0x0E
#define LCD_CURSOR_BLINK    0x0F
#define LCD_FUNCTION_SET    0x38  // 8-bit, 2 líneas, 5x7 dots
#define LCD_SET_CURSOR      0x80

// Prototipos de funciones
void lcd_init(void);
void lcd_command(uint8_t cmd);
void lcd_data(uint8_t data);
void lcd_string(char* str);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_clear(void);

#endif