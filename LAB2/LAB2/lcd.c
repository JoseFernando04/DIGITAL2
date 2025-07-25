#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "lcd.h"

// Función para poner los datos en los pines correctos
void lcd_set_data(uint8_t data) {
	// D0-D5 en PORTD, D6-D7 en PORTB
	if (data & (1 << 0)) LCD_D0_PORT |= (1 << LCD_D0_PIN);
	else LCD_D0_PORT &= ~(1 << LCD_D0_PIN);

	if (data & (1 << 1)) LCD_D1_PORT |= (1 << LCD_D1_PIN);
	else LCD_D1_PORT &= ~(1 << LCD_D1_PIN);

	if (data & (1 << 2)) LCD_D2_PORT |= (1 << LCD_D2_PIN);
	else LCD_D2_PORT &= ~(1 << LCD_D2_PIN);

	if (data & (1 << 3)) LCD_D3_PORT |= (1 << LCD_D3_PIN);
	else LCD_D3_PORT &= ~(1 << LCD_D3_PIN);

	if (data & (1 << 4)) LCD_D4_PORT |= (1 << LCD_D4_PIN);
	else LCD_D4_PORT &= ~(1 << LCD_D4_PIN);

	if (data & (1 << 5)) LCD_D5_PORT |= (1 << LCD_D5_PIN);
	else LCD_D5_PORT &= ~(1 << LCD_D5_PIN);

	if (data & (1 << 6)) LCD_D6_PORT |= (1 << LCD_D6_PIN);
	else LCD_D6_PORT &= ~(1 << LCD_D6_PIN);

	if (data & (1 << 7)) LCD_D7_PORT |= (1 << LCD_D7_PIN);
	else LCD_D7_PORT &= ~(1 << LCD_D7_PIN);
}

// Pulso de habilitación
void lcd_pulse_enable(void) {
	LCD_CTRL_PORT |= (1 << LCD_E);   // E = 1
	_delay_us(1);                    // Pulso mínimo
	LCD_CTRL_PORT &= ~(1 << LCD_E);  // E = 0
	_delay_us(50);                   // Tiempo de setup
}

// Enviar comando a la LCD
void lcd_command(uint8_t cmd) {
	LCD_CTRL_PORT &= ~(1 << LCD_RS); // RS = 0 para comando
	lcd_set_data(cmd);               // Colocar comando en pines de datos
	lcd_pulse_enable();              // Generar pulso Enable
	_delay_ms(2);                    // Tiempo de ejecución del comando
}

// Enviar dato a la LCD
void lcd_data(uint8_t data) {
	LCD_CTRL_PORT |= (1 << LCD_RS);  // RS = 1 para datos
	lcd_set_data(data);              // Colocar dato en pines de datos
	lcd_pulse_enable();              // Generar pulso Enable
	_delay_us(50);                   // Tiempo de ejecución del dato
}

// Inicializar LCD
void lcd_init(void) {
	// Configurar pines de datos como salida
	LCD_D0_DDR |= (1 << LCD_D0_PIN);
	LCD_D1_DDR |= (1 << LCD_D1_PIN);
	LCD_D2_DDR |= (1 << LCD_D2_PIN);
	LCD_D3_DDR |= (1 << LCD_D3_PIN);
	LCD_D4_DDR |= (1 << LCD_D4_PIN);
	LCD_D5_DDR |= (1 << LCD_D5_PIN);
	LCD_D6_DDR |= (1 << LCD_D6_PIN);
	LCD_D7_DDR |= (1 << LCD_D7_PIN);

	// Pines de control como salida
	LCD_CTRL_DDR |= (1 << LCD_RS) | (1 << LCD_E);

	// Esperar estabilización de la alimentación
	_delay_ms(20);

	// Secuencia de inicialización por software (según datasheet HD44780)
	lcd_command(0x30);  // Function set: 8-bit
	_delay_ms(5);
	lcd_command(0x30);  // Function set: 8-bit
	_delay_us(100);
	lcd_command(0x30);  // Function set: 8-bit

	// Configuración final de la LCD
	lcd_command(LCD_FUNCTION_SET);   // 8-bit, 2 líneas, 5x7 dots
	lcd_command(LCD_DISPLAY_OFF);    // Display off
	lcd_command(LCD_CLEAR);          // Clear display
	lcd_command(LCD_ENTRY_MODE);     // Entry mode: incrementar cursor
	lcd_command(LCD_DISPLAY_ON);     // Display on, cursor off
}

// Mostrar cadena de caracteres
void lcd_string(char* str) {
	while(*str) {
		lcd_data(*str++);
	}
}

// Posicionar cursor
void lcd_set_cursor(uint8_t row, uint8_t col) {
	uint8_t address;
	if(row == 0) {
		address = 0x00 + col;  // Primera línea
		} else {
		address = 0x40 + col;  // Segunda línea
	}
	lcd_command(LCD_SET_CURSOR | address);
}

// Limpiar pantalla
void lcd_clear(void) {
	lcd_command(LCD_CLEAR);
	_delay_ms(2);
}