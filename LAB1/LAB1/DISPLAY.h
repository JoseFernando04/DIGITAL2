#include "DISPLAY.h"

// Patrones para cátodo común, mapeo PD0–PD6: A-G
const uint8_t display_patterns[10] = {
	0b00111111, // 0
	0b00000110, // 1
	0b01011011, // 2
	0b01001111, // 3
	0b01100110, // 4
	0b01101101, // 5
	0b01111101, // 6
	0b00000111, // 7
	0b01111111, // 8
	0b01101111  // 9
};

void display_init(void) {
	DDRD |= 0x7F;    // PD0–PD6 como salida
	PORTD = 0x00;    // Apaga todos los segmentos
}

void display_num(uint8_t numero) {
	if (numero < 10) {
		PORTD = display_patterns[numero];
	}
}

void display_off(void) {
	PORTD = 0x00;
}