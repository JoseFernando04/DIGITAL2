#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

int main(void) {
	// Configura PD0 a PD6 como salida
	DDRD = 0x7F;      // 0b01111111, PD0-PD6 como salida

	while (1) {
		PORTD = 0x7F; // Enciende todos los segmentos (PD0-PD6 en alto)
		// Si quieres que se queden encendidos, no pongas delay ni apagues
		// Si quieres que parpadeen, descomenta lo siguiente:
		// _delay_ms(1000);
		// PORTD = 0x00; // Apaga todos los segmentos
		// _delay_ms(1000);
	}
}