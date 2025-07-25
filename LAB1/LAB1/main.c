#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>

// Segmentos del display a–g conectados en PD0 a PD6
const uint8_t numeros[10] = {
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

// Variables globales
uint8_t contadorJ1 = 0;
uint8_t contadorJ2 = 0;
bool juegoActivo = false;

// Botones
#define BTN_INICIO PB0
#define BTN_J1     PB1
#define BTN_J2     PB2

// LEDs jugador 1: PB3, PB4, PB5, PC0
// LEDs jugador 2: PC1, PC2, PC3, PC4

void mostrar_display(uint8_t valor) {
	if (valor > 9) return;
	PORTD = numeros[valor];  // PD0–PD6 (a–g)
}

void apagar_display(void) {
	PORTD = 0x00;
}

// Enciende LEDs de jugador 1 en modo "de décadas" (uno a uno, máximo 4)
void encender_leds_jugador1(uint8_t valor) {
	// Apaga todos
	PORTB &= ~((1 << PB3)|(1 << PB4)|(1 << PB5));
	PORTC &= ~(1 << PC0);

	// De 1 a 4: enciende progresivamente, de 5 a 9: todos encendidos
	uint8_t leds = (valor > 4) ? 4 : valor;
	if (leds >= 1) PORTB |= (1 << PB3);
	if (leds >= 2) PORTB |= (1 << PB4);
	if (leds >= 3) PORTB |= (1 << PB5);
	if (leds == 4) PORTC |= (1 << PC0);
}

// Enciende LEDs de jugador 2 en modo "de décadas" (uno a uno, máximo 4)
void encender_leds_jugador2(uint8_t valor) {
	// Apaga todos
	PORTC &= ~((1 << PC1)|(1 << PC2)|(1 << PC3)|(1 << PC4));

	// De 1 a 4: enciende progresivamente, de 5 a 9: todos encendidos
	uint8_t leds = (valor > 4) ? 4 : valor;
	if (leds >= 1) PORTC |= (1 << PC1);
	if (leds >= 2) PORTC |= (1 << PC2);
	if (leds >= 3) PORTC |= (1 << PC3);
	if (leds == 4) PORTC |= (1 << PC4);
}

void refrescar_leds() {
	encender_leds_jugador1(contadorJ1);
	encender_leds_jugador2(contadorJ2);
}

void iniciar_carrera() {
	// Apagar LEDs y reiniciar
	juegoActivo = false;
	contadorJ1 = 0;
	contadorJ2 = 0;
	encender_leds_jugador1(0);
	encender_leds_jugador2(0);
	mostrar_display(0); // evitar residuo visual
	_delay_ms(300);

	// Conteo regresivo
	for (int i = 5; i >= 0; i--) {
		mostrar_display(i);
		_delay_ms(1000);
	}

	apagar_display();
	juegoActivo = true;
}

bool leer_boton(uint8_t pin) {
	if (!(PINB & (1 << pin))) {
		_delay_ms(20); // debounce
		if (!(PINB & (1 << pin))) {
			while (!(PINB & (1 << pin))); // espera a soltar
			return true;
		}
	}
	return false;
}

int main(void)
{
	// Display: PD0–PD6 como salida
	DDRD = 0b01111111;

	// LEDs: PB3, PB4, PB5 como salida
	DDRB |= (1 << PB3)|(1 << PB4)|(1 << PB5);
	// LEDs: PC0, PC1, PC2, PC3, PC4 como salida
	DDRC |= (1 << PC0)|(1 << PC1)|(1 << PC2)|(1 << PC3)|(1 << PC4);

	// Botones PB0, PB1, PB2 como entrada con pull-up
	DDRB &= ~((1 << BTN_INICIO)|(1 << BTN_J1)|(1 << BTN_J2));
	PORTB |= (1 << BTN_INICIO)|(1 << BTN_J1)|(1 << BTN_J2);

	while (1)
	{
		if (leer_boton(BTN_INICIO) && !juegoActivo) {
			iniciar_carrera();
		}

		if (juegoActivo) {
			if (leer_boton(BTN_J1) && contadorJ1 < 9) {
				contadorJ1++;
			}
			if (leer_boton(BTN_J2) && contadorJ2 < 9) {
				contadorJ2++;
			}
		}

		if (contadorJ1 == 9) {
			mostrar_display(1); // Jugador 1 gana
			encender_leds_jugador1(4); // Enciende todos los LEDs jugador 1
			encender_leds_jugador2(0); // Apaga los de jugador 2
			juegoActivo = false;
			} else if (contadorJ2 == 9) {
			mostrar_display(2); // Jugador 2 gana
			encender_leds_jugador2(4); // Enciende todos los LEDs jugador 2
			encender_leds_jugador1(0); // Apaga los de jugador 1
			juegoActivo = false;
			} else if (juegoActivo) {
			apagar_display(); // Display apagado durante la carrera
		}

		refrescar_leds();
	}
}