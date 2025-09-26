/*
 * LAB6.c
 *
 * José Fernando Gordillo Flores
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include "uart.h"

#define BTN_UP     PC2
#define BTN_DOWN   PC1
#define BTN_LEFT   PC0
#define BTN_RIGHT  PC3
#define BTN_A      PC4
#define BTN_B      PC5

static void buttons_init(void) {
	DDRC &= ~((1<<BTN_UP)|(1<<BTN_DOWN)|(1<<BTN_LEFT)|(1<<BTN_RIGHT)|(1<<BTN_A)|(1<<BTN_B));
	PORTC |= (1<<BTN_UP)|(1<<BTN_DOWN)|(1<<BTN_LEFT)|(1<<BTN_RIGHT)|(1<<BTN_A)|(1<<BTN_B); // pull-ups
}

int main(void) {
	uart_init();
	buttons_init();

	while (1) {
		if (!(PINC & (1<<BTN_UP))) {
			uart_transmit('U');
			while(!(PINC & (1<<BTN_UP))); // esperar hasta que se suelte
			_delay_ms(50); // pequeño delay anti-rebote
		}
		if (!(PINC & (1<<BTN_DOWN))) {
			uart_transmit('D');
			while(!(PINC & (1<<BTN_DOWN)));
			_delay_ms(50);
		}
		if (!(PINC & (1<<BTN_LEFT))) {
			uart_transmit('L');
			while(!(PINC & (1<<BTN_LEFT)));
			_delay_ms(50);
		}
		if (!(PINC & (1<<BTN_RIGHT))) {
			uart_transmit('R');
			while(!(PINC & (1<<BTN_RIGHT)));
			_delay_ms(50);
		}
		if (!(PINC & (1<<BTN_A))) {
			uart_transmit('A');
			while(!(PINC & (1<<BTN_A)));
			_delay_ms(50);
		}
		if (!(PINC & (1<<BTN_B))) {
			uart_transmit('B');
			while(!(PINC & (1<<BTN_B)));
			_delay_ms(50);
		}
	}
}