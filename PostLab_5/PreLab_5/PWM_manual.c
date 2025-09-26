// PWM.c
#include "PWM_manual.h"

static uint8_t pwmPin; // Pin de salida para el PWM
static volatile uint8_t dutyCycle; // Ciclo de trabajo
static volatile uint8_t counter = 0; // Contador para la interrupción

void PWM_init(uint8_t pin) {
	pwmPin = pin; // Guardar el pin seleccionado

	// Configurar el pin como salida
	if (pwmPin >= 0 && pwmPin <= 13) { // Verificar que el pin esté en el rango válido
		DDRB |= (1 << pwmPin); // Configurar el pin como salida
	}

	// Configurar el Timer 0
	TCCR0A |= (1 << WGM01); // Modo CTC
	TCCR0B |= (1 << CS01) | (1 << CS00); // Prescaler 64
	OCR0A = 5; // Valor de comparación para el Timer 0
	TIMSK0 |= (1 << OCIE0A); // Habilitar la interrupción de comparación
}

void PWM_setDutyCycle(uint8_t duty) {
	// Asegurarse de que el duty cycle esté en el rango de 0 a 255
	if (duty > 255) {
		duty = 255;
	}
	dutyCycle = duty; // Establecer el ciclo de trabajo
}

void PWM_start() {
	counter = 0; // Reiniciar el contador
	sei(); // Habilitar interrupciones globales
}

void PWM_stop() {
	cli(); // Deshabilitar interrupciones globales
	PORTB &= ~(1 << pwmPin); // Asegurarse de que el pin esté bajo
}

// PWM.c (continuación)
ISR(TIMER0_COMPA_vect) {
	counter++; // Incrementar el contador

	if (counter < dutyCycle) {
		PORTB |= (1 << pwmPin); // Encender el pin
		} else {
		PORTB &= ~(1 << pwmPin); // Apagar el pin
	}

	if (counter >= 255) {
		counter = 0; // Reiniciar el contador
	}
}