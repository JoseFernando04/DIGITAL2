#include "servo.h"

#define SERVO_DDR   DDRB
#define SERVO_PORT  PORTB
#define SERVO_PIN   PB1   // OC1A

void servo_init(void) {
	// Configurar pin OC1A como salida
	SERVO_DDR |= (1 << SERVO_PIN);

	// Modo Fast PWM, TOP = ICR1
	// COM1A1=1 ? salida no invertida
	TCCR1A = (1 << COM1A1) | (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler 8

	ICR1 = 39999;  // 20 ms con F_CPU=16MHz y preescaler=8

	// Iniciar en 90° (1.5ms ? 3000)
	OCR1A = 3000;
}

void servo_set_angle(uint8_t angle) {
	// Mapear 0-180° a 1000-5000 ticks (1ms - 2ms)
	uint16_t min = 1000;
	uint16_t max = 5000;
	uint16_t value = min + ((uint32_t)(max - min) * angle) / 180;

	OCR1A = value;
}