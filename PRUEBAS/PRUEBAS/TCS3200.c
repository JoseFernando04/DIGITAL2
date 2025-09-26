/*
 * TCS3200.c
 *
 */ 

#include "TCS3200.h"

// Variables globales para calibración
static uint32_t baseline_red = 0;
static uint32_t baseline_green = 0;
static uint32_t baseline_blue = 0;

// Inicialización de pines
void tcs_init(void) {
	DDRD |= (1 << TCS_S0) | (1 << TCS_S2) | (1 << TCS_S3);
	DDRB |= (1 << TCS_S1);
	DDRB &= ~(1 << TCS_OUT);

	PORTD |= (1 << TCS_S0);  // S0=1 ? 20% escala
	PORTB &= ~(1 << TCS_S1); // S1=0
}

// Leer frecuencia con filtro
uint32_t tcs_read_frequency(uint8_t s2, uint8_t s3) {
	if (s2) PORTD |= (1 << TCS_S2);
	else    PORTD &= ~(1 << TCS_S2);

	if (s3) PORTD |= (1 << TCS_S3);
	else    PORTD &= ~(1 << TCS_S3);

	_delay_ms(50);

	uint16_t count = 0;
	uint32_t time_ms = 100;
	for (uint32_t i = 0; i < (time_ms * 1000UL); i++) {
		if (PINB & (1 << TCS_OUT)) {
			while (PINB & (1 << TCS_OUT));
			count++;
		}
	}

	return count * (1000 / time_ms);
}

// ---- Nueva función de calibración ----
void tcs_calibrate(void) {
	uint32_t r_sum = 0, g_sum = 0, b_sum = 0;

	// Tomar 10 muestras y promediar
	for (uint8_t i = 0; i < 10; i++) {
		r_sum += tcs_read_frequency(0,0); // Red
		b_sum += tcs_read_frequency(0,1); // Blue
		g_sum += tcs_read_frequency(1,1); // Green
	}

	baseline_red   = r_sum / 10;
	baseline_green = g_sum / 10;
	baseline_blue  = b_sum / 10;
}

// Detectar color dominante usando calibración
void tcs_get_color(char *color) {
	uint32_t red   = tcs_read_frequency(0,0);
	uint32_t blue  = tcs_read_frequency(0,1);
	uint32_t green = tcs_read_frequency(1,1);

	// Definimos tolerancia como +30 encima del baseline (ajústalo en pruebas)
	const uint32_t TOLERANCE = 30;

	if ((red < baseline_red + TOLERANCE) &&
	(green < baseline_green + TOLERANCE) &&
	(blue < baseline_blue + TOLERANCE)) {
		strcpy(color, "NINGUNO");
		} else if (red > green && red > blue) {
		strcpy(color, "ROJO");
		} else if (green > red && green > blue) {
		strcpy(color, "VERDE");
		} else {
		strcpy(color, "AZUL");
	}
}