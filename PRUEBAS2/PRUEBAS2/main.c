/*
 * PRUEBAS2.c
 * COOKING
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>

#include "uart.h"
#include "HX711.h"

// Factor de calibración para celda de carga de 1kg
// Este valor necesita ser calibrado con pesos conocidos
#define CALIBRATION_FACTOR 900000.0  // Ajustar según tu celda de carga

void print_weight(float weight) {
	char buffer[32];
		
	// Convertir a gramos
	int grams = (int)(weight * 1000);
		
	sprintf(buffer, "Peso: %d g\r\n", grams);
	uart_string(buffer);
}

int main(void) {
	// Inicializar UART
	uart_init();
	
	// Inicializar HX711
	hx711_init();
	
	// Mensaje de inicio
	uart_string("=== SENSOR DE PESO HX711 ===\r\n");
	uart_string("Inicializando...\r\n");
	
	_delay_ms(1000);
	
	// Configurar factor de escala
	hx711_set_scale(CALIBRATION_FACTOR);
	
	uart_string("Calibrando cero (tara)...\r\n");
	uart_string("Asegurate de que no haya peso en la celda\r\n");
	
	_delay_ms(3000);
	
	// Realizar tara (calibrar cero)
	hx711_tare(10);
	
	uart_string("Calibracion completada!\r\n");
	uart_string("Coloca peso en la celda de carga\r\n\r\n");
	
	_delay_ms(1000);
	
	while (1) {
		if (hx711_is_ready()) {
			// Leer peso promedio de 5 lecturas
			float weight = hx711_get_units(5);
			
			// Mostrar peso por UART
			print_weight(weight);
			
			// También mostrar valor raw para debug
			int32_t raw_value = hx711_read_average(3);
			char debug_buffer[32];
			sprintf(debug_buffer, "Raw: %ld\r\n\r\n", raw_value);
			uart_string(debug_buffer);
		}
		
		_delay_ms(500);  // Leer cada 500ms
	}
	
	return 0;
}