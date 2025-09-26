#define F_CPU 16000000UL
#include "HX711.h"

// Variables globales
int32_t hx711_offset = 0;
float hx711_scale = 1.0;
uint8_t hx711_gain = HX711_GAIN_128;

// Inicializar HX711
void hx711_init(void) {
	// Configurar DOUT como entrada
	HX711_DOUT_DDR &= ~(1 << HX711_DOUT_BIT);
	
	// Configurar PD_SCK como salida
	HX711_PD_SCK_DDR |= (1 << HX711_PD_SCK_BIT);
	
	// Inicializar PD_SCK en LOW
	HX711_PD_SCK_PORT &= ~(1 << HX711_PD_SCK_BIT);
	
	// Esperar a que el HX711 esté listo
	_delay_ms(100);
	
	// Configurar ganancia por defecto
	hx711_set_gain(HX711_GAIN_128);
}

// Verificar si HX711 está listo para leer
uint8_t hx711_is_ready(void) {
	return !(HX711_DOUT_PIN & (1 << HX711_DOUT_BIT));
}

// Leer valor raw del HX711
int32_t hx711_read(void) {
	uint32_t count = 0;
	uint8_t i;
	
	// Esperar a que DOUT esté en LOW (listo para leer)
	while (!hx711_is_ready()) {
		_delay_us(1);
	}
	
	// Leer 24 bits de datos
	for (i = 0; i < 24; i++) {
		// Pulso de clock
		HX711_PD_SCK_PORT |= (1 << HX711_PD_SCK_BIT);
		_delay_us(1);
		
		count = count << 1;
		
		HX711_PD_SCK_PORT &= ~(1 << HX711_PD_SCK_BIT);
		_delay_us(1);
		
		if (HX711_DOUT_PIN & (1 << HX711_DOUT_BIT)) {
			count++;
		}
	}
	
	// Pulsos adicionales para configurar ganancia
	for (i = 0; i < hx711_gain; i++) {
		HX711_PD_SCK_PORT |= (1 << HX711_PD_SCK_BIT);
		_delay_us(1);
		HX711_PD_SCK_PORT &= ~(1 << HX711_PD_SCK_BIT);
		_delay_us(1);
	}
	
	// Convertir a complemento a 2 si es necesario
	if (count & 0x800000) {
		count |= 0xFF000000;
	}
	
	return (int32_t)count;
}

// Leer promedio de múltiples lecturas
int32_t hx711_read_average(uint8_t times) {
	int64_t sum = 0;
	uint8_t i;
	
	for (i = 0; i < times; i++) {
		sum += hx711_read();
		_delay_ms(1);
	}
	
	return (int32_t)(sum / times);
}

// Configurar ganancia
void hx711_set_gain(uint8_t gain) {
	switch (gain) {
		case HX711_GAIN_128:
		hx711_gain = 1;
		break;
		case HX711_GAIN_32:
		hx711_gain = 2;
		break;
		case HX711_GAIN_64:
		hx711_gain = 3;
		break;
		default:
		hx711_gain = 1;
		break;
	}
	
	// Realizar una lectura para aplicar la nueva ganancia
	hx711_read();
}

// Apagar HX711
void hx711_power_down(void) {
	HX711_PD_SCK_PORT |= (1 << HX711_PD_SCK_BIT);
	_delay_us(60);
}

// Encender HX711
void hx711_power_up(void) {
	HX711_PD_SCK_PORT &= ~(1 << HX711_PD_SCK_BIT);
	_delay_us(1);
}

// Calibrar el cero (tara)
void hx711_tare(uint8_t times) {
	int64_t sum = 0;
	uint8_t i;
	
	for (i = 0; i < times; i++) {
		sum += hx711_read();
		_delay_ms(10);
	}
	
	hx711_offset = (int32_t)(sum / times);
}

// Obtener peso en unidades calibradas
float hx711_get_units(uint8_t times) {
	return (float)(hx711_read_average(times) - hx711_offset) / hx711_scale;
}

// Configurar factor de escala
void hx711_set_scale(float scale) {
	hx711_scale = scale;
}