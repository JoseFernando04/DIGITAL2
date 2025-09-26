#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>

#define F_CPU 16000000UL

// Definición de pines para TCS3200
#define S0_PIN PD6    // Pin D6
#define S1_PIN PB0    // Pin D8
#define S2_PIN PD4    // Pin D4
#define S3_PIN PD2    // Pin D2
#define OUT_PIN PB4   // Pin D12

// Definición de pin para Servo
#define SERVO_PIN PB1 // Pin D9 (OC1A - Timer1)

// Variables para lectura de frecuencia
volatile uint16_t pulse_count = 0;
volatile uint8_t measurement_ready = 0;

// Estructura para valores RGB
typedef struct {
	uint16_t red;
	uint16_t green;
	uint16_t blue;
} rgb_values_t;

// Función para inicializar UART
void uart_init(void) {
	// Configurar baudrate a 9600
	UBRR0H = (F_CPU/16/9600-1) >> 8;
	UBRR0L = F_CPU/16/9600-1;
	
	// Habilitar transmisión y recepción
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	
	// Configurar formato: 8 bits de datos, 1 stop bit
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
}

// Función para enviar un carácter por UART
void uart_putchar(char c) {
	while (!(UCSR0A & (1<<UDRE0)));
	UDR0 = c;
}

// Función para enviar string por UART
void uart_puts(const char* str) {
	while (*str) {
		uart_putchar(*str++);
	}
}

// Función para inicializar Timer1 para PWM del servo
void servo_init(void) {
	// Configurar pin del servo como salida
	DDRB |= (1 << SERVO_PIN);
	
	// Configurar Timer1 en modo Fast PWM, TOP en ICR1
	TCCR1A = (1 << COM1A1) | (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11) | (1 << CS10); // Prescaler 64
	
	// Configurar periodo de 20ms (50Hz) para servo
	ICR1 = 4999; // (16MHz / (64 * 50Hz)) - 1
	
	// Posición inicial del servo (0 grados) - 1ms pulse
	OCR1A = 249; // (1ms / 20ms) * 4999
}

// Función para mover servo a posición específica
void servo_set_position(uint8_t angle) {
	// Convertir ángulo (0-180) a valor PWM
	// 0° = 1ms (249), 90° = 1.5ms (374), 180° = 2ms (499)
	if (angle > 180) angle = 180;
	uint16_t pulse_width = 249 + (angle * 125) / 90; // Interpolación lineal
	OCR1A = pulse_width;
}

// Función para inicializar TCS3200
void tcs3200_init(void) {
	// Configurar pines de control como salidas
	DDRD |= (1 << S0_PIN) | (1 << S2_PIN) | (1 << S3_PIN);
	DDRB |= (1 << S1_PIN);
	
	// Configurar pin OUT como entrada
	DDRB &= ~(1 << OUT_PIN);
	
	// Configurar frecuencia de salida (S0=L, S1=L = 2%) - Menor sensibilidad
	PORTD &= ~(1 << S0_PIN);
	PORTB &= ~(1 << S1_PIN);
	
	// Configurar Timer2 para contar pulsos (Timer1 se usa para servo)
	TCCR2A = 0;
	TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); // Prescaler 1024
	
	// Habilitar Pin Change Interrupt para PB4 (OUT pin)
	PCICR |= (1 << PCIE0);  // Habilitar PCINT para PORTB
	PCMSK0 |= (1 << PCINT4); // Habilitar PCINT4 (PB4)
}

// Función para seleccionar filtro de color
void tcs3200_select_filter(uint8_t filter) {
	switch(filter) {
		case 0: // Sin filtro (claro)
		PORTD &= ~(1 << S2_PIN);
		PORTD &= ~(1 << S3_PIN);
		break;
		case 1: // Filtro rojo
		PORTD &= ~(1 << S2_PIN);
		PORTD |= (1 << S3_PIN);
		break;
		case 2: // Filtro azul
		PORTD &= ~(1 << S3_PIN);
		PORTD |= (1 << S2_PIN);
		break;
		case 3: // Filtro verde
		PORTD |= (1 << S2_PIN);
		PORTD |= (1 << S3_PIN);
		break;
	}
}

// Interrupción para contar pulsos del TCS3200
ISR(PCINT0_vect) {
	static uint8_t last_state = 0;
	uint8_t current_state = PINB & (1 << OUT_PIN);
	
	// Detectar flanco ascendente
	if (current_state && !last_state) {
		pulse_count++;
	}
	last_state = current_state;
}

// Timer para controlar el tiempo de medición
ISR(TIMER2_COMPA_vect) {
	measurement_ready = 1;
	TCCR2B = 0; // Detener timer
}

// Función para leer frecuencia del TCS3200 (mejorada)
uint16_t tcs3200_read_frequency(void) {
	pulse_count = 0;
	measurement_ready = 0;
	
	// Configurar Timer2 para ~200ms (tiempo más largo para mejor precisión)
	TCNT2 = 0;
	OCR2A = 195;
	TIMSK2 |= (1 << OCIE2A);
	TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20);
	
	// Esperar hasta que la medición esté lista
	while (!measurement_ready);
	
	TIMSK2 &= ~(1 << OCIE2A); // Deshabilitar interrupción
	
	return pulse_count * 5; // Multiplicar por 5 para obtener Hz (200ms = 1/5 segundo)
}

// Función para leer valores RGB
void tcs3200_read_rgb(rgb_values_t* rgb) {
	// Leer rojo
	tcs3200_select_filter(1);
	_delay_ms(10);
	rgb->red = tcs3200_read_frequency();
	
	// Leer verde
	tcs3200_select_filter(3);
	_delay_ms(10);
	rgb->green = tcs3200_read_frequency();
	
	// Leer azul
	tcs3200_select_filter(2);
	_delay_ms(10);
	rgb->blue = tcs3200_read_frequency();
}

// Función para detectar si el color es azul (mejorada)
uint8_t is_blue_color(rgb_values_t* rgb) {
	// Calcular el total para obtener porcentajes relativos
	uint32_t total = rgb->red + rgb->green + rgb->blue;
	
	// Evitar división por cero
	if (total < 100) return 0;
	
	// Calcular porcentajes
	uint16_t red_percent = (rgb->red * 100) / total;
	uint16_t green_percent = (rgb->green * 100) / total;
	uint16_t blue_percent = (rgb->blue * 100) / total;
	
	// Criterios para detectar azul:
	// 1. El azul debe ser al menos 40% del total
	// 2. El azul debe ser mayor que rojo y verde
	// 3. La diferencia debe ser significativa (al menos 10%)
	if ((blue_percent >= 40) &&
	(blue_percent > red_percent + 10) &&
	(blue_percent > green_percent + 10)) {
		return 1;
	}
	return 0;
}

int main(void) {
	rgb_values_t color_values;
	char buffer[100];
	
	// Inicializar periféricos
	uart_init();
	tcs3200_init();
	servo_init();
	
	// Habilitar interrupciones globales
	sei();
	
	uart_puts("Sistema iniciado - Detector de color azul\r\n");
	uart_puts("Servo en posicion inicial (0 grados)\r\n");
	
	while(1) {
		// Leer valores RGB del sensor
		tcs3200_read_rgb(&color_values);
		
		// Calcular porcentajes para mejor análisis
		uint32_t total = color_values.red + color_values.green + color_values.blue;
		uint16_t red_percent = 0, green_percent = 0, blue_percent = 0;
		
		if (total > 0) {
			red_percent = (color_values.red * 100) / total;
			green_percent = (color_values.green * 100) / total;
			blue_percent = (color_values.blue * 100) / total;
		}
		
		// Mostrar valores por UART con porcentajes
		sprintf(buffer, "RGB: R=%u(%u%%) G=%u(%u%%) B=%u(%u%%) Total=%lu\r\n",
		color_values.red, red_percent,
		color_values.green, green_percent,
		color_values.blue, blue_percent, total);
		uart_puts(buffer);
		
		// Verificar si es color azul
		if (is_blue_color(&color_values)) {
			uart_puts("¡Color AZUL detectado! Moviendo servo a 90 grados\r\n");
			servo_set_position(90);
			_delay_ms(2000); // Mantener posición por 2 segundos
			
			uart_puts("Regresando servo a posicion inicial\r\n");
			servo_set_position(0);
			} else {
			uart_puts("Color no es azul\r\n");
		}
		
		_delay_ms(500); // Esperar antes de la siguiente lectura
	}
	
	return 0;
}