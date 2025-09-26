/*
 * BANDA_TRANSPORTADORA_I2C_Fixed.c
 * 
 * ESCLAVO I2C CORREGIDO - SIN BLOQUEOS
 * - Rutina de interrupción I2C simplificada y robusta
 * - Gestión mejorada de estados del sistema
 * - Prevención de condiciones de carrera
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include "ServoControl.h"
#include "HX711.h"
#include "I2C_Global.h"

// Configuración I2C
#define SLAVE_ADDRESS   0x30
#define CMD_GET_WEIGHT  'W'
#define CMD_GET_STATE   'S'
#define CMD_START       'T'
#define CMD_STOP        'P'
#define CMD_EMERGENCY   'E'

// Definiciones de pines
#define MOTOR_PIN   PD5
#define SERVO_PIN   PB1
#define BTN_START   PD3
#define BTN_EMERG   PD4
#define SENSOR_IR   PD2

// Variables globales del sistema
volatile uint8_t motor_running = 0;
volatile uint8_t process_active = 0;
volatile uint8_t emergency_stop = 0;

// Variables I2C - SIMPLIFICADAS
volatile uint8_t i2c_command_received = 0;
volatile uint8_t i2c_last_command = 0;
volatile uint8_t i2c_response_ready = 0;
volatile uint8_t i2c_response_data[4];
volatile uint8_t i2c_response_index = 0;
volatile uint8_t i2c_response_length = 0;
volatile uint8_t i2c_state = 0; // 0=idle, 1=receiving, 2=transmitting

// Configuración del sensor
#define CALIBRATION_FACTOR 900000.0f
#define TARGET_WEIGHT_GRAMS 50

// Estados del sistema
typedef enum {
    STATE_IDLE,
    STATE_MOVING,
    STATE_BOX_DETECTED,
    STATE_WEIGHING,
    STATE_EMERGENCY
} system_state_t;

volatile system_state_t current_state = STATE_IDLE;
volatile float current_weight_grams = 0.0f;

// RUTINA I2C CORREGIDA Y SIMPLIFICADA
ISR(TWI_vect) {
    uint8_t twi_status = TWSR & 0xF8;
    
    switch(twi_status) {
        // ========== MODO ESCLAVO RECEPTOR ==========
        case 0x60: // SLA+W recibido, ACK enviado
            i2c_state = 1; // Recibiendo
            i2c_response_index = 0;
            TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA) | (1 << TWINT);
            break;
            
        case 0x80: // Dato recibido, ACK enviado
            i2c_last_command = TWDR;
            i2c_command_received = 1;
            TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA) | (1 << TWINT);
            break;
            
        case 0xA0: // STOP/RESTART recibido
            i2c_state = 0; // Idle
            TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA) | (1 << TWINT);
            break;
            
        // ========== MODO ESCLAVO TRANSMISOR ==========
        case 0xA8: // SLA+R recibido, ACK enviado
            i2c_state = 2; // Transmitiendo
            i2c_response_index = 0;
            
            // Preparar respuesta basada en el último comando
            prepare_i2c_response();
            
            // Enviar primer byte
            if (i2c_response_length > 0) {
                TWDR = i2c_response_data[i2c_response_index++];
            } else {
                TWDR = 0xFF; // Error
            }
            TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA) | (1 << TWINT);
            break;
            
        case 0xB8: // Dato transmitido, ACK recibido
            if (i2c_response_index < i2c_response_length) {
                TWDR = i2c_response_data[i2c_response_index++];
                TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA) | (1 << TWINT);
            } else {
                // No hay más datos, preparar para NACK
                TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT);
            }
            break;
            
        case 0xC0: // Dato transmitido, NACK recibido
        case 0xC8: // Último dato transmitido, ACK recibido
            i2c_state = 0; // Idle
            TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA) | (1 << TWINT);
            break;
            
        // ========== CASOS DE ERROR ==========
        default:
            // Error o estado no manejado - resetear I2C
            i2c_state = 0;
            TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWEA) | (1 << TWINT);
            break;
    }
}

// Función para preparar respuesta I2C (llamada desde ISR)
void prepare_i2c_response(void) {
    union {
        float f;
        uint8_t bytes[4];
    } weight_union;
    
    switch(i2c_last_command) {
        case CMD_GET_WEIGHT:
            // Respuesta de 4 bytes: peso como float
            weight_union.f = current_weight_grams;
            for (uint8_t i = 0; i < 4; i++) {
                i2c_response_data[i] = weight_union.bytes[i];
            }
            i2c_response_length = 4;
            break;
            
        case CMD_GET_STATE:
            // Respuesta de 3 bytes: estado, motor, emergencia
            i2c_response_data[0] = (uint8_t)current_state;
            i2c_response_data[1] = motor_running;
            i2c_response_data[2] = emergency_stop;
            i2c_response_length = 3;
            break;
            
        case CMD_START:
        case CMD_STOP:
        case CMD_EMERGENCY:
            // Respuesta de 1 byte: ACK
            i2c_response_data[0] = 0x01;
            i2c_response_length = 1;
            break;
            
        default:
            // Comando desconocido - NACK
            i2c_response_data[0] = 0xFF;
            i2c_response_length = 1;
            break;
    }
}

// Procesar comandos I2C en el loop principal
void process_i2c_commands(void) {
    if (!i2c_command_received) return;
    
    // Deshabilitar interrupciones temporalmente para evitar condiciones de carrera
    cli();
    uint8_t command = i2c_last_command;
    i2c_command_received = 0;
    sei();
    
    // Procesar comando
    switch(command) {
        case CMD_START:
            if (current_state == STATE_IDLE && !emergency_stop) {
                current_state = STATE_MOVING;
            }
            break;
            
        case CMD_STOP:
            if (current_state != STATE_EMERGENCY) {
                current_state = STATE_IDLE;
                stop_motor();
            }
            break;
            
        case CMD_EMERGENCY:
            emergency_stop = 1;
            current_state = STATE_EMERGENCY;
            break;
            
        case CMD_GET_WEIGHT:
        case CMD_GET_STATE:
            // Estos comandos solo preparan la respuesta, no ejecutan acciones
            break;
    }
}

// Inicialización I2C simplificada
void init_i2c_slave(void) {
    // Configurar dirección del esclavo
    TWAR = (SLAVE_ADDRESS << 1);
    
    // Habilitar TWI con ACK automático e interrupciones
    TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWIE);
    
    // Inicializar variables
    i2c_command_received = 0;
    i2c_last_command = 0;
    i2c_response_ready = 0;
    i2c_state = 0;
    i2c_response_length = 0;
}

// Funciones de control mejoradas
void start_motor(void) {
    if (!emergency_stop) {
        PORTD |= (1 << MOTOR_PIN);
        motor_running = 1;
    }
}

void stop_motor(void) {
    PORTD &= ~(1 << MOTOR_PIN);
    motor_running = 0;
}

void open_servo(void) {
    servo_writeA(512);
}

void close_servo(void) {
    servo_writeA(0);
}

// Actualización de peso con filtrado
void update_weight_reading(void) {
    static uint8_t reading_count = 0;
    static float weight_accumulator = 0.0f;
    
    if (hx711_is_ready()) {
        float weight_kg = hx711_get_units(1); // Una sola lectura para mayor velocidad
        float weight_grams = weight_kg * 1000.0f;
        
        // Filtrar valores fuera de rango
        if (weight_grams >= 0 && weight_grams <= 1000) {
            weight_accumulator += weight_grams;
            reading_count++;
            
            // Promedio cada 3 lecturas para suavizar
            if (reading_count >= 3) {
                cli(); // Proteger variable compartida
                current_weight_grams = weight_accumulator / reading_count;
                sei();
                
                weight_accumulator = 0.0f;
                reading_count = 0;
            }
        }
    }
}

// Interrupciones externas simplificadas
void interrupts_init(void) {
    // Configurar pines como entrada con pull-up
    DDRD &= ~((1 << BTN_START) | (1 << BTN_EMERG) | (1 << SENSOR_IR));
    PORTD |= (1 << BTN_START) | (1 << BTN_EMERG) | (1 << SENSOR_IR);
    
    // INT0 (PD2 - sensor IR) - flanco descendente
    EICRA |= (1 << ISC01);
    EIMSK |= (1 << INT0);
    
    // INT1 (PD3 - botón start) - flanco descendente
    EICRA |= (1 << ISC11);
    EIMSK |= (1 << INT1);
    
    // PCINT para botón emergencia (PD4)
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT20);
}

// ISR para sensor IR
ISR(INT0_vect) {
    if (current_state == STATE_MOVING && !emergency_stop) {
        current_state = STATE_BOX_DETECTED;
    }
}

// ISR para botón start
ISR(INT1_vect) {
    static uint32_t last_press_time = 0;
    uint32_t current_time = 0; // Implementar contador de tiempo si es necesario
    
    // Debounce simple
    if ((current_time - last_press_time) > 500) {
        if (current_state == STATE_IDLE && !emergency_stop) {
            current_state = STATE_MOVING;
        }
        last_press_time = current_time;
    }
}

// ISR para botón emergencia
ISR(PCINT2_vect) {
    if (!(PIND & (1 << BTN_EMERG))) {
        emergency_stop = 1;
        current_state = STATE_EMERGENCY;
    }
}

// Inicialización del sistema
void system_init(void) {
    // Configurar motor como salida
    DDRD |= (1 << MOTOR_PIN);
    stop_motor();
    
    // Inicializar servo
    PWM1_init();
    close_servo();
    
    // Inicializar HX711 con configuración optimizada
    hx711_init();
    _delay_ms(500);
    hx711_set_scale(CALIBRATION_FACTOR);
    _delay_ms(1000);
    hx711_tare(5); // Menos lecturas para tara más rápida
    _delay_ms(500);
    
    // Inicializar I2C como esclavo
    init_i2c_slave();
    
    // Configurar interrupciones externas
    interrupts_init();
    
    // Variables iniciales
    emergency_stop = 0;
    current_state = STATE_IDLE;
    current_weight_grams = 0.0f;
    
    // Habilitar interrupciones globales AL FINAL
    sei();
}

// MAIN - Máquina de estados mejorada
int main(void) {
    system_init();
    
    uint16_t weight_update_counter = 0;
    uint16_t state_counter = 0;
    
    while (1) {
        // Procesar comandos I2C
        process_i2c_commands();
        
        // Actualizar lectura de peso cada ~100ms (2 ciclos de 50ms)
        if (++weight_update_counter >= 2) {
            update_weight_reading();
            weight_update_counter = 0;
        }
        
        // Máquina de estados con contadores para timeouts
        switch (current_state) {
            
            case STATE_IDLE:
                stop_motor();
                state_counter = 0;
                _delay_ms(50);
                break;
                
            case STATE_MOVING:
                start_motor();
                state_counter = 0;
                _delay_ms(50);
                break;
                
            case STATE_BOX_DETECTED:
                stop_motor();
                state_counter++;
                
                if (state_counter >= 20) { // ~1 segundo (20 * 50ms)
                    open_servo();
                    current_state = STATE_WEIGHING;
                    state_counter = 0;
                }
                _delay_ms(50);
                break;
                
            case STATE_WEIGHING:
                state_counter++;
                
                // Verificar si se alcanzó el peso objetivo
                if (current_weight_grams >= TARGET_WEIGHT_GRAMS) {
                    close_servo();
                    _delay_ms(500);
                    current_state = STATE_MOVING;
                    state_counter = 0;
                }
                
                // Timeout de pesaje (10 segundos)
                else if (state_counter >= 200) { // 200 * 50ms = 10s
                    close_servo();
                    current_state = STATE_IDLE;
                    state_counter = 0;
                }
                
                _delay_ms(50);
                break;
                
            case STATE_EMERGENCY:
                stop_motor();
                close_servo();
                
                // Verificar si se liberó el botón de emergencia
                if (PIND & (1 << BTN_EMERG)) {
                    _delay_ms(100); // Debounce
                    if (PIND & (1 << BTN_EMERG)) {
                        emergency_stop = 0;
                        current_state = STATE_IDLE;
                    }
                }
                
                _delay_ms(100);
                break;
                
            default:
                current_state = STATE_IDLE;
                break;
        }
        
        // Verificación de emergencia desde cualquier estado
        if (emergency_stop && current_state != STATE_EMERGENCY) {
            current_state = STATE_EMERGENCY;
        }
    }
    
    return 0;
}