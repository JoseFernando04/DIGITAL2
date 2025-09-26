/*
 * Esclavo_TCS3200_Fixed.c
 *
 * - Uso de I2C_Global para compatibilidad
 * - Posicionamiento de servomotor
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include "uart.h"
#include "TCS3200.h"
#include "I2C_Global.h"
#include "servo.h"

#define SLAVE_ADDRESS 0x20

// Variables globales
volatile uint8_t color_code = 0;
volatile uint8_t command_received = 0;
volatile uint8_t i2c_busy = 0;
volatile uint8_t update_requested = 0;  // Bandera para actualización inmediata
char color[10];

// ISR I2C optimizada con manejo de comando 'C'
ISR(TWI_vect) {
    uint8_t status = TWI_SR & 0xF8;

    switch(status) {
        case 0x60: // SLA+W recibido, ACK enviado
            i2c_busy = 1;
            TWI_CR = (1<<TWEN) | (1<<TWINT) | (1<<TWEA);
            break;

        case 0x80: // Dato recibido, ACK enviado
            command_received = TWI_DR;
            if (command_received == 'C') {
                update_requested = 1;  // Solicitar actualización inmediata
            }
            TWI_CR = (1<<TWEN) | (1<<TWINT) | (1<<TWEA);
            break;

        case 0xA0: // STOP o START repetido recibido
            TWI_CR = (1<<TWEN) | (1<<TWINT) | (1<<TWEA);
            i2c_busy = 0;
            break;

        case 0xA8: // SLA+R recibido, ACK enviado
            i2c_busy = 1;
            TWI_DR = color_code;  // Enviar código del color actual
            TWI_CR = (1<<TWEN) | (1<<TWINT) | (1<<TWEA);
            break;

        case 0xB8: // Dato transmitido, ACK recibido
            TWI_CR = (1<<TWEN) | (1<<TWINT) | (1<<TWEA);
            break;

        case 0xC0: // Dato transmitido, NACK recibido
            TWI_CR = (1<<TWEN) | (1<<TWINT) | (1<<TWEA);
            i2c_busy = 0;
            break;

        case 0xC8: // Último dato transmitido, ACK recibido
            TWI_CR = (1<<TWEN) | (1<<TWINT) | (1<<TWEA);
            i2c_busy = 0;
            break;

        default:
            // En caso de error, resetear y liberar
            TWI_CR = (1<<TWEN) | (1<<TWINT) | (1<<TWEA);
            i2c_busy = 0;
            break;
    }
}

int main(void) {
    // Inicializar periféricos
    uart_init();
    tcs_init();
    I2C_Slave_Init(SLAVE_ADDRESS);
    servo_init();
    servo_set_angle(90); // posición inicial por defecto

    sei(); // Habilitar interrupciones globales

    uart_string("=== TCS3200 + Servo Control ===\n");
    uart_string("Calibrando...\n");
    _delay_ms(2000);
    tcs_calibrate();
    uart_string("Listo!\n");

    while (1) {
        // PRIORIDAD 1: Procesar comando de actualización inmediata
        if (update_requested && !i2c_busy) {
            update_requested = 0;  // Limpiar bandera
            
            uart_string("Comando 'C' recibido - Actualizando color...\n");
            
            // Leer color del sensor inmediatamente
            tcs_get_color(color);
            
            // Interpretar color y mover servo
            if (strcmp(color, "ROJO") == 0) {
                color_code = 1;
                servo_set_angle(0);
            }
            else if (strcmp(color, "VERDE") == 0 || strcmp(color, "AZUL") == 0) {
                color_code = (strcmp(color, "VERDE") == 0) ? 2 : 3;
                servo_set_angle(180);
            }
            else {
                color_code = 0;
                servo_set_angle(90);
            }
            
            uart_string("Color detectado: ");
            uart_string(color);
            uart_string(" (Codigo: ");
            uart_print_number(color_code);
            uart_string(")\n");
        }
        
        // PRIORIDAD 2: Lectura periódica normal (si no hay I2C en curso)
        else if (!i2c_busy) {
            tcs_get_color(color);

            uint8_t new_color_code;
            if (strcmp(color, "AZUL") == 0) {
                new_color_code = 1;
                servo_set_angle(0);
            }
            else if (strcmp(color, "VERDE") == 0 || strcmp(color, "ROJO") == 0) {
                new_color_code = (strcmp(color, "VERDE") == 0) ? 2 : 3;
                servo_set_angle(180);
            }
            else {
                new_color_code = 0;
                servo_set_angle(90);
            }

            if (new_color_code != color_code) {
                color_code = new_color_code;

                uart_string("Nuevo color detectado: ");
                uart_string(color);
                uart_string(" (Codigo: ");
                uart_print_number(color_code);
                uart_string(")\n");
            }
        }

        _delay_ms(50);  // Refresco rápido
    }

    return 0;
}