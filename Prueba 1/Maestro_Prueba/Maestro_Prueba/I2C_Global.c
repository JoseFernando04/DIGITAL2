/*
 * I2C_Global_Fixed.c
 * 
 * Implementación CORREGIDA de librería I2C universal
 * - Timeouts mejorados para prevenir bloqueos
 * - Recuperación robusta de errores
 * - Prevención de condiciones de carrera
 */

#include <stdlib.h>
#include "I2C_Global.h"

#define F_CPU 16000000UL
#define I2C_TIMEOUT_MS 100  // Timeout aumentado

// Variables estáticas para control de estado
static volatile uint8_t i2c_error_count = 0;
static volatile uint8_t bus_recovery_needed = 0;

// Función para detectar y recuperar bus bloqueado
static void I2C_BusRecovery(void) {
    // Deshabilitar I2C temporalmente
    TWI_CR = 0;
    
    // Configurar SCL como salida para generar pulsos de clock
    DDRC |= (1 << DDC5);  // SCL como salida
    DDRC &= ~(1 << DDC4); // SDA como entrada
    
    // Generar hasta 9 pulsos de clock para liberar SDA
    for (uint8_t i = 0; i < 9; i++) {
        PORTC |= (1 << DDC5);   // SCL alto
        _delay_us(10);
        PORTC &= ~(1 << DDC5);  // SCL bajo
        _delay_us(10);
        
        // Si SDA se libera, salir del bucle
        if (PINC & (1 << DDC4)) {
            break;
        }
    }
    
    // Restaurar pines como entradas
    DDRC &= ~((1 << DDC4) | (1 << DDC5));
    
    // Esperar estabilización
    _delay_ms(10);
    
    // Reinicializar I2C
    I2C_Init(100000);
    
    bus_recovery_needed = 0;
    i2c_error_count = 0;
}

// Función de espera con timeout mejorada
static uint8_t I2C_WaitForFlag(uint16_t timeout_ms) {
    uint16_t timeout_counter = 0;
    
    while (!(TWI_CR & (1 << TWINT))) {
        _delay_us(10);
        timeout_counter++;
        
        // Timeout alcanzado
        if (timeout_counter >= (timeout_ms * 100)) {
            return 0; // Timeout
        }
        
        // Verificar si el bus está bloqueado
        if (timeout_counter > 5000) { // 50ms
            if (!(PINC & (1 << DDC4)) || !(PINC & (1 << DDC5))) {
                bus_recovery_needed = 1;
                return 0;
            }
        }
    }
    
    return 1; // Éxito
}

// Inicializar I2C como maestro - MEJORADO
void I2C_Init(uint32_t frequency) {
    // Asegurar que los pines estén en estado conocido
    DDRC &= ~((1 << DDC4) | (1 << DDC5)); // Entradas
    PORTC |= (1 << DDC4) | (1 << DDC5);   // Pull-ups internos habilitados
    
    // Calcular TWBR
    uint8_t twbr_value = ((F_CPU / frequency) - 16) / 2;
    if (twbr_value < 10) twbr_value = 10;
    
    TWI_BR = twbr_value;
    TWI_SR = 0; // Prescaler = 1
    
    // Limpiar cualquier estado previo
    TWI_CR = (1 << TWINT) | (1 << TWEN);
    
    // Variables de control
    i2c_error_count = 0;
    bus_recovery_needed = 0;
    
    _delay_ms(10); // Estabilización
}

// START mejorado con detección de bloqueo
uint8_t I2C_Start(uint8_t address) {
    // Verificar si necesitamos recuperación del bus
    if (bus_recovery_needed) {
        I2C_BusRecovery();
    }
    
    // Limpiar flags previos
    TWI_CR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    
    // Esperar con timeout
    if (!I2C_WaitForFlag(I2C_TIMEOUT_MS)) {
        i2c_error_count++;
        if (i2c_error_count > 3) {
            bus_recovery_needed = 1;
        }
        return 0xFF; // Error de timeout
    }
    
    // Verificar estado START
    uint8_t status = I2C_GetStatus();
    if (status != I2C_START && status != I2C_REP_START) {
        return status;
    }
    
    // Enviar dirección con timeout
    TWI_DR = address;
    TWI_CR = (1 << TWINT) | (1 << TWEN);
    
    if (!I2C_WaitForFlag(I2C_TIMEOUT_MS)) {
        i2c_error_count++;
        return 0xFF; // Timeout
    }
    
    status = I2C_GetStatus();
    
    // Reset contador de errores en caso de éxito
    if (status == I2C_SLA_W_ACK || status == I2C_SLA_R_ACK) {
        i2c_error_count = 0;
    }
    
    return status;
}

// STOP mejorado
void I2C_Stop(void) {
    TWI_CR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    
    // Esperar que STOP se complete con timeout
    uint16_t timeout = 1000; // 10ms
    while ((TWI_CR & (1 << TWSTO)) && timeout > 0) {
        _delay_us(10);
        timeout--;
    }
    
    // Si STOP no se completa, forzar reset
    if (timeout == 0) {
        bus_recovery_needed = 1;
    }
    
    _delay_us(50); // Pausa mínima entre transacciones
}

// Escritura con timeout mejorado
uint8_t I2C_Write(uint8_t data) {
    TWI_DR = data;
    TWI_CR = (1 << TWINT) | (1 << TWEN);
    
    if (!I2C_WaitForFlag(I2C_TIMEOUT_MS)) {
        i2c_error_count++;
        return 0xFF; // Timeout
    }
    
    return I2C_GetStatus();
}

// Lectura con ACK - mejorado
uint8_t I2C_ReadACK(void) {
    TWI_CR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    
    if (!I2C_WaitForFlag(I2C_TIMEOUT_MS)) {
        return 0xFF; // Timeout
    }
    
    return TWI_DR;
}

// Lectura con NACK - mejorado
uint8_t I2C_ReadNACK(void) {
    TWI_CR = (1 << TWINT) | (1 << TWEN);
    
    if (!I2C_WaitForFlag(I2C_TIMEOUT_MS)) {
        return 0xFF; // Timeout
    }
    
    return TWI_DR;
}

// Estado del bus
uint8_t I2C_GetStatus(void) {
    return TWI_SR & 0xF8;
}

// Inicialización esclavo - CORREGIDA
void I2C_Slave_Init(uint8_t address) {
    // Configurar pines
    DDRC &= ~((1 << DDC4) | (1 << DDC5));
    
    // Dirección del esclavo
    TWI_AR = (address << 1) | 0x01; // Habilitar respuesta a general call
    
    // Limpiar estado previo y habilitar
    TWI_CR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
    
    // Esperar estabilización antes de habilitar interrupciones
    _delay_ms(10);
    
    // Habilitar interrupciones I2C
    TWI_CR |= (1 << TWIE);
}

// Función de escritura a registro - ROBUSTA
uint8_t I2C_WriteRegister(uint8_t device_addr, uint8_t reg_addr, uint8_t data) {
    uint8_t status;
    uint8_t retry_count = 0;
    
    while (retry_count < 3) {
        // START + dirección de escritura
        status = I2C_Start((device_addr << 1) | 0);
        if (status != I2C_SLA_W_ACK) {
            I2C_Stop();
            retry_count++;
            _delay_ms(10);
            continue;
        }
        
        // Dirección del registro
        status = I2C_Write(reg_addr);
        if (status != I2C_DATA_W_ACK) {
            I2C_Stop();
            retry_count++;
            _delay_ms(10);
            continue;
        }
        
        // Dato
        status = I2C_Write(data);
        if (status != I2C_DATA_W_ACK) {
            I2C_Stop();
            retry_count++;
            _delay_ms(10);
            continue;
        }
        
        I2C_Stop();
        return 0; // Éxito
    }
    
    return 1; // Error después de reintentos
}

// Función de lectura de registro - ROBUSTA
uint8_t I2C_ReadRegister(uint8_t device_addr, uint8_t reg_addr) {
    uint8_t status, data;
    uint8_t retry_count = 0;
    
    while (retry_count < 3) {
        // Fase de escritura (enviar registro)
        status = I2C_Start((device_addr << 1) | 0);
        if (status != I2C_SLA_W_ACK) {
            I2C_Stop();
            retry_count++;
            _delay_ms(10);
            continue;
        }
        
        status = I2C_Write(reg_addr);
        if (status != I2C_DATA_W_ACK) {
            I2C_Stop();
            retry_count++;
            _delay_ms(10);
            continue;
        }
        
        // Fase de lectura (repeated start)
        status = I2C_Start((device_addr << 1) | 1);
        if (status != I2C_SLA_R_ACK) {
            I2C_Stop();
            retry_count++;
            _delay_ms(10);
            continue;
        }
        
        data = I2C_ReadNACK();
        I2C_Stop();
        
        return data; // Éxito
    }
    
    return 0xFF; // Error después de reintentos
}

// Lectura múltiple - ROBUSTA
uint8_t I2C_ReadMultiple(uint8_t device_addr, uint8_t reg_addr, uint8_t* buffer, uint8_t length) {
    uint8_t status;
    uint8_t retry_count = 0;
    
    if (length == 0 || buffer == NULL) return 0;
    
    while (retry_count < 3) {
        // Escritura del registro
        status = I2C_Start((device_addr << 1) | 0);
        if (status != I2C_SLA_W_ACK) {
            I2C_Stop();
            retry_count++;
            _delay_ms(10);
            continue;
        }
        
        status = I2C_Write(reg_addr);
        if (status != I2C_DATA_W_ACK) {
            I2C_Stop();
            retry_count++;
            _delay_ms(10);
            continue;
        }
        
        // Lectura con repeated start
        status = I2C_Start((device_addr << 1) | 1);
        if (status != I2C_SLA_R_ACK) {
            I2C_Stop();
            retry_count++;
            _delay_ms(10);
            continue;
        }
        
        // Leer bytes
        for (uint8_t i = 0; i < length; i++) {
            if (i == length - 1) {
                buffer[i] = I2C_ReadNACK();
            } else {
                buffer[i] = I2C_ReadACK();
            }
            
            // Verificar timeout en cada lectura
            if (buffer[i] == 0xFF && i > 0 && buffer[i-1] != 0xFF) {
                // Posible timeout, abortar
                I2C_Stop();
                retry_count++;
                break;
            }
        }
        
        I2C_Stop();
        
        if (retry_count == 0 || buffer[0] != 0xFF) {
            return length; // Éxito
        }
        
        _delay_ms(10);
    }
    
    return 0; // Error después de reintentos
}

// Función para verificar estado del bus
uint8_t I2C_BusStatus(void) {
    // Verificar si SDA y SCL están altos (bus libre)
    if ((PINC & (1 << DDC4)) && (PINC & (1 << DDC5))) {
        return 0; // Bus libre
    } else {
        return 1; // Bus ocupado o bloqueado
    }
}

// Función para obtener contador de errores
uint8_t I2C_GetErrorCount(void) {
    return i2c_error_count;
}

// Función para reset manual del bus
void I2C_ForceRecovery(void) {
    bus_recovery_needed = 1;
    I2C_BusRecovery();
}