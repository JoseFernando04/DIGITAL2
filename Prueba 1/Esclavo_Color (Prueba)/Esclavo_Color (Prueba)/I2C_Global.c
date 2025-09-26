/*
 * I2C_Global.c
 * 
 * Implementación de librería I2C universal
 * Compatible con ATMega328P y ATMega328PB
 */

#include "I2C_Global.h"

#define F_CPU 16000000UL

// Inicializar I2C como maestro
void I2C_Init(uint32_t frequency) {
    // Configurar pines SDA y SCL como entradas (pull-ups externos requeridos)
    DDRC &= ~((1 << DDC4) | (1 << DDC5));
    
    // Calcular TWBR para la frecuencia deseada
    // Formula: SCL = F_CPU / (16 + 2 * TWBR * Prescaler)
    // Prescaler = 1, entonces: TWBR = (F_CPU / SCL - 16) / 2
    uint8_t twbr_value = ((F_CPU / frequency) - 16) / 2;
    
    // Asegurar que TWBR sea al menos 10 para operación estable
    if (twbr_value < 10) twbr_value = 10;
    
    TWI_BR = twbr_value;
    
    // Prescaler = 1 (TWPS1:0 = 00)
    TWI_SR &= ~((1 << TWPS1) | (1 << TWPS0));
    
    // Habilitar I2C
    TWI_CR = (1 << TWEN);
}

// Enviar condición START y dirección del esclavo
uint8_t I2C_Start(uint8_t address) {
    // Enviar START
    TWI_CR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWI_CR & (1 << TWINT)));
    
    // Verificar START enviado
    uint8_t status = I2C_GetStatus();
    if (status != I2C_START && status != I2C_REP_START) {
        return status;
    }
    
    // Enviar dirección del esclavo
    TWI_DR = address;
    TWI_CR = (1 << TWINT) | (1 << TWEN);
    while (!(TWI_CR & (1 << TWINT)));
    
    // Verificar ACK del esclavo
    status = I2C_GetStatus();
    return status;
}

// Enviar condición STOP
void I2C_Stop(void) {
    TWI_CR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    // Esperar a que STOP se complete
    while (TWI_CR & (1 << TWSTO));
}

// Escribir un byte de datos
uint8_t I2C_Write(uint8_t data) {
    TWI_DR = data;
    TWI_CR = (1 << TWINT) | (1 << TWEN);
    while (!(TWI_CR & (1 << TWINT)));
    
    return I2C_GetStatus();
}

// Leer un byte con ACK
uint8_t I2C_ReadACK(void) {
    TWI_CR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWI_CR & (1 << TWINT)));
    return TWI_DR;
}

// Leer un byte con NACK (último byte)
uint8_t I2C_ReadNACK(void) {
    TWI_CR = (1 << TWINT) | (1 << TWEN);
    while (!(TWI_CR & (1 << TWINT)));
    return TWI_DR;
}

// Obtener estado actual del bus I2C
uint8_t I2C_GetStatus(void) {
    return TWI_SR & 0xF8;
}

// Inicializar como esclavo I2C
void I2C_Slave_Init(uint8_t address) {
    // Configurar pines como entradas
    DDRC &= ~((1 << DDC4) | (1 << DDC5));
    
    // Configurar dirección del esclavo
    TWI_AR = address << 1;
    
    // Habilitar I2C, ACK automático e interrupciones
    TWI_CR = (1 << TWEN) | (1 << TWEA) | (1 << TWIE);
}

// Función de alto nivel: escribir a un registro
uint8_t I2C_WriteRegister(uint8_t device_addr, uint8_t reg_addr, uint8_t data) {
    uint8_t status;
    
    // START + dirección del dispositivo + W
    status = I2C_Start((device_addr << 1) | 0);
    if (status != I2C_SLA_W_ACK) {
        I2C_Stop();
        return status;
    }
    
    // Enviar dirección del registro
    status = I2C_Write(reg_addr);
    if (status != I2C_DATA_W_ACK) {
        I2C_Stop();
        return status;
    }
    
    // Enviar dato
    status = I2C_Write(data);
    if (status != I2C_DATA_W_ACK) {
        I2C_Stop();
        return status;
    }
    
    I2C_Stop();
    return 0; // Éxito
}

// Función de alto nivel: leer de un registro
uint8_t I2C_ReadRegister(uint8_t device_addr, uint8_t reg_addr) {
    uint8_t status, data;
    
    // START + dirección del dispositivo + W
    status = I2C_Start((device_addr << 1) | 0);
    if (status != I2C_SLA_W_ACK) {
        I2C_Stop();
        return 0xFF; // Error
    }
    
    // Enviar dirección del registro
    status = I2C_Write(reg_addr);
    if (status != I2C_DATA_W_ACK) {
        I2C_Stop();
        return 0xFF; // Error
    }
    
    // RESTART + dirección del dispositivo + R
    status = I2C_Start((device_addr << 1) | 1);
    if (status != I2C_SLA_R_ACK) {
        I2C_Stop();
        return 0xFF; // Error
    }
    
    // Leer dato (NACK porque es el último byte)
    data = I2C_ReadNACK();
    I2C_Stop();
    
    return data;
}

// Función de alto nivel: leer múltiples bytes
uint8_t I2C_ReadMultiple(uint8_t device_addr, uint8_t reg_addr, uint8_t* buffer, uint8_t length) {
    uint8_t status;
    
    if (length == 0) return 0;
    
    // START + dirección del dispositivo + W
    status = I2C_Start((device_addr << 1) | 0);
    if (status != I2C_SLA_W_ACK) {
        I2C_Stop();
        return 0;
    }
    
    // Enviar dirección del registro
    status = I2C_Write(reg_addr);
    if (status != I2C_DATA_W_ACK) {
        I2C_Stop();
        return 0;
    }
    
    // RESTART + dirección del dispositivo + R
    status = I2C_Start((device_addr << 1) | 1);
    if (status != I2C_SLA_R_ACK) {
        I2C_Stop();
        return 0;
    }
    
    // Leer bytes
    for (uint8_t i = 0; i < length; i++) {
        if (i == length - 1) {
            buffer[i] = I2C_ReadNACK(); // Último byte con NACK
        } else {
            buffer[i] = I2C_ReadACK();  // Bytes intermedios con ACK
        }
    }
    
    I2C_Stop();
    return length; // Número de bytes leídos
}
