/*
 * I2C_Global.h
 *
 * Librería I2C universal compatible con ATmega328P y ATmega328PB
 * Detecta automáticamente el microcontrolador y usa los registros correctos
 *
 * Creado para resolver conflictos entre sensores VL53L0X y TCS3200
 */

#ifndef I2C_GLOBAL_H_
#define I2C_GLOBAL_H_

#include <avr/io.h>
#include <stdint.h>

/* ========================================================
   Detección automática del microcontrolador
   - ATmega328P usa registros:  TWBR, TWCR, TWDR, TWSR, TWAR
   - ATmega328PB usa registros: TWBR0, TWCR0, TWDR0, TWSR0, TWAR0
   ======================================================== */
#if defined(__AVR_ATmega328PB__)
    // ---- ATmega328PB ----
    #define TWI_BR      TWBR0
    #define TWI_CR      TWCR0
    #define TWI_DR      TWDR0
    #define TWI_SR      TWSR0
    #define TWI_AR      TWAR0
#else
    // ---- ATmega328P (normal) ----
    #define TWI_BR      TWBR
    #define TWI_CR      TWCR
    #define TWI_DR      TWDR
    #define TWI_SR      TWSR
    #define TWI_AR      TWAR
#endif

// ================== Estados I2C estándar ==================
#define I2C_START           0x08
#define I2C_REP_START       0x10
#define I2C_SLA_W_ACK       0x18
#define I2C_SLA_W_NACK      0x20
#define I2C_DATA_W_ACK      0x28
#define I2C_DATA_W_NACK     0x30
#define I2C_SLA_R_ACK       0x40
#define I2C_SLA_R_NACK      0x48
#define I2C_DATA_R_ACK      0x50
#define I2C_DATA_R_NACK     0x58

// ================== Prototipos de funciones ==================
void I2C_Init(uint32_t frequency);
uint8_t I2C_Start(uint8_t address);
void I2C_Stop(void);
uint8_t I2C_Write(uint8_t data);
uint8_t I2C_ReadACK(void);
uint8_t I2C_ReadNACK(void);
uint8_t I2C_GetStatus(void);

// --------- Funciones para esclavo ---------
void I2C_Slave_Init(uint8_t address);

// --------- Funciones de alto nivel ---------
uint8_t I2C_WriteRegister(uint8_t device_addr, uint8_t reg_addr, uint8_t data);
uint8_t I2C_ReadRegister(uint8_t device_addr, uint8_t reg_addr);
uint8_t I2C_ReadMultiple(uint8_t device_addr, uint8_t reg_addr, uint8_t* buffer, uint8_t length);

#endif /* I2C_GLOBAL_H_ */