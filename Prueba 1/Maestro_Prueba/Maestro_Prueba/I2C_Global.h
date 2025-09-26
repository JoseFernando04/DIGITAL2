/*
 * I2C_Global.h
 * 
 * Header file para librería I2C universal corregida
 * Compatible con ATMega328P y ATMega328PB
 */

#ifndef I2C_GLOBAL_H_
#define I2C_GLOBAL_H_

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// Definiciones de registros TWI para compatibilidad
#define TWI_BR      TWBR
#define TWI_SR      TWSR
#define TWI_AR      TWAR
#define TWI_DR      TWDR
#define TWI_CR      TWCR

// Códigos de estado I2C (Master Transmitter Mode)
#define I2C_START           0x08
#define I2C_REP_START       0x10
#define I2C_SLA_W_ACK       0x18
#define I2C_SLA_W_NACK      0x20
#define I2C_DATA_W_ACK      0x28
#define I2C_DATA_W_NACK     0x30
#define I2C_ARB_LOST        0x38

// Códigos de estado I2C (Master Receiver Mode)
#define I2C_SLA_R_ACK       0x40
#define I2C_SLA_R_NACK      0x48
#define I2C_DATA_R_ACK      0x50
#define I2C_DATA_R_NACK     0x58

// Códigos de estado I2C (Slave Receiver Mode)
#define I2C_SLA_R_RECV_ACK  0x60
#define I2C_ARB_LOST_SLA_R  0x68
#define I2C_GEN_CALL_ACK    0x70
#define I2C_ARB_LOST_GEN    0x78
#define I2C_SLA_DATA_ACK    0x80
#define I2C_SLA_DATA_NACK   0x88
#define I2C_GEN_DATA_ACK    0x90
#define I2C_GEN_DATA_NACK   0x98
#define I2C_STOP_RESTART    0xA0

// Códigos de estado I2C (Slave Transmitter Mode)
#define I2C_SLA_T_RECV_ACK  0xA8
#define I2C_ARB_LOST_SLA_T  0xB0
#define I2C_DATA_T_ACK      0xB8
#define I2C_DATA_T_NACK     0xC0
#define I2C_LAST_DATA_ACK   0xC8





// Funciones principales I2C
void I2C_Init(uint32_t frequency);
uint8_t I2C_Start(uint8_t address);
void I2C_Stop(void);
uint8_t I2C_Write(uint8_t data);
uint8_t I2C_ReadACK(void);
uint8_t I2C_ReadNACK(void);
uint8_t I2C_GetStatus(void);

// Funciones para modo esclavo
void I2C_Slave_Init(uint8_t address);

// Funciones de alto nivel
uint8_t I2C_WriteRegister(uint8_t device_addr, uint8_t reg_addr, uint8_t data);
uint8_t I2C_ReadRegister(uint8_t device_addr, uint8_t reg_addr);
uint8_t I2C_ReadMultiple(uint8_t device_addr, uint8_t reg_addr, uint8_t* buffer, uint8_t length);

// Funciones de utilidad y recuperación
void I2C_Recovery(void);
uint8_t I2C_BusReady(void);

#endif /* I2C_GLOBAL_H_ */