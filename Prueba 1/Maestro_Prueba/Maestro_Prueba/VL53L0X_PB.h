/*
 * VL53L0X_PB.h
 * 
 * HEADER CORREGIDO PARA VL53L0X
 * - Funciones adicionales para manejo robusto
 * - Códigos de error mejorados
 * - Funciones de diagnóstico y recuperación
 */

#ifndef VL53L0X_PB_H_
#define VL53L0X_PB_H_

#include <avr/io.h>
#include <util/delay.h>
#include "I2C_Global.h"

// Dirección I2C del sensor VL53L0X
#define VL53L0X_ADDRESS 0x29

// Registros principales del VL53L0X
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         0xC0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID      0xC2
#define VL53L0X_REG_SYSRANGE_START                  0x00
#define VL53L0X_REG_RESULT_RANGE_STATUS            0x14
#define VL53L0X_REG_RESULT_RANGE_VAL               0x1E
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS         0x13

// Códigos de error mejorados
#define VL53L0X_ERROR_NONE          0x00
#define VL53L0X_ERROR_INIT          0x01
#define VL53L0X_ERROR_I2C           0x02
#define VL53L0X_ERROR_TIMEOUT       0x03
#define VL53L0X_ERROR_RANGE         0x04

// Funciones principales
uint8_t VL53L0X_Init(void);
uint8_t VL53L0X_ReadByte(uint8_t reg);
uint8_t VL53L0X_WriteByte(uint8_t reg, uint8_t data);
uint16_t VL53L0X_ReadDistance(void);
uint8_t VL53L0X_DataReady(void);
void VL53L0X_StartMeasurement(void);

// Funciones de diagnóstico y recuperación
uint8_t VL53L0X_GetLastError(void);
uint8_t VL53L0X_GetConsecutiveErrors(void);
uint8_t VL53L0X_IsInitialized(void);
uint8_t VL53L0X_ForceReinit(void);
uint8_t VL53L0X_DiagnoseHealth(void);
uint8_t VL53L0X_FullRecovery(void);

// Función auxiliar (declaración para uso interno)
static uint8_t configure_sensor(void);
static uint8_t validate_i2c_response(uint8_t expected_mask, uint8_t received_value);

#endif /* VL53L0X_PB_H_ */