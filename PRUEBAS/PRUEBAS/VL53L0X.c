/*
 * VL53L0X.c
 *
 * Implementaci�n de la librer�a para sensor VL53L0X
 * Comunicaci�n I2C
 */ 

#include "VL53L0X.h"

// Inicializaci�n del sensor VL53L0X
uint8_t VL53L0X_Init(void) {
    uint8_t model_id;
    
    // Leer el ID del modelo para verificar comunicaci�n
    model_id = VL53L0X_ReadByte(VL53L0X_REG_IDENTIFICATION_MODEL_ID);
    
    if (model_id != 0xEE) {
        return 0; // Error: sensor no encontrado
    }
    
    // Configuraci�n b�sica del sensor
    VL53L0X_WriteByte(0x80, 0x01);
    VL53L0X_WriteByte(0xFF, 0x01);
    VL53L0X_WriteByte(0x00, 0x00);
    VL53L0X_WriteByte(0x91, 0x3C);
    VL53L0X_WriteByte(0x00, 0x01);
    VL53L0X_WriteByte(0xFF, 0x00);
    VL53L0X_WriteByte(0x80, 0x00);
    
    // Configurar para medici�n continua
    VL53L0X_WriteByte(VL53L0X_REG_SYSRANGE_START, 0x02);
    
    return 1; // Inicializaci�n exitosa
}

// Leer un byte de un registro espec�fico
uint8_t VL53L0X_ReadByte(uint8_t reg) {
    uint8_t data = 0;
    
    I2C_Master_Start();
    I2C_Master_Write((VL53L0X_ADDRESS << 1) | 0); // Direcci�n + Write
    I2C_Master_Write(reg); // Registro a leer
    
    I2C_Master_Start(); // Restart
    I2C_Master_Write((VL53L0X_ADDRESS << 1) | 1); // Direcci�n + Read
    I2C_Master_Read(&data, 0); // Leer sin ACK (�ltimo byte)
    I2C_Master_Stop();
    
    return data;
}

// Escribir un byte a un registro espec�fico
void VL53L0X_WriteByte(uint8_t reg, uint8_t data) {
    I2C_Master_Start();
    I2C_Master_Write((VL53L0X_ADDRESS << 1) | 0); // Direcci�n + Write
    I2C_Master_Write(reg); // Registro
    I2C_Master_Write(data); // Dato
    I2C_Master_Stop();
}

// Leer distancia en mil�metros
uint16_t VL53L0X_ReadDistance(void) {
    uint16_t distance = 0;
    uint8_t high_byte, low_byte;
    
    // Esperar a que la medici�n est� lista
    while (!VL53L0X_DataReady()) {
        _delay_ms(1);
    }
    
    // Leer los bytes de distancia (registro 0x14 + 10 y 0x14 + 11)
    high_byte = VL53L0X_ReadByte(0x1E);
    low_byte = VL53L0X_ReadByte(0x1F);
    
    distance = (high_byte << 8) | low_byte;
    
    // Limpiar el flag de interrupci�n
    VL53L0X_WriteByte(VL53L0X_REG_RESULT_INTERRUPT_STATUS, 0x01);
    
    return distance;
}

// Verificar si hay datos listos
uint8_t VL53L0X_DataReady(void) {
    uint8_t status = VL53L0X_ReadByte(VL53L0X_REG_RESULT_RANGE_STATUS);
    return (status & 0x01);
}

// Iniciar una nueva medici�n
void VL53L0X_StartMeasurement(void) {
    VL53L0X_WriteByte(VL53L0X_REG_SYSRANGE_START, 0x01);
}