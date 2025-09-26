/*
 * VL53L0X_Fixed.c
 * 
 * LIBRERÍA VL53L0X CORREGIDA - SIN BLOQUEOS
 * - Timeouts mejorados para prevenir bloqueos
 * - Manejo robusto de errores I2C
 * - Recuperación automática de errores
 * - Validación exhaustiva de datos
 */

#include "VL53L0X_PB.h"

// Variables estáticas para control de errores
static uint8_t last_error = VL53L0X_ERROR_NONE;
static uint8_t consecutive_errors = 0;
static uint8_t sensor_initialized = 0;

// Función auxiliar para validar respuesta I2C
static uint8_t validate_i2c_response(uint8_t expected_mask, uint8_t received_value) {
    // Verificar que la respuesta no sea 0xFF (timeout) ni 0x00 (error común)
    if (received_value == 0xFF || received_value == 0x00) {
        return 0;
    }
    
    // Si hay máscara específica, validar
    if (expected_mask != 0x00) {
        return (received_value & expected_mask) != 0;
    }
    
    return 1; // Válido
}

// Función mejorada para escribir byte con validación
uint8_t VL53L0X_WriteByte(uint8_t reg, uint8_t data) {
    uint8_t result;
    uint8_t retry_count = 0;
    
    while (retry_count < 3) {
        result = I2C_WriteRegister(VL53L0X_ADDRESS, reg, data);
        
        if (result == 0) {
            // Verificar que se escribió correctamente leyendo de vuelta
            uint8_t read_back = VL53L0X_ReadByte(reg);
            if (read_back == data || reg == VL53L0X_REG_SYSRANGE_START) {
                // Algunos registros no se pueden leer de vuelta (como SYSRANGE_START)
                consecutive_errors = 0;
                last_error = VL53L0X_ERROR_NONE;
                return 1; // Éxito
            }
        }
        
        retry_count++;
        _delay_ms(10);
    }
    
    consecutive_errors++;
    last_error = VL53L0X_ERROR_I2C;
    return 0; // Error
}

// Función mejorada para leer byte con validación
uint8_t VL53L0X_ReadByte(uint8_t reg) {
    uint8_t data;
    uint8_t retry_count = 0;
    
    while (retry_count < 3) {
        data = I2C_ReadRegister(VL53L0X_ADDRESS, reg);
        
        // Validar respuesta básica (no debe ser siempre 0xFF)
        if (data != 0xFF || retry_count == 2) {
            // En el último intento, aceptar cualquier valor
            consecutive_errors = 0;
            return data;
        }
        
        retry_count++;
        _delay_ms(10);
    }
    
    consecutive_errors++;
    last_error = VL53L0X_ERROR_I2C;
    return 0xFF; // Error
}

// Inicialización robusta del sensor
uint8_t VL53L0X_Init(void) {
    uint8_t model_id;
    uint8_t retry_count = 0;
    
    last_error = VL53L0X_ERROR_NONE;
    consecutive_errors = 0;
    sensor_initialized = 0;
    
    // Intentar inicialización hasta 3 veces
    while (retry_count < 3) {
        _delay_ms(50); // Pausa antes de cada intento
        
        // Verificar comunicación leyendo el ID del modelo
        model_id = VL53L0X_ReadByte(VL53L0X_REG_IDENTIFICATION_MODEL_ID);
        
        if (model_id == 0xEE) {
            // Sensor encontrado, proceder con configuración
            if (configure_sensor()) {
                sensor_initialized = 1;
                consecutive_errors = 0;
                last_error = VL53L0X_ERROR_NONE;
                return 1; // Éxito
            }
        }
        
        retry_count++;
        
        // Si no es el último intento, hacer reset del I2C
        if (retry_count < 3) {
            I2C_ForceRecovery();
            _delay_ms(100);
        }
    }
    
    // Error después de todos los intentos
    last_error = VL53L0X_ERROR_INIT;
    sensor_initialized = 0;
    return 0;
}

// Función auxiliar para configurar el sensor
static uint8_t configure_sensor(void) {
    // Secuencia de configuración del VL53L0X
    const struct {
        uint8_t reg;
        uint8_t value;
    } config_sequence[] = {
        {0x80, 0x01},
        {0xFF, 0x01},
        {0x00, 0x00},
        {0x91, 0x3C},
        {0x00, 0x01},
        {0xFF, 0x00},
        {0x80, 0x00},
        // Configuración adicional para mayor robustez
        {0x60, 0x00},
        {0x61, 0xFF},
        {0x62, 0x00},
        {0x63, 0xFF},
        {VL53L0X_REG_SYSRANGE_START, 0x02}
    };
    
    const uint8_t config_length = sizeof(config_sequence) / sizeof(config_sequence[0]);
    
    for (uint8_t i = 0; i < config_length; i++) {
        if (!VL53L0X_WriteByte(config_sequence[i].reg, config_sequence[i].value)) {
            return 0; // Error en configuración
        }
        _delay_ms(1); // Pequeña pausa entre escrituras
    }
    
    return 1; // Configuración exitosa
}

// Función mejorada para iniciar medición
void VL53L0X_StartMeasurement(void) {
    if (!sensor_initialized) {
        // Intentar reinicializar si es necesario
        if (!VL53L0X_Init()) {
            return;
        }
    }
    
    // Limpiar cualquier medición previa
    VL53L0X_WriteByte(VL53L0X_REG_RESULT_INTERRUPT_STATUS, 0x01);
    
    // Iniciar nueva medición
    VL53L0X_WriteByte(VL53L0X_REG_SYSRANGE_START, 0x01);
}

// Función mejorada para verificar si los datos están listos
uint8_t VL53L0X_DataReady(void) {
    if (!sensor_initialized) return 0;
    
    uint8_t status = VL53L0X_ReadByte(VL53L0X_REG_RESULT_RANGE_STATUS);
    
    // Verificar bit de datos listos
    return (status & 0x01) != 0;
}

// Función principal mejorada para leer distancia
uint16_t VL53L0X_ReadDistance(void) {
    uint16_t distance = 0;
    uint16_t timeout_ms = 0;
    const uint16_t MAX_TIMEOUT_MS = 200; // Timeout aumentado pero limitado
    
    last_error = VL53L0X_ERROR_NONE;
    
    if (!sensor_initialized) {
        last_error = VL53L0X_ERROR_INIT;
        return 0xFFFF;
    }
    
    // Esperar datos listos con timeout
    while (!VL53L0X_DataReady() && timeout_ms < MAX_TIMEOUT_MS) {
        _delay_ms(1);
        timeout_ms++;
        
        // Verificar cada 50ms si el sensor sigue respondiendo
        if (timeout_ms % 50 == 0) {
            uint8_t test_read = VL53L0X_ReadByte(VL53L0X_REG_IDENTIFICATION_MODEL_ID);
            if (test_read != 0xEE && test_read != 0xFF) {
                // Sensor no responde correctamente
                last_error = VL53L0X_ERROR_I2C;
                consecutive_errors++;
                return 0xFFFF;
            }
        }
    }
    
    if (timeout_ms >= MAX_TIMEOUT_MS) {
        last_error = VL53L0X_ERROR_TIMEOUT;
        consecutive_errors++;
        
        // Si hay muchos timeouts consecutivos, reinicializar
        if (consecutive_errors > 5) {
            sensor_initialized = 0;
        }
        
        return 0xFFFF;
    }
    
    // Leer distancia usando función robusta
    uint8_t distance_bytes[2];
    uint8_t bytes_read = I2C_ReadMultiple(VL53L0X_ADDRESS, 
                                          VL53L0X_REG_RESULT_RANGE_VAL, 
                                          distance_bytes, 2);
    
    if (bytes_read == 2) {
        distance = (distance_bytes[0] << 8) | distance_bytes[1];
        
        // Validar rango de distancia (VL53L0X: 30mm - 2000mm típicamente)
        if (distance > 0 && distance <= 2000) {
            consecutive_errors = 0;
        } else if (distance > 2000) {
            // Distancia fuera de rango, pero sensor funciona
            distance = 2000; // Clampear al máximo
            consecutive_errors = 0;
        } else {
            // Distancia = 0, posible error
            last_error = VL53L0X_ERROR_RANGE;
            consecutive_errors++;
            return 0xFFFF;
        }
    } else {
        last_error = VL53L0X_ERROR_I2C;
        consecutive_errors++;
        return 0xFFFF;
    }
    
    // Limpiar flag de interrupción para próxima medición
    VL53L0X_WriteByte(VL53L0X_REG_RESULT_INTERRUPT_STATUS, 0x01);
    
    return distance;
}

// Función para obtener último error
uint8_t VL53L0X_GetLastError(void) {
    return last_error;
}

// Función para obtener número de errores consecutivos
uint8_t VL53L0X_GetConsecutiveErrors(void) {
    return consecutive_errors;
}

// Función para verificar si el sensor está inicializado
uint8_t VL53L0X_IsInitialized(void) {
    return sensor_initialized;
}

// Función para forzar reinicialización
uint8_t VL53L0X_ForceReinit(void) {
    sensor_initialized = 0;
    consecutive_errors = 0;
    return VL53L0X_Init();
}

// Función para diagnóstico del sensor
uint8_t VL53L0X_DiagnoseHealth(void) {
    if (!sensor_initialized) return 0;
    
    // Leer registros importantes para verificar estado
    uint8_t model_id = VL53L0X_ReadByte(VL53L0X_REG_IDENTIFICATION_MODEL_ID);
    uint8_t revision_id = VL53L0X_ReadByte(0xC2); // Revision ID
    uint8_t status = VL53L0X_ReadByte(VL53L0X_REG_RESULT_RANGE_STATUS);
    
    // Verificar que las lecturas sean coherentes
    if (model_id == 0xEE && revision_id != 0xFF && status != 0xFF) {
        consecutive_errors = 0;
        return 1; // Sensor saludable
    }
    
    consecutive_errors++;
    return 0; // Sensor con problemas
}

// Función de recuperación completa
uint8_t VL53L0X_FullRecovery(void) {
    // Recuperar bus I2C
    I2C_ForceRecovery();
    _delay_ms(100);
    
    // Reinicializar sensor
    return VL53L0X_ForceReinit();
}