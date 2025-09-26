/*
 * VL53L0X_Fixed.c
 * 
 * LIBRER�A VL53L0X CORREGIDA - SIN BLOQUEOS
 * - Timeouts mejorados para prevenir bloqueos
 * - Manejo robusto de errores I2C
 * - Recuperaci�n autom�tica de errores
 * - Validaci�n exhaustiva de datos
 */

#include "VL53L0X_PB.h"

// Variables est�ticas para control de errores
static uint8_t last_error = VL53L0X_ERROR_NONE;
static uint8_t consecutive_errors = 0;
static uint8_t sensor_initialized = 0;

// Funci�n auxiliar para validar respuesta I2C
static uint8_t validate_i2c_response(uint8_t expected_mask, uint8_t received_value) {
    // Verificar que la respuesta no sea 0xFF (timeout) ni 0x00 (error com�n)
    if (received_value == 0xFF || received_value == 0x00) {
        return 0;
    }
    
    // Si hay m�scara espec�fica, validar
    if (expected_mask != 0x00) {
        return (received_value & expected_mask) != 0;
    }
    
    return 1; // V�lido
}

// Funci�n mejorada para escribir byte con validaci�n
uint8_t VL53L0X_WriteByte(uint8_t reg, uint8_t data) {
    uint8_t result;
    uint8_t retry_count = 0;
    
    while (retry_count < 3) {
        result = I2C_WriteRegister(VL53L0X_ADDRESS, reg, data);
        
        if (result == 0) {
            // Verificar que se escribi� correctamente leyendo de vuelta
            uint8_t read_back = VL53L0X_ReadByte(reg);
            if (read_back == data || reg == VL53L0X_REG_SYSRANGE_START) {
                // Algunos registros no se pueden leer de vuelta (como SYSRANGE_START)
                consecutive_errors = 0;
                last_error = VL53L0X_ERROR_NONE;
                return 1; // �xito
            }
        }
        
        retry_count++;
        _delay_ms(10);
    }
    
    consecutive_errors++;
    last_error = VL53L0X_ERROR_I2C;
    return 0; // Error
}

// Funci�n mejorada para leer byte con validaci�n
uint8_t VL53L0X_ReadByte(uint8_t reg) {
    uint8_t data;
    uint8_t retry_count = 0;
    
    while (retry_count < 3) {
        data = I2C_ReadRegister(VL53L0X_ADDRESS, reg);
        
        // Validar respuesta b�sica (no debe ser siempre 0xFF)
        if (data != 0xFF || retry_count == 2) {
            // En el �ltimo intento, aceptar cualquier valor
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

// Inicializaci�n robusta del sensor
uint8_t VL53L0X_Init(void) {
    uint8_t model_id;
    uint8_t retry_count = 0;
    
    last_error = VL53L0X_ERROR_NONE;
    consecutive_errors = 0;
    sensor_initialized = 0;
    
    // Intentar inicializaci�n hasta 3 veces
    while (retry_count < 3) {
        _delay_ms(50); // Pausa antes de cada intento
        
        // Verificar comunicaci�n leyendo el ID del modelo
        model_id = VL53L0X_ReadByte(VL53L0X_REG_IDENTIFICATION_MODEL_ID);
        
        if (model_id == 0xEE) {
            // Sensor encontrado, proceder con configuraci�n
            if (configure_sensor()) {
                sensor_initialized = 1;
                consecutive_errors = 0;
                last_error = VL53L0X_ERROR_NONE;
                return 1; // �xito
            }
        }
        
        retry_count++;
        
        // Si no es el �ltimo intento, hacer reset del I2C
        if (retry_count < 3) {
            I2C_ForceRecovery();
            _delay_ms(100);
        }
    }
    
    // Error despu�s de todos los intentos
    last_error = VL53L0X_ERROR_INIT;
    sensor_initialized = 0;
    return 0;
}

// Funci�n auxiliar para configurar el sensor
static uint8_t configure_sensor(void) {
    // Secuencia de configuraci�n del VL53L0X
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
        // Configuraci�n adicional para mayor robustez
        {0x60, 0x00},
        {0x61, 0xFF},
        {0x62, 0x00},
        {0x63, 0xFF},
        {VL53L0X_REG_SYSRANGE_START, 0x02}
    };
    
    const uint8_t config_length = sizeof(config_sequence) / sizeof(config_sequence[0]);
    
    for (uint8_t i = 0; i < config_length; i++) {
        if (!VL53L0X_WriteByte(config_sequence[i].reg, config_sequence[i].value)) {
            return 0; // Error en configuraci�n
        }
        _delay_ms(1); // Peque�a pausa entre escrituras
    }
    
    return 1; // Configuraci�n exitosa
}

// Funci�n mejorada para iniciar medici�n
void VL53L0X_StartMeasurement(void) {
    if (!sensor_initialized) {
        // Intentar reinicializar si es necesario
        if (!VL53L0X_Init()) {
            return;
        }
    }
    
    // Limpiar cualquier medici�n previa
    VL53L0X_WriteByte(VL53L0X_REG_RESULT_INTERRUPT_STATUS, 0x01);
    
    // Iniciar nueva medici�n
    VL53L0X_WriteByte(VL53L0X_REG_SYSRANGE_START, 0x01);
}

// Funci�n mejorada para verificar si los datos est�n listos
uint8_t VL53L0X_DataReady(void) {
    if (!sensor_initialized) return 0;
    
    uint8_t status = VL53L0X_ReadByte(VL53L0X_REG_RESULT_RANGE_STATUS);
    
    // Verificar bit de datos listos
    return (status & 0x01) != 0;
}

// Funci�n principal mejorada para leer distancia
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
    
    // Leer distancia usando funci�n robusta
    uint8_t distance_bytes[2];
    uint8_t bytes_read = I2C_ReadMultiple(VL53L0X_ADDRESS, 
                                          VL53L0X_REG_RESULT_RANGE_VAL, 
                                          distance_bytes, 2);
    
    if (bytes_read == 2) {
        distance = (distance_bytes[0] << 8) | distance_bytes[1];
        
        // Validar rango de distancia (VL53L0X: 30mm - 2000mm t�picamente)
        if (distance > 0 && distance <= 2000) {
            consecutive_errors = 0;
        } else if (distance > 2000) {
            // Distancia fuera de rango, pero sensor funciona
            distance = 2000; // Clampear al m�ximo
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
    
    // Limpiar flag de interrupci�n para pr�xima medici�n
    VL53L0X_WriteByte(VL53L0X_REG_RESULT_INTERRUPT_STATUS, 0x01);
    
    return distance;
}

// Funci�n para obtener �ltimo error
uint8_t VL53L0X_GetLastError(void) {
    return last_error;
}

// Funci�n para obtener n�mero de errores consecutivos
uint8_t VL53L0X_GetConsecutiveErrors(void) {
    return consecutive_errors;
}

// Funci�n para verificar si el sensor est� inicializado
uint8_t VL53L0X_IsInitialized(void) {
    return sensor_initialized;
}

// Funci�n para forzar reinicializaci�n
uint8_t VL53L0X_ForceReinit(void) {
    sensor_initialized = 0;
    consecutive_errors = 0;
    return VL53L0X_Init();
}

// Funci�n para diagn�stico del sensor
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

// Funci�n de recuperaci�n completa
uint8_t VL53L0X_FullRecovery(void) {
    // Recuperar bus I2C
    I2C_ForceRecovery();
    _delay_ms(100);
    
    // Reinicializar sensor
    return VL53L0X_ForceReinit();
}