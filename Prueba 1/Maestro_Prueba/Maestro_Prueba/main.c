/*
 * Maestro_I2C_Fixed.c
 * 
 * CÓDIGO MAESTRO CORREGIDO - SIN BLOQUEOS
 * - Gestión robusta de múltiples esclavos I2C
 * - Timeouts y recuperación de errores mejorados
 * - Prevención de bloqueos del bus
 */

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include "LCD_8bits.h"
#include "I2C_Global.h"
#include "VL53L0X_PB.h"

// Direcciones de esclavos I2C
#define SLAVE_TCS3200   0x20
#define SLAVE_BANDA     0x30
#define CMD_PREP        'C'
#define CMD_GET_WEIGHT  'W'
#define CMD_GET_STATE   'S'

// Variables globales
uint8_t color_code = 0;
uint16_t distance = 0;
float current_weight = 0.0f;
uint8_t banda_state = 0;
char buffer[20];

// Contadores de error mejorados
typedef struct {
    uint8_t error_count;
    uint8_t consecutive_errors;
    uint8_t device_available;
    uint32_t last_success_time;
} device_status_t;

device_status_t vl53_status = {0, 0, 1, 0};
device_status_t tcs_status = {0, 0, 1, 0};
device_status_t banda_status = {0, 0, 1, 0};

uint32_t system_tick = 0;

// Función para actualizar tick del sistema
void update_system_tick(void) {
    system_tick++;
}

// Función mejorada para verificar disponibilidad de dispositivo
uint8_t check_device_availability(uint8_t device_addr, device_status_t* status) {
    uint8_t result;
    
    // Intentar comunicación simple con el dispositivo
    result = I2C_Start((device_addr << 1) | 0);
    I2C_Stop();
    
    if (result == I2C_SLA_W_ACK) {
        status->consecutive_errors = 0;
        status->device_available = 1;
        status->last_success_time = system_tick;
        return 1;
    } else {
        status->consecutive_errors++;
        if (status->consecutive_errors > 5) {
            status->device_available = 0;
        }
        return 0;
    }
}

// Función de recuperación de I2C específica para cada dispositivo
void recover_device_communication(uint8_t device_addr, device_status_t* status) {
    // Verificar estado del bus antes de intentar recuperación
    if (I2C_BusStatus() != 0) {
        I2C_ForceRecovery();
        _delay_ms(50);
    }
    
    // Intentar reconectar con el dispositivo
    for (uint8_t i = 0; i < 3; i++) {
        if (check_device_availability(device_addr, status)) {
            break;
        }
        _delay_ms(100);
    }
}

// Función MEJORADA para leer color del TCS3200
uint8_t Read_TCS3200_Color(void) {
    uint8_t status;
    uint8_t color_data = 0;
    uint8_t retry_count = 0;
    
    // Verificar disponibilidad del dispositivo
    if (!tcs_status.device_available) {
        return 0xFF;
    }
    
    while (retry_count < 2) { // Reducir intentos para evitar bloqueos
        // PASO 1: Verificar que el dispositivo responde
        if (!check_device_availability(SLAVE_TCS3200, &tcs_status)) {
            retry_count++;
            _delay_ms(20);
            continue;
        }
        
        // PASO 2: Enviar comando de preparación
        status = I2C_Start((SLAVE_TCS3200 << 1) | 0);
        if (status != I2C_SLA_W_ACK) {
            I2C_Stop();
            tcs_status.error_count++;
            retry_count++;
            _delay_ms(20);
            continue;
        }
        
        status = I2C_Write(CMD_PREP);
        if (status != I2C_DATA_W_ACK) {
            I2C_Stop();
            tcs_status.error_count++;
            retry_count++;
            _delay_ms(20);
            continue;
        }
        
        I2C_Stop();
        _delay_ms(100); // Tiempo para que el esclavo procese
        
        // PASO 3: Leer respuesta
        status = I2C_Start((SLAVE_TCS3200 << 1) | 1);
        if (status != I2C_SLA_R_ACK) {
            I2C_Stop();
            tcs_status.error_count++;
            retry_count++;
            _delay_ms(20);
            continue;
        }
        
        color_data = I2C_ReadNACK();
        I2C_Stop();
        
        // Validar respuesta
        if (color_data <= 3) {
            tcs_status.error_count = 0;
            tcs_status.consecutive_errors = 0;
            return color_data;
        }
        
        retry_count++;
        _delay_ms(20);
    }
    
    // Error después de todos los intentos
    tcs_status.error_count++;
    tcs_status.consecutive_errors++;
    
    if (tcs_status.consecutive_errors > 10) {
        recover_device_communication(SLAVE_TCS3200, &tcs_status);
    }
    
    return 0xFF;
}

// Función MEJORADA para leer peso de la banda
float Read_Banda_Weight(void) {
    uint8_t status;
    uint8_t weight_bytes[4];
    uint8_t retry_count = 0;
    
    union {
        float f;
        uint8_t bytes[4];
    } weight_union;
    
    // Verificar disponibilidad del dispositivo
    if (!banda_status.device_available) {
        return -1.0f;
    }
    
    while (retry_count < 2) {
        // Verificar comunicación básica
        if (!check_device_availability(SLAVE_BANDA, &banda_status)) {
            retry_count++;
            _delay_ms(20);
            continue;
        }
        
        // Enviar comando para peso
        status = I2C_Start((SLAVE_BANDA << 1) | 0);
        if (status != I2C_SLA_W_ACK) {
            I2C_Stop();
            banda_status.error_count++;
            retry_count++;
            _delay_ms(20);
            continue;
        }
        
        status = I2C_Write(CMD_GET_WEIGHT);
        if (status != I2C_DATA_W_ACK) {
            I2C_Stop();
            banda_status.error_count++;
            retry_count++;
            _delay_ms(20);
            continue;
        }
        
        I2C_Stop();
        _delay_ms(50); // Tiempo para procesamiento
        
        // Leer respuesta usando función robusta
        if (I2C_ReadMultiple(SLAVE_BANDA, 0, weight_bytes, 4) == 4) {
            // Convertir bytes a float
            for (uint8_t i = 0; i < 4; i++) {
                weight_union.bytes[i] = weight_bytes[i];
            }
            
            // Validar rango del peso
            if (weight_union.f >= 0 && weight_union.f < 1000) {
                banda_status.error_count = 0;
                banda_status.consecutive_errors = 0;
                return weight_union.f;
            }
        }
        
        retry_count++;
        _delay_ms(20);
    }
    
    // Error después de todos los intentos
    banda_status.error_count++;
    banda_status.consecutive_errors++;
    
    if (banda_status.consecutive_errors > 10) {
        recover_device_communication(SLAVE_BANDA, &banda_status);
    }
    
    return -1.0f;
}

// Función MEJORADA para leer VL53L0X
uint16_t Read_VL53L0X_Distance(void) {
    uint16_t new_distance;
    
    VL53L0X_StartMeasurement();
    _delay_ms(50);
    
    new_distance = VL53L0X_ReadDistance();
    
    if (new_distance != 0xFFFF && VL53L0X_GetLastError() == VL53L0X_ERROR_NONE) {
        vl53_status.error_count = 0;
        vl53_status.consecutive_errors = 0;
        return new_distance;
    } else {
        vl53_status.error_count++;
        vl53_status.consecutive_errors++;
        
        // Recuperación específica para VL53L0X
        if (vl53_status.consecutive_errors > 5) {
            I2C_ForceRecovery();
            _delay_ms(100);
            if (VL53L0X_Init()) {
                vl53_status.consecutive_errors = 0;
            }
        }
        
        return 0;
    }
}

void Update_LCD_Display(void) {
    // LÍNEA SUPERIOR: Labels
    LCD8_Set_Cursor(0, 0);
    LCD8_Write_String("Nvl:  Wg:   Clr:");
    
    // LÍNEA INFERIOR: Valores
    LCD8_Set_Cursor(0, 1);
    
    // Distancia
    if (distance > 0 && distance < 2000) {
        sprintf(buffer, "%umm ", distance);
    } else {
        sprintf(buffer, "ERR  ");
    }
    LCD8_Write_String(buffer);
    
    // Peso
    LCD8_Set_Cursor(7, 1);
    if (current_weight >= 0 && current_weight < 999) {
        sprintf(buffer, "%dg ", (int)current_weight);
    } else {
        sprintf(buffer, "ERR ");
    }
    LCD8_Write_String(buffer);
    
    // Color
    LCD8_Set_Cursor(12, 1);
    switch(color_code) {
        case 0: LCD8_Write_String("NONE"); break;
        case 1: LCD8_Write_String("RED "); break;
        case 2: LCD8_Write_String("GRN "); break;
        case 3: LCD8_Write_String("BLU "); break;
        case 0xFF: LCD8_Write_String("ERR "); break;
        default: LCD8_Write_String("??? "); break;
    }
    
    // Mostrar estado de dispositivos en caso de errores
    if (!vl53_status.device_available || !tcs_status.device_available || !banda_status.device_available) {
        LCD8_Set_Cursor(0, 0);
        sprintf(buffer, "DEV:%d%d%d", 
                vl53_status.device_available, 
                tcs_status.device_available, 
                banda_status.device_available);
        LCD8_Write_String(buffer);
        _delay_ms(1000);
    }
}

int main(void) {
    // Inicialización
    initLCD8();
    I2C_Init(100000);
    
    LCD8_Clear();
    LCD8_Set_Cursor(0, 0);
    LCD8_Write_String("Sistema Industrial");
    LCD8_Set_Cursor(0, 1);
    LCD8_Write_String("Iniciando...");
    _delay_ms(2000);
    
    // Inicializar VL53L0X
    LCD8_Clear();
    LCD8_Set_Cursor(0, 0);
    LCD8_Write_String("Init VL53L0X...");
    
    if (VL53L0X_Init()) {
        LCD8_Set_Cursor(0, 1);
        LCD8_Write_String("VL53L0X OK!");
        vl53_status.device_available = 1;
    } else {
        LCD8_Set_Cursor(0, 1);
        LCD8_Write_String("VL53L0X ERROR!");
        vl53_status.device_available = 0;
    }
    
    _delay_ms(1500);
    
    // Detectar dispositivos disponibles en el bus
    LCD8_Clear();
    LCD8_Set_Cursor(0, 0);
    LCD8_Write_String("Detectando I2C...");
    
    check_device_availability(SLAVE_TCS3200, &tcs_status);
    check_device_availability(SLAVE_BANDA, &banda_status);
    
    LCD8_Set_Cursor(0, 1);
    sprintf(buffer, "TCS:%d BANDA:%d", 
            tcs_status.device_available, 
            banda_status.device_available);
    LCD8_Write_String(buffer);
    _delay_ms(2000);
    
    LCD8_Clear();
    
    uint8_t cycle_counter = 0;
    
    // Loop principal
    while (1) {
        update_system_tick();
        
        // ========== SENSOR DE DISTANCIA ==========
        distance = Read_VL53L0X_Distance();
        
        // ========== SENSOR DE COLOR (cada 3 ciclos) ==========
        if (cycle_counter % 3 == 0) {
            uint8_t new_color = Read_TCS3200_Color();
            if (new_color != 0xFF) {
                color_code = new_color;
            }
        }
        
        // ========== PESO DE BANDA (cada 5 ciclos) ==========
        if (cycle_counter % 5 == 0) {
            float new_weight = Read_Banda_Weight();
            if (new_weight >= 0) {
                current_weight = new_weight;
            }
        }
        
        // ========== VERIFICAR DISPOSITIVOS (cada 20 ciclos) ==========
        if (cycle_counter % 20 == 0) {
            // Verificar dispositivos no disponibles
            if (!tcs_status.device_available) {
                check_device_availability(SLAVE_TCS3200, &tcs_status);
            }
            if (!banda_status.device_available) {
                check_device_availability(SLAVE_BANDA, &banda_status);
            }
        }
        
        // ========== ACTUALIZAR DISPLAY ==========
        Update_LCD_Display();
        
        // Control de ciclo
        cycle_counter++;
        if (cycle_counter >= 60) cycle_counter = 0;
        
        _delay_ms(150); // Ciclo más lento para mayor estabilidad
    }
    
    return 0;
}