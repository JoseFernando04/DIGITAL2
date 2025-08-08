#ifndef ADC_H
#define ADC_H

#include <avr/io.h>

// Definiciones para el ADC
#define ADC_VREF_MV     5000    // Voltaje de referencia en mV (5V)
#define ADC_RESOLUTION  1024    // Resolución del ADC (10 bits = 2^10)

// Canales ADC disponibles
#define ADC_CHANNEL_0   0       // A0 - PC0
#define ADC_CHANNEL_1   1       // A1 - PC1
#define ADC_CHANNEL_2   2       // A2 - PC2
#define ADC_CHANNEL_3   3       // A3 - PC3
#define ADC_CHANNEL_4   4       // A4 - PC4
#define ADC_CHANNEL_5   5       // A5 - PC5

// Prototipos de funciones
void adc_init(void);
uint16_t adc_read(uint8_t channel);
uint16_t adc_to_voltage_mv(uint16_t adc_value);
uint8_t adc_to_percentage(uint16_t adc_value);

#endif