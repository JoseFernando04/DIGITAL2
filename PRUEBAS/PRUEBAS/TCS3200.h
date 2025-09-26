/*
 * TCS3200.h
 *
 */ 

#ifndef TCS3200_H
#define TCS3200_H

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>

// Definición de pines en ATmega328P (Arduino Nano)
#define TCS_S0   PD7
#define TCS_S1   PB0
#define TCS_S2   PD4
#define TCS_S3   PD2
#define TCS_OUT  PB4   // frecuencia de salida

// Prototipos de funciones
void tcs_init(void);
uint32_t tcs_read_frequency(uint8_t s2, uint8_t s3);
void tcs_get_color(char *color);

#endif