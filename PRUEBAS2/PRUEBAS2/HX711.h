#ifndef HX711_H
#define HX711_H

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// Configuración de pines para HX711
#define HX711_DOUT_PORT     PORTD
#define HX711_DOUT_DDR      DDRD
#define HX711_DOUT_PIN      PIND
#define HX711_DOUT_BIT      PD2

#define HX711_PD_SCK_PORT   PORTD
#define HX711_PD_SCK_DDR    DDRD
#define HX711_PD_SCK_BIT    PD3

// Ganancia del HX711
#define HX711_GAIN_128      1   // Canal A, ganancia 128
#define HX711_GAIN_32       2   // Canal B, ganancia 32
#define HX711_GAIN_64       3   // Canal A, ganancia 64

// Prototipos de funciones
void hx711_init(void);
uint8_t hx711_is_ready(void);
int32_t hx711_read(void);
int32_t hx711_read_average(uint8_t times);
void hx711_set_gain(uint8_t gain);
void hx711_power_down(void);
void hx711_power_up(void);
void hx711_tare(uint8_t times);
float hx711_get_units(uint8_t times);
void hx711_set_scale(float scale);

// Variables globales
extern int32_t hx711_offset;
extern float hx711_scale;

#endif