/*
 * Neopixel.h
 *
 *  Created on: May 14, 2025
 *      Author: pablo
 */

#ifndef INC_NEOPIXEL_H_
#define INC_NEOPIXEL_H_

#include "main.h"
#include "math.h"

//Numero de leds

#define numPixels 4


//valores de ancho de pulso del uno y el cero
#define CCR_0 34
#define CCR_1 67
//colocar el timer y canal usado
extern TIM_HandleTypeDef htim1;
//coloque el timer usado
#define neoPixel_timer htim1
#define neoPixel_canal TIM_CHANNEL_1

// funcion para aplicar el brillo con la funcion gamma
#define GAMMA_CORRECTION 2.2f  // Valor de corrección gamma adpatada al ojo humano
#define GAMMA 2.2f  // Valor de corrección gamma
#define MAX_BRIGHTNESS 255  // Valor máximo de brillo


void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b);
void setBrightness(uint8_t b);
void pixelShow(void);
void pixelClear(void);

uint8_t Gamma_correccion(uint8_t color, float brillo_);

#endif /* INC_NEOPIXEL_H_ */
