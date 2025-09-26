/*
 * ADC.h
 *
 * Created: 8/04/2025 06:51:46
 *  Author: valen
 */ 


#ifndef ADC_H_
#define ADC_H_

#include <avr/io.h>

void ADC_init(void);
uint16_t ADC_read(uint8_t canal);

#endif /* ADC_H_ */