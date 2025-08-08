/*
 * Esclavo 2.c
 *
 * Created: 1/08/2025 11:17:06
 * Author : valen
 */ 

#define F_CPU 16000000 //Frecuencia en la que opera el sistema - 16 MHz

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "ADC.h"
#include "I2C.h"

//Declaraciones
#define SlaveAddress 0x40

uint8_t buffer = 0;
uint16_t valueADC = 0;

//******************************************************************


int main(void)
{
	ADC_init();
	//UART_init();
	I2C_Slave_Init (SlaveAddress);
	
	sei();
	
    while (1) 
    {
		
		valueADC = ADC_read(6) >> 2;
		//uint8_to_string(valueADC);
		//UART_Char('\n');
		//_delay_ms(100);

		if (buffer == 'L')
		{
			buffer = 0;
		}
		
		
    }
}

ISR(TWI_vect){
	
	uint8_t estado;
	estado = TWSR & 0xFC;
	switch (estado){
		case  0x60:
		case 0x70:
			TWCR |= (1<<TWINT);
			break;
		case 0x80:
		case 0x90:
			buffer = TWDR;
			TWCR |= (1 << TWINT); //Bandera limpia			
			break;
		case 0xA8:
		case 0xB8:
			TWDR = valueADC;	
			TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA); //Envia
 			break;
		default:		
			TWCR |= (1 << TWINT) | (1 << TWSTO); //Libera errores
			break;
	}
	
}
