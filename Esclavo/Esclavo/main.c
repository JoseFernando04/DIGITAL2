/*
 * Esclavo.c
 *
 */ 

#define F_CPU 16000000 //Frecuencia en la que opera el sistema - 16 MHz
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "I2C.h"

//Declaraciones
#define SlaveAddress 0x30
uint8_t buffer = 0;
uint8_t contador4bits = 0;  //Contador de 4 bits en 0 - Jugador 1
void initPorts(void);
void setup(void);
//******************************************************************
int main(void)
{
    
	initPorts();
	setup();
	I2C_Slave_Init (SlaveAddress);
	
	sei();
	
    while (1) 
    {
		
		if (buffer == 'R')
		{
			buffer = 0;
		}
		
		
    }
}
ISR(TWI0_vect){
	
	uint8_t estado;
	estado = TWSR0 & 0xFC;
	switch (estado){
		case  0x60:
		case 0x70:
			TWCR0 |= (1<<TWINT);
			break;
		case 0x80:
		case 0x90:
			buffer = TWDR0;
			TWCR0 |= (1 << TWINT); //Bandera limpia			
			break;
		case 0xA8:
		case 0xB8:
			//TWDR = valorADC;	
			TWDR0 = contador4bits;
			TWCR0 = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA); //Envia
 			break;
		default:		
			TWCR0 |= (1 << TWINT) | (1 << TWSTO); //Libera errores
			break;
	}
	
}
void initPorts(void){
	// Configurar A3-A0 (PC3-PC0) como salidas para el contador de 4 bits
	DDRC |= (1<< DDC0)|(1<< DDC1)|(1<< DDC2)|(1<< DDC3);
	PORTC = 0; // Inicializar en 0
	
	// Otras salidas (mantener las que no sean A3-A0)
	DDRD |= (1<< DDD4)|(1<< DDD5)|(1<< DDD6);
	PORTD = 0;
	DDRB |= (1<<DDB5);
}
void setup(void){ //Se utiliza void cuando no se emplean parámetros
	
	cli(); //Deshabilita interrupciones
		
	//Pines PD3, PD2 como entradas con pull-up
	DDRD &=  ~(1 << PIND2) & ~(1 << PIND3);
	PORTD |= (1 << PIND2) | (1 << PIND3);
	
	// Interrupción en los pines PD2 y PD3 (PCINT18 y PCINT19)
	PCICR |= (1 << PCIE2); // Habilita interrupciones en el puerto D (PCIE2)
	PCMSK2 |= (1 << PCINT18) | (1 << PCINT19); // Habilita interrupciones en PD2 y PD3
	sei(); // Habilita las interrupciones
		
	}
ISR(PCINT2_vect){
	
	if (!(PIND & (1 << PIND2))) {
		// Incrementar con overflow: 15 -> 0
		contador4bits++;
		contador4bits &= 0x0F; // Asegurar que solo use 4 bits (0-15)
		_delay_ms(250);
		// Mostrar contador en A3-A0 (PC3-PC0)
		PORTC = (PORTC & 0xF0) | (contador4bits & 0x0F);
	}
	else if (!(PIND & (1 << PIND3))) {
		// Decrementar con underflow: 0 -> 15
		if (contador4bits == 0) {
			contador4bits = 15; // Underflow: va a 15
		} else {
			contador4bits--;
		}
		_delay_ms(250);
		// Mostrar contador en A3-A0 (PC3-PC0)
		PORTC = (PORTC & 0xF0) | (contador4bits & 0x0F);
	}
}