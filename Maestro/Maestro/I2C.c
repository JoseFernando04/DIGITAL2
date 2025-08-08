/*
 * I2C.c
 *
 * Created: 1/08/2025 07:14:56
 *  Author: valen
 */ 

#include "I2C.h"
//***************************************************************
// Funcion para inicializar I2C Maestro
//***************************************************************
void I2C_Master_Init(unsigned long SCL_Clock, uint8_t Prescaler) {

    DDRC &= ~((1 << DDC4) | (1 << DDC5));  // Pines de I2C como entradas

    switch (Prescaler) {
        case 1:
            TWSR0 &= ~((1 << TWPS1) | (1 << TWPS0));
            break;
        case 4:
            TWSR0 &= ~(1 << TWPS1);
            TWSR0 |= (1 << TWPS0);
            break;
        case 16:
            TWSR0 &= ~(1 << TWPS0);
            TWSR0 |= (1 << TWPS1);
            break;
        case 64:
            TWSR0 |= (1 << TWPS1) | (1 << TWPS0);
            break;
		default:
			TWSR0 &= ~((1 << TWPS1) | (1 << TWPS0));
			Prescaler=1;
			break;

    }
	TWBR0 = (((16000000)/SCL_Clock)-16)/(2*Prescaler); //Debe ser mayor a 10 para operar de forma estable
	TWCR0 |= (1<<TWEN);
}

//************************************************************************
// Funcion de inicio de la comunicacion I2C
//************************************************************************
void I2C_Master_Start(void){
    //uint8_t estado;
    
    TWCR0 = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // Iniciar condición de start
    while(!(TWCR0 & (1<<TWINT))); // Espera a que termine la flag TWINT

}

//************************************************************************
// Funcion de parada de la comunicacion I2C
//************************************************************************
void I2C_Master_Stop(void){
    TWCR0 = (1<<TWEN) | (1<<TWINT) | (1<<TWSTO); // Inicia el envío secuencia parada STOP
}

//************************************************************************
// Función de transmisión de datos del maestro al esclavo
// Retorna 1 si hubo un error o 0 si el esclavo ha recibido
// el dato
//************************************************************************
uint8_t I2C_Master_Write(uint8_t dato){
    uint8_t estado;

    TWDR0 = dato;  // Cargar el dato
    TWCR0 = (1<<TWEN) | (1<<TWINT); // Inicia el envío

    while(!(TWCR0 & (1<<TWINT))); // Espera al flag TWINT

    estado = TWSR0 & 0xF8; // Verificar estado

    // Verificar si se transmitió una SLA + W con ACK, SLA + R con ACK, o un Dato con ACK
    if(estado == 0x18 || estado == 0x28 || estado == 0x40){
        return 1;
    }else{
        return estado;
    }
}

//************************************************************************
// Función de recepción de datos enviados por el esclavo al maestro
// Esta función es para leer los datos que están en el esclavo
//************************************************************************
uint8_t I2C_Master_Read(uint8_t *buffer, uint8_t ack){
    uint8_t estado;
    
    if(ack){
        TWCR0 |= (1<<TWEA);  // Lectura con ACK
    }else{
        TWCR0 &= ~(1<<TWEA); // Lectura sin ACK
    }

    TWCR0 |= (1<<TWINT); // Iniciamos la lectura
    while(!(TWCR0 & (1<<TWINT))); // Espera al flag TWINT

    estado = TWSR0 & 0xF8; // Verificar estado

    // Verificar dato leído con ACK o sin ACK
    if(estado == 0x58 || estado == 0x50){
        *buffer = TWDR0;
        return 1;
    }else{
        return estado;
    }
}

//*****************************************************************************
// Funcion para inicializar I2C Esclavo
//*****************************************************************************

void I2C_Slave_Init(uint8_t address) {
    DDRC &= ~((1<<DDC4)|(1<<DDC5));  // Pines de I2C como entradas

    TWAR0 = address << 1;  // Se asigna la direccion que tendra
    // TWAR = (address << 1 | 0x01);  // Se asigna la direccion que tendra y habilita llamada gen

    // Se habilita la interfaz, ACK automático, se habilita la ISR
    TWCR0 = (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
}