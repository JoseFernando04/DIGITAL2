/*
 * Maestro.c
 *
 */ 

#define F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>
#include "LCD_8bits.h"
#include "I2C.h"

#define slave_1 0x30
#define slave_2 0x40

// Variables
uint8_t direccion;
uint8_t temp;
uint8_t bufferI2C;
uint8_t bufferI2C_2;
uint8_t valorI2C = 0;
uint8_t valorI2C_2 = 0;

int main(void)
{
	// Inicialización
	initLCD8();
	I2C_Master_Init(100000, 1);
	
	// Mensaje inicial en LCD
	LCD8_Clear();
	LCD8_Set_Cursor(0, 0);
	LCD8_Write_String("Sistema I2C");
	LCD8_Set_Cursor(0, 1);
	LCD8_Write_String("Iniciando...");
	_delay_ms(2000);
	
	while (1)
	{
		// _______________ ACTUALIZACIÓN DEL DISPLAY _______________
		LCD8_Clear(); // Limpiar pantalla para evitar caracteres residuales
		
		// Primera línea: Contador
		LCD8_Set_Cursor(0, 0);
		LCD8_Write_String("Contador: ");
		LCD8_Set_Cursor(4, 1);
		LCD8_Variable_U(valorI2C);
		
		// Segunda línea: ADC
		LCD8_Set_Cursor(11, 0);
		LCD8_Write_String("ADC: ");
		LCD8_Set_Cursor(12, 1);
		LCD8_Variable_U(valorI2C_2);
		
		_delay_ms(600);//peueña pausa para estabilidad del display
		
		//________________ COMUNICACIÓN I2C - CONTADOR _______________
		I2C_Master_Start();
		// Escritura
		bufferI2C = slave_1 << 1 & 0b11111110;
		temp = I2C_Master_Write(bufferI2C);
		if(temp != 1){
			I2C_Master_Stop();
			} else {
			I2C_Master_Write('R');
			I2C_Master_Stop();
		}
		
		_delay_ms(10);
		
		I2C_Master_Start();
		// Lectura
		bufferI2C = slave_1 << 1 | 0b00000001; // Bit menos significativo en 1 para leer
		temp = I2C_Master_Write(bufferI2C);
		if (temp != 1){
			I2C_Master_Stop();
			} else{
			TWCR0 = (1<<TWINT)|(1<<TWEN); // Activar pines de lectura
			while (!(TWCR0 & (1<<TWINT))); // Espera flag
			valorI2C = TWDR0;
			I2C_Master_Stop();
		}
		_delay_ms(10);
		
		//________________ COMUNICACIÓN I2C - ADC _______________
		I2C_Master_Start();
		// Escritura
		bufferI2C_2 = slave_2 << 1 & 0b11111110;
		temp = I2C_Master_Write(bufferI2C_2);
		if(temp != 1){
			I2C_Master_Stop();
			} else {
			I2C_Master_Write('L');
			I2C_Master_Stop();
		}
		
		_delay_ms(10);
		
		I2C_Master_Start();
		// Lectura
		bufferI2C_2 = slave_2 << 1 | 0b00000001; // Bit menos significativo en 1 para leer
		temp = I2C_Master_Write(bufferI2C_2);
		if (temp != 1){
			I2C_Master_Stop();
			} else{
			TWCR0 = (1<<TWINT)|(1<<TWEN); // Activar pines de lectura
			while (!(TWCR0 & (1<<TWINT))); // Espera flag
			valorI2C_2 = TWDR0;
			I2C_Master_Stop();
		}
		_delay_ms(10);
		
		// Pausa antes de la próxima actualización
		_delay_ms(100);
	}
}

