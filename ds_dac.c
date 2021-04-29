/*
 * ds_dac.c
 *
 *  Created on: 1 lut 2017
 *      Author: Dawid Sobków
 */

#include <avr/io.h>
#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
//#include <string.h>
#include <util/delay.h>
//#include <stdlib.h>
#include <util/atomic.h>

#include "ds_dac.h"
#include "../I2C_TWI/i2c_twi.h"
#include "../DS_AKUT/ds_akuParam.h"

uint8_t used_LDAC;

volatile uint8_t int0_flag, int0_petla1=5, int0_petla2=5;

void init_DAC(){
	// Przerwanie INT0
	PORTD |= (1<<PD2);
	EIMSK |= (1<<INT0);				/* enable INT0 */
	EICRA |= (1<<ISC00)|(1<<ISC01);	/* INT0 - Rising edge */
//		EICRA |= (1<<ISC00);	/* INT0 - Falling edge */

	// LDAC 1 & 2
	DDRC |= (1<<PC4)|(1<<PC2);
	PORTC |= (1<<PC4)|(1<<PC2);

	//Aby przypisac adres I2C nale¿y uzyc tylko funkcji DAC_SetNewAddress() bez DAC_GeneralCallReadAddress() ta jest juz uzyta w tej pierwszej
	//DAC_GeneralCallReadAddress(LDAC2);
//	DAC_SetNewAddress(2, LDAC2); 	// DAC2
//	DAC_SetNewAddress(1, LDAC1);	// DAC1

	DAC_SequentialWriteCom(DAC1, DAC_UPDATE_VALUE, DAC1_DEF_VALUE); // musi byc wartosc wieksza od zera inaczej bez aku nie da sie ustawic napiecia
	DAC_SequentialWriteCom(DAC2, DAC_UPDATE_VALUE, DAC2_DEF_VALUE);
}

void SetVoltage(uint16_t value)
{
	uint16_t val = (value * AdcCountValues.adc_CV) / VOLTAGE_CALC;
	DAC_SingleWriteCom(DAC1, DAC_CHANNEL_A, DAC_UPDATE_VALUE, val);
}

void SetCharge(uint16_t value)
{
	uint16_t val = (value * AdcCountValues.adc_CC) / CHARGE_CALC;
	DAC_SingleWriteCom(DAC1, DAC_CHANNEL_C, DAC_UPDATE_VALUE, val);
}

void SetLoad(uint16_t value)
{
	uint16_t val = (value * AdcCountValues.adc_CL) / LOAD_CALC;
	DAC_SingleWriteCom(DAC2, DAC_CHANNEL_A, DAC_UPDATE_VALUE, val);	// Set LOAD
}

uint8_t DAC_GeneralCallReadAddress(uint8_t selectIC){
	uint8_t dac_Addr;

	if(!selectIC)
	used_LDAC = LDAC1;
	else
	used_LDAC = LDAC2;

	Set_LDAC(HIGH);
	int0_petla1=5;
	int0_petla2=3;

	TWI_start();
	TWI_write(0b00000000);
	TWI_write(0b00001100);
	TWI_start();
	TWI_write(0b11000001);
	dac_Addr = TWI_read(0);
	TWI_stop();

	return dac_Addr>>5;
}

uint8_t DAC_SetNewAddress(uint8_t newAdr, uint8_t selectIC){
	if(newAdr < 0 || newAdr > 7) return 255;

	char mask[2];
	char sendByte[4];

	uint8_t dac_Addr = DAC_GeneralCallReadAddress(selectIC);

	mask[0] = 0b00001110;
	mask[1] = 0b00011100;

	// 1st Byte
	sendByte[0] = 0b11000000;
	sendByte[0] |= mask[0] & dac_Addr<<1;
	// 2st Byte
	sendByte[1] = 0b01100001;
	sendByte[1] |= mask[1] & sendByte[0]<<1;
	// 3st Byte
	sendByte[2] = 0b01100010;
	sendByte[2] |= mask[1] & newAdr<<2;
	// 4st Byte
	sendByte[3] = 0b01100011;
	sendByte[3] |= mask[1] & newAdr<<2;

	WaitFor_RDYBSY();

	Set_LDAC(HIGH);
	int0_petla1=10;
	int0_petla2=3;

	TWI_start();
	TWI_write(sendByte[0]);
	TWI_write(sendByte[1]);
	TWI_write(sendByte[2]);
	TWI_write(sendByte[3]);
	TWI_stop();

	WaitFor_RDYBSY();

	dac_Addr = DAC_GeneralCallReadAddress(selectIC);

	return dac_Addr;
}

void DAC_SequentialWriteCom(uint8_t adr, uint8_t update, uint16_t value){

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
		if(value > 4095) value = 4095;
		else if(value < 0) value = 0;

		WaitFor_RDYBSY();

		uint8_t send_bit[4];
		send_bit[0] = 0b11000000;
		send_bit[1] = 0b01010000;
		send_bit[2] = 0b10010000;
		send_bit[3] = 0;

		send_bit[0] |= adr<<1;
		send_bit[1] |= update;
		send_bit[2] |= (uint8_t)(value>>8);
		send_bit[3] |= (uint8_t)value;

		//Write
		TWI_start();
		TWI_write(send_bit[0]);
		TWI_write(send_bit[1]);
		//CH A
		TWI_write(send_bit[2]);
		TWI_write(send_bit[3]);
		//CH B
		TWI_write(send_bit[2]);
		TWI_write(send_bit[3]);
		//CH C
		TWI_write(send_bit[2]);
		TWI_write(send_bit[3]);
		//CH D
		TWI_write(send_bit[2]);
		TWI_write(send_bit[3]);
		TWI_stop();
	}
}

void DAC_SingleWriteCom(uint8_t adr, uint8_t channel, uint8_t update, uint16_t value){

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
		if(value > 4095) value = 4095;
		else if(value < 0) value = 0;

		WaitFor_RDYBSY();

		uint8_t send_bit[4];
		send_bit[0] = 0b11000000;
		send_bit[1] = 0b01011000;
		send_bit[2] = 0b10010000;
		send_bit[3] = 0;

		send_bit[0] |= adr<<1;
		send_bit[1] |= channel;
		send_bit[1] |= update;
		send_bit[2] |= (uint8_t)(value>>8);
		send_bit[3] |= (uint8_t)value;

		//Write
		TWI_start();
		TWI_write(send_bit[0]);
		TWI_write(send_bit[1]);
		TWI_write(send_bit[2]);
		TWI_write(send_bit[3]);
		TWI_stop();
	}
}

void Set_LDAC(uint8_t out){
 if(used_LDAC == LDAC1)
	 if(!out)
		 PORTC &= ~(1<<PC4);
	 else
		 PORTC |= (1<<PC4);
 else
	 if(!out)
		 PORTC &= ~(1<<PC2);
	 else
		 PORTC |= (1<<PC2);
}

void WaitFor_RDYBSY( void ){
	if(used_LDAC == LDAC1)
		while(!(PINC & (1<<PC5)));
	else
		while(!(PINC & (1<<PC3)));
}

// przerwanie ustawiaj¹ce w odpowiednim momencie LDAC by wybrac scalak
ISR (INT0_vect) {

	int0_flag++;
	if(int0_flag>int0_petla1){
		for (uint8_t i = 0; i < int0_petla2; ++i) {
			asm("nop");
		}
//		asm("nop");
//		asm("nop");
//		asm("nop");
//		asm("nop");

		Set_LDAC(LOW);
		int0_flag=0;
	}
}
