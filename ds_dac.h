/*
 * ds_dac.h
 *
 *  Created on: 1 lut 2017
 *      Author: Dawid Sobków
 */

#ifndef DS_DAC_MCP4728_DS_DAC_H_
#define DS_DAC_MCP4728_DS_DAC_H_

#define LDAC1 0
#define LDAC2 1
#define HIGH  1
#define LOW   0

#define DAC1 1
#define DAC2 2
#define DAC_UPDATE_VALUE 0
#define DAC_DONT_UPDATE_VALUE 1
#define DAC_CHANNEL_A 0
#define DAC_CHANNEL_B 0x2
#define DAC_CHANNEL_C 0x4
#define DAC_CHANNEL_D 0x6

#define DAC1_DEF_VALUE 200
#define DAC2_DEF_VALUE 0

void SetVoltage(uint16_t value);
void SetCharge(uint16_t value);
void SetLoad(uint16_t value);

void init_DAC( void );
void DAC_SequentialWriteCom(uint8_t adr, uint8_t update, uint16_t value); // Write DAC Input Registers and EEPROM Sequentially from Starting Channel to Channel D
void DAC_SingleWriteCom(uint8_t adr, uint8_t channel, uint8_t update, uint16_t value); // Write to a Single DAC Input Register and EEPROM

uint8_t DAC_GeneralCallReadAddress(uint8_t selectIC);
uint8_t DAC_SetNewAddress(uint8_t newAdr, uint8_t selectIC);

void Set_LDAC(uint8_t out);
void WaitFor_RDYBSY( void );



#endif /* DS_DAC_MCP4728_DS_DAC_H_ */
