/*
 * st_ERROR.c
 *
 *  Created on: 22 mar 2017
 *      Author: Dawid Sobków
 */

#include <avr/io.h>

#include "ds_akut.h"
#include "ds_akuParam.h"
#include "../ADC/adc.h"
#include "../I2C_TWI/i2c_twi.h"
#include "../DS_DAC_MCP4728/ds_dac.h"
#include "../WS2812/ws281x.h"
#include "../common.h"

#define NOERROR 0
#define ISERROR 1

uint8_t err_lms10=255, err_ls1=255; // dla timerow
uint8_t err_t1=0, err_fx=0;	//licznik
uint8_t isError = NOERROR;

void DS_STATUS_ERROR( void ){
	if(err_ls1 != s1_cnt){ // co 1s
		err_ls1=s1_cnt;

		if(isError == NOERROR)
		{
			FX_SetActiveFX(BLINK, 20, 0, 0, 255, 255);

			DAC_SequentialWriteCom(DAC1, DAC_UPDATE_VALUE, DAC1_DEF_VALUE);
			DAC_SequentialWriteCom(DAC2, DAC_UPDATE_VALUE, DAC2_DEF_VALUE);
		}
		else if(isError == ISERROR)
		{
				if(!err_fx)
					FX_SetActiveFX(BLINK, 10, 0, RED, 255, 255);
				if(err_fx)
					FX_SetActiveFX(BLINK, 10, 0, BLUE, 255, 255);

				err_fx = !err_fx;

				Err_RestartCurrentTest(0);
		}
	}
}

//zapisuje do zmiennych kod bledu
void ST_ERROR(int8_t status, int8_t err){
	err_diferentERR++;

	if(err_status == status && err_code == err)
		err_sameERR++;
	else
		err_sameERR=0;

	err_status = status;
	err_code = err;

	if(err_nr >= 16)
		err_nr=0;

	err_List[err_nr][0] = err_status;
	err_List[err_nr][1] = err_code;

	err_nr++;
}

// Obsluga bledow *************************************************************************
void Err_RestartCurrentTest(uint8_t start){
	static uint8_t firstRun = 0;
	if(start)
	{
		err_t1=0;

		if(err_sameERR < 3)
		{
			firstRun = 1;
			isError=ISERROR;
			err_limit=0;
		}
		else
			err_limit=1;

		SetTestingStatus(ERROR);

		DAC_SequentialWriteCom(DAC1, DAC_UPDATE_VALUE, DAC1_DEF_VALUE);
		DAC_SequentialWriteCom(DAC2, DAC_UPDATE_VALUE, DAC2_DEF_VALUE);

		if(GetTestStatus() == DISCHARGE)
			SET_CHARGE
		else
			SET_LOAD
	}

	if(firstRun)
	{
		if(err_t1 > 5)
		{
			firstRun = 0;
			isError=NOERROR;
			FX_SetActiveFX(COLOR, 0, 0, 0, 0, 255); // bialy
			SetTestingStatus(err_status);
		}

		err_t1++;
	}
}
// Obsluga bledow KONIEC ******************************************************************
