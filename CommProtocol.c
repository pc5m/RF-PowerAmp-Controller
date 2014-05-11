/*
 * CommProtocol.c
 *
 * Created: 8-5-2014 1:18:14
 *  Author: pc5m
 */ 

#include "CommProtocol.h"
#include "avr/io.h"
#include "uart.h"
#include "generalDefine.h"

Com_Message_struct Com_Message_Rx;		//Struct which contains data of the last received message


void comm_RX_process(void)
{
	static uint8_t state;
	static uint8_t data_bytes_to_read;
	uint16_t temp;

	while ((temp = uart_getc())!= UART_NO_DATA){
		if(temp&0xFF00)
			return;
		else
			temp &= 0x00FF;
	}

	switch(state)
	{
		case UNKNOWN:
			if(temp == SOF)
				state = SOF_RECEIVED;
			break;
		case SOF_RECEIVED:
			if(temp == SYNC)
				state = SYNC_RECEIVED;
			else
				state = UNKNOWN;
			break;
		case SYNC_RECEIVED:
			Com_Message_Rx.id = (uint8_t)temp;
			state = ID_RECEIVED;
			break;
		case ID_RECEIVED:
			Com_Message_Rx.dlc = (uint8_t)temp;
			state = DLC_RECEIVED;
			data_bytes_to_read = Com_Message_Rx.dlc;
			if(data_bytes_to_read == 0)
				state = EOF;
			break;
		case DLC_RECEIVED:
			Com_Message_Rx.data[Com_Message_Rx.dlc - data_bytes_to_read] = (uint8_t)temp;
			data_bytes_to_read--;
			if(data_bytes_to_read == 0)
				state = EOF;
			break;
		case EOF:
			if (temp == EOFSYNC)
				state = EOFSYNC_RECEIVED;
			else
				state = UNKNOWN;
		break;
		default:
			state = UNKNOWN;
		break;
	}

	if(state == EOFSYNC_RECEIVED)
	{
		switch(Com_Message_Rx.id)
		{
			case PC_ID_REQ_FW_VERSION:
				comm_tx_version();
				break;
			case  PC_ID_SET_STATUS_AUTOTX_STATUS:
								// insert function //
								break;
			case  PC_ID_SET_STATUS_AUTOTX_CURRENTS_ADC:
								// insert function //
								break;
			case  PC_ID_SET_STATUS_AUTOTX_CURRENTS_VALS:
								// insert function //
								break;
			case  PC_ID_REQ_CURRENT_TRIP_ADC:
								// insert function //
								break;
			case  PC_ID_REQ_CURRENT_TRIP_VAL:
								// insert function //
								break;
			case  PC_ID_SET_CURRENT_TRIP_ADC:
								// insert function //
								break;
			case  PC_ID_SET_CURRENT_TRIP_VAL:
								// insert function //
								break;
			case  PC_ID_SET_CAL_CURRENTS_AMP2ADC:
								// insert function //
								break;
			case  PC_ID_SET_CAL_CURRENTS_ADC2AMP:
								// insert function //
								break;
			case  PC_ID_SET_STATUS_AUTOTX_POWERS_ADC:
								// insert function //
								break;
			case  PC_ID_SET_STATUS_AUTOTX_POWERS_VALS:
								// insert function //
								break;
			case  PC_ID_REQ_POWERS_TRIP_ADC:
								// insert function //
								break;
			case  PC_ID_REQ_POWERS_TRIP_VALS:
								// insert function //
								break;
			case  PC_ID_SET_POWERS_TRIP_ADC:
								// insert function //
								break;
			case  PC_ID_SET_POWERS_TRIP_VAL:
								// insert function //
								break;
			case  PC_ID_SET_CAL_POWERS_W2ADC:
								// insert function //
								break;
			case  PC_ID_SET_CAL_POWERS_ADC2W:
								// insert function //
								break;
			case  PC_ID_REQ_TEMPS:
								// insert function //
								break;
			case  PC_ID_REQ_TEMP_TRIP:
								// insert function //
								break;
			case  PC_ID_SET_TEMP_TRIP:
								// insert function //
								break;
			default:
			break;
		}
		state = UNKNOWN;
		Com_Message_Rx.id = 0xFF;
	}
}

/*************************************************************************
Function: comm_tx_version()
Purpose:  Transmit the version number of firmware
Input:    none
Returns:  none
**************************************************************************/
void comm_tx_version(){
	#define BYTES 1  // nr of bytes 1 * 1;
	uint8_t txBuffer[BYTES+5];
	uint8_t i;
	txBuffer[0] = SOF;
	txBuffer[1] = SYNC;
	txBuffer[2] = MC_ID_FW_VERSION;
	txBuffer[3] = BYTES;
	txBuffer[4] = FW_VERSION_NR;
	txBuffer[BYTES+4] = EOFSYNC;
	for (i=0;i<=BYTES+4;i++)
	{
		uart_putc(txBuffer[i]);
	}
}


/*************************************************************************
Function: uart_tx_status()
Purpose:  Transmit the status of TX/RX, PSU On/Off, Error/Trip times 1 bytes (uint8_t)
Input:    none
Returns:  none
**************************************************************************/
void uart_tx_status(){
	#define BYTES 1  // nr of bytes 1 * 1;
	uint8_t txBuffer[BYTES+5];
	uint8_t i;
	txBuffer[0] = SOF;
	txBuffer[1] = SYNC;
	txBuffer[2] = MC_ID_STATUS;
	txBuffer[3] = BYTES;
	txBuffer[4] = SSPAstatus;
	txBuffer[BYTES+4] = EOFSYNC;
	for (i=0;i<=BYTES+4;i++)
	{
		uart_putc(txBuffer[i]);
	}
}

/*************************************************************************
Function: uart_tx_temperatures()
Purpose:  Transmit the actual temperature values via UART
Input:    none
Returns:  none
**************************************************************************/
void uart_tx_temperatures()
{
	uint8_t txBuffer[9];
	uint8_t i;
	txBuffer[0] = SOF;
	txBuffer[1] = SYNC;
	txBuffer[2] = MC_ID_TEMPERATURE;
	txBuffer[3] = 4; // nr of bytes 4 * 1;
	txBuffer[4] = temp_values.tempA;
	txBuffer[5] = temp_values.tempB;
	txBuffer[6] = temp_values.tempC;
	txBuffer[7] = temp_values.tempD;
	txBuffer[8] = EOFSYNC;
	for (i=0;i<=8;i++)
	{
		uart_putc(txBuffer[i]);
	}
}

/*************************************************************************
Function: uart_tx_temperatureTrip()
Purpose:  Transmit the temperature trip value via UART as 1 times 1 bytes (uint8_t)
Input:    none
Returns:  none
**************************************************************************/
void uart_tx_temperatureTrip(){
	#define BYTES 1 // nr of bytes 1 * 1;
	uint8_t txBuffer[BYTES+5];
	uint8_t i;
	unsigned char *p;
	txBuffer[0] = SOF;
	txBuffer[1] = SYNC;
	txBuffer[2] = MC_ID_TEMPERATURE_TRIP;
	txBuffer[3] = BYTES;
	p = (unsigned char *)&trip_values.temp_trip;
	for(i=0; i<1; i++){ txBuffer[4+i] = *p++ ;}
	txBuffer[BYTES+4] = EOFSYNC;
	for (i=0;i<=BYTES+4;i++)
	{
		uart_putc(txBuffer[i]);
	}
}

/*************************************************************************
Function: uart_tx_current vals()
Purpose:  Transmit the actual current values via UART as 4 times 4 bytes (float)
Input:    none
Returns:  none
**************************************************************************/
void uart_tx_currentVals()
{
	uint8_t txBuffer[21];
	uint8_t i;
	unsigned char *p;
	txBuffer[0] = SOF;
	txBuffer[1] = SYNC;
	txBuffer[2] = MC_ID_CURRENT_VALS;
	txBuffer[3] = 16; // nr of bytes 4 * 4;
	p = (unsigned char *)&current.moduleA;
	for(i=0; i<4; i++){ txBuffer[4+i] = *p++ ;}
	p = (unsigned char *)&current.moduleB;
	for(i=0; i<4; i++){ txBuffer[8+i] = *p++ ;}
	p = (unsigned char *)&current.moduleC;
	for(i=0; i<4; i++){ txBuffer[12+i] = *p++ ;}
	p = (unsigned char *)&current.moduleD;
	for(i=0; i<4; i++){ txBuffer[16+i] = *p++ ;}
	txBuffer[20] = EOFSYNC;
	for (i=0;i<=20;i++)
	{
		uart_putc(txBuffer[i]);
	}
}


/*************************************************************************
Function: uart_tx_currentADCVals()
Purpose:  Transmit the current ADC values via UART as 4 times 2 bytes (uint16t)
Input:    none
Returns:  none
**************************************************************************/
void uart_tx_currentADCVals(){
	uint8_t txBuffer[13];
	uint8_t i;
	unsigned char *p;
	txBuffer[0] = SOF;
	txBuffer[1] = SYNC;
	txBuffer[2] = MC_ID_CURRENT_ADC_VALS;
	txBuffer[3] = 8; // nr of bytes 4 * 2;
	p = (unsigned char *)&adc_values.iModuleA_ADC;
	for(i=0; i<2; i++){ txBuffer[4+i] = *p++ ;}
	p = (unsigned char *)&adc_values.iModuleB_ADC;
	for(i=0; i<2; i++){ txBuffer[6+i] = *p++ ;}
	p = (unsigned char *)&adc_values.iModuleC_ADC;
	for(i=0; i<2; i++){ txBuffer[8+i] = *p++ ;}
	p = (unsigned char *)&adc_values.iModuleD_ADC;
	for(i=0; i<2; i++){ txBuffer[10+i] = *p++ ;}
	txBuffer[12] = EOFSYNC;
	for (i=0;i<=12;i++)
	{
		uart_putc(txBuffer[i]);
	}
}

/*************************************************************************
Function: uart_tx_currentTripADC()
Purpose:  Transmit the current Trip ADC values via UART as 4 times 2 bytes (uint16t)
Input:    none
Returns:  none
**************************************************************************/
void uart_tx_currentTripADC(){
	uint8_t txBuffer[13];
	uint8_t i;
	unsigned char *p;
	txBuffer[0] = SOF;
	txBuffer[1] = SYNC;
	txBuffer[2] = MC_ID_CURRENT_TRIP_ADC;
	txBuffer[3] = 8; // nr of bytes 4 * 2;
	p = (unsigned char *)&trip_values.ImodA_trip_ADC;
	for(i=0; i<2; i++){ txBuffer[4+i] = *p++ ;}
	p = (unsigned char *)&trip_values.ImodB_trip_ADC;
	for(i=0; i<2; i++){ txBuffer[6+i] = *p++ ;}
	p = (unsigned char *)&trip_values.ImodC_trip_ADC;
	for(i=0; i<2; i++){ txBuffer[8+i] = *p++ ;}
	p = (unsigned char *)&trip_values.ImodD_trip_ADC;
	for(i=0; i<2; i++){ txBuffer[10+i] = *p++ ;}
	txBuffer[12] = EOFSYNC;
	for (i=0;i<=12;i++)
	{
		uart_putc(txBuffer[i]);
	}
}

/*************************************************************************
Function: uart_tx_currentTripVal()
Purpose:  Transmit the current Trip values via UART as 1 times 2 bytes (uint16_t)
Input:    none
Returns:  none
**************************************************************************/
void uart_tx_currentTripVal(){
	uint8_t txBuffer[7];
	uint8_t i;
	unsigned char *p;
	txBuffer[0] = SOF;
	txBuffer[1] = SYNC;
	txBuffer[2] = MC_ID_CURRENT_TRIP_VAL;
	txBuffer[3] = 2; // nr of bytes 1 * 2;
	p = (unsigned char *)&trip_values.Imod_trip_AMP;
	for(i=0; i<2; i++){ txBuffer[4+i] = *p++ ;}
	txBuffer[6] = EOFSYNC;
	for (i=0;i<=6;i++)
	{
		uart_putc(txBuffer[i]);
	}
}

/*************************************************************************
Function: uart_tx_currentCalibrationADC2Amp()
Purpose:  Transmit the ADC to current calibration factors via UART as 4 times 4 bytes (float)
Input:    none
Returns:  none
**************************************************************************/
void uart_tx_currentCalibrationADC2Amp(){
	uint8_t txBuffer[21];
	uint8_t i;
	unsigned char *p;
	txBuffer[0] = SOF;
	txBuffer[1] = SYNC;
	txBuffer[2] = MC_ID_CURRENT_CAL_ADC2AMP;
	txBuffer[3] = 16; // nr of bytes 4 * 4;
	p = (unsigned char *)&cal_values.ImodA_ADC2AMP;
	for(i=0; i<4; i++){ txBuffer[4+i] = *p++ ;}
	p = (unsigned char *)&cal_values.ImodB_ADC2AMP;
	for(i=0; i<4; i++){ txBuffer[8+i] = *p++ ;}
	p = (unsigned char *)&cal_values.ImodC_ADC2AMP;
	for(i=0; i<4; i++){ txBuffer[12+i] = *p++ ;}
	p = (unsigned char *)&cal_values.ImodD_ADC2AMP;
	for(i=0; i<4; i++){ txBuffer[16+i] = *p++ ;}
	txBuffer[20] = EOFSYNC;
	for (i=0;i<=20;i++)
	{
		uart_putc(txBuffer[i]);
	}
}

/*************************************************************************
Function: uart_tx_current vals()
Purpose:  Transmit the current to ADC calibration factors via UART as 4 times 4 bytes (float)
Input:    none
Returns:  none
**************************************************************************/
void uart_tx_currentCalibrationAmp2ADC(){
	uint8_t txBuffer[21];
	uint8_t i;
	unsigned char *p;
	txBuffer[0] = SOF;
	txBuffer[1] = SYNC;
	txBuffer[2] = MC_ID_CURRENT_CAL_AMP2ADC;
	txBuffer[3] = 16; // nr of bytes 4 * 4
	p = (unsigned char *)&cal_values.ImodA_AMP2ADC;
	for(i=0; i<4; i++){ txBuffer[4+i] = *p++ ;}
	p = (unsigned char *)&cal_values.ImodB_AMP2ADC;
	for(i=0; i<4; i++){ txBuffer[8+i] = *p++ ;}
	p = (unsigned char *)&cal_values.ImodC_AMP2ADC;
	for(i=0; i<4; i++){ txBuffer[12+i] = *p++ ;}
	p = (unsigned char *)&cal_values.ImodD_AMP2ADC;
	for(i=0; i<4; i++){ txBuffer[16+i] = *p++ ;}
	txBuffer[20] = EOFSYNC;
	for (i=0;i<=20;i++)
	{
		uart_putc(txBuffer[i]);
	}
}


/*************************************************************************
Function: uart_tx_powerADCVals()
Purpose:  Transmit the current power ADC values via UART as 3 times 2 bytes (uint16_t)
Input:    none
Returns:  none
**************************************************************************/
void uart_tx_powerADCVals(){
	#define BYTES 6  // nr of bytes 3 * 2;
	uint8_t txBuffer[BYTES+5];
	uint8_t i;
	unsigned char *p;
	txBuffer[0] = SOF;
	txBuffer[1] = SYNC;
	txBuffer[2] = MC_ID_POWER_ADC_VALS;
	txBuffer[3] = BYTES;
	p = (unsigned char *)&adc_values.pwrFwrd_ADC;
	for(i=0; i<2; i++){ txBuffer[4+i] = *p++ ;}
	p = (unsigned char *)&adc_values.pwrRefl_ADC;
	for(i=0; i<2; i++){ txBuffer[6+i] = *p++ ;}
	p = (unsigned char *)&adc_values.pwrIn_ADC;
	for(i=0; i<2; i++){ txBuffer[8+i] = *p++ ;}
	txBuffer[BYTES+4] = EOFSYNC;
	for (i=0;i<=BYTES+4;i++)
	{
		uart_putc(txBuffer[i]);
	}
}

/*************************************************************************
Function: uart_tx_powerTripADC()
Purpose:  Transmit the current power trip ADC values via UART as 3 times 2 bytes (uint16_t)
Input:    none
Returns:  none
**************************************************************************/
void uart_tx_powerTripADC(){
	#define BYTES 6
	uint8_t txBuffer[BYTES+5];
	uint8_t i;
	unsigned char *p;
	txBuffer[0] = SOF;
	txBuffer[1] = SYNC;
	txBuffer[2] = MC_ID_POWER_TRIP_ADC;
	txBuffer[3] = BYTES; // nr of bytes 3 * 2;
	p = (unsigned char *)&trip_values.Pfwrd_trip_ADC;
	for(i=0; i<2; i++){ txBuffer[4+i] = *p++ ;}
	p = (unsigned char *)&trip_values.Prefl_trip_ADC;
	for(i=0; i<2; i++){ txBuffer[6+i] = *p++ ;}
	p = (unsigned char *)&trip_values.Pin_trip_ADC;
	for(i=0; i<2; i++){ txBuffer[8+i] = *p++ ;}
	txBuffer[BYTES+4] = EOFSYNC;
	for (i=0;i<=BYTES+4;i++)
	{
		uart_putc(txBuffer[i]);
	}
}

/*************************************************************************
Function: uart_tx_powerTripVals()
Purpose:  Transmit the current power trip values via UART as 3 times 2 bytes (uint16_t)
Input:    none
Returns:  none
**************************************************************************/
void uart_tx_powerTripVals(){
	#define BYTES 6
	uint8_t txBuffer[BYTES+5];
	uint8_t i;
	unsigned char *p;
	txBuffer[0] = SOF;
	txBuffer[1] = SYNC;
	txBuffer[2] = MC_ID_POWER_TRIP_VALS;
	txBuffer[3] = BYTES; // nr of bytes 3 * 2;
	p = (unsigned char *)&trip_values.Pfwrd_trip_W;
	for(i=0; i<2; i++){ txBuffer[4+i] = *p++ ;}
	p = (unsigned char *)&trip_values.Prefl_trip_W;
	for(i=0; i<2; i++){ txBuffer[6+i] = *p++ ;}
	p = (unsigned char *)&trip_values.Pin_trip_W;
	for(i=0; i<2; i++){ txBuffer[8+i] = *p++ ;}
	txBuffer[BYTES+4] = EOFSYNC;
	for (i=0;i<=BYTES+4;i++)
	{
		uart_putc(txBuffer[i]);
	}
}

/*************************************************************************
Function: uart_tx_powerCalibrationADC2W()
Purpose:  Transmit the power values incl. swr via UART as 4 times 4 bytes (float)
Input:    none
Returns:  none
**************************************************************************/
void uart_tx_powerVals(){
	#define BYTES 16 // nr of bytes 4 * 4;
	uint8_t txBuffer[BYTES+5];
	uint8_t i;
	unsigned char *p;
	txBuffer[0] = SOF;
	txBuffer[1] = SYNC;
	txBuffer[2] = MC_ID_POWER_VALS;
	txBuffer[3] = BYTES;
	p = (unsigned char *)&power.fwrd;
	for(i=0; i<4; i++){ txBuffer[4+i] = *p++ ;}
	p = (unsigned char *)&power.refl;
	for(i=0; i<4; i++){ txBuffer[8+i] = *p++ ;}
	p = (unsigned char *)&power.input;
	for(i=0; i<4; i++){ txBuffer[12+i] = *p++ ;}
	p = (unsigned char *)&power.swr;
	for(i=0; i<4; i++){ txBuffer[16+i] = *p++ ;}
	txBuffer[BYTES+4] = EOFSYNC;
	for (i=0;i<=BYTES+4;i++)
	{
		uart_putc(txBuffer[i]);
	}
}


/*************************************************************************
Function: uart_tx_powerCalibrationADC2W()
Purpose:  Transmit the ADC to Powercalibration factors via UART as 3 times 4 bytes (float)
Input:    none
Returns:  none
**************************************************************************/
void uart_tx_powerCalibrationADC2W(){
	#define BYTES 12 // nr of bytes 3 * 4;
	uint8_t txBuffer[BYTES+5];
	uint8_t i;
	unsigned char *p;
	txBuffer[0] = SOF;
	txBuffer[1] = SYNC;
	txBuffer[2] = MC_ID_POWER_CAL_ADC2W;
	txBuffer[3] = BYTES;
	p = (unsigned char *)&cal_values.Pfwrd_ADC2W;
	for(i=0; i<4; i++){ txBuffer[4+i] = *p++ ;}
	p = (unsigned char *)&cal_values.Prefl_ADC2W;
	for(i=0; i<4; i++){ txBuffer[8+i] = *p++ ;}
	p = (unsigned char *)&cal_values.Pin_ADC2W;
	for(i=0; i<4; i++){ txBuffer[12+i] = *p++ ;}
	txBuffer[BYTES+4] = EOFSYNC;
	for (i=0;i<=BYTES+4;i++)
	{
		uart_putc(txBuffer[i]);
	}
}

/*************************************************************************
Function: uart_tx_powerCalibrationW2ADC()
Purpose:  Transmit the Power to ADC calibration factors via UART as 3 times 4 bytes (float)
Input:    none
Returns:  none
**************************************************************************/
void uart_tx_powerCalibrationW2ADC(){
	#define BYTES 12 // nr of bytes 3 * 4;
	uint8_t txBuffer[BYTES+5];
	uint8_t i;
	unsigned char *p;
	txBuffer[0] = SOF;
	txBuffer[1] = SYNC;
	txBuffer[2] = MC_ID_POWER_CAL_W2ADC;
	txBuffer[3] = BYTES;
	p = (unsigned char *)&cal_values.Pfwrd_W2ADC;
	for(i=0; i<4; i++){ txBuffer[4+i] = *p++ ;}
	p = (unsigned char *)&cal_values.Prefl_W2ADC;
	for(i=0; i<4; i++){ txBuffer[8+i] = *p++ ;}
	p = (unsigned char *)&cal_values.Pin_W2ADC;
	for(i=0; i<4; i++){ txBuffer[12+i] = *p++ ;}
	txBuffer[BYTES+4] = EOFSYNC;
	for (i=0;i<=BYTES+4;i++)
	{
		uart_putc(txBuffer[i]);
	}
}
