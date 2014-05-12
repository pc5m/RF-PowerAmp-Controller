/*
 * CommProtocol.h
 *
 * Created: 8-5-2014 1:18:38
 *  Author: pc5m
 */ 


#ifndef COMMPROTOCOL_H_
#define COMMPROTOCOL_H_

#include <inttypes.h>

/* PROTOCOL STATES */
#define UNKNOWN					0x00
#define SOF_RECEIVED			0x01
#define SYNC_RECEIVED			0x02
#define ID_RECEIVED				0x03
#define DLC_RECEIVED			0x04
#define EOF						0x05
#define EOFSYNC_RECEIVED		0x06

/* PROTOCOL BYTES */
#define SOF						0xAA
#define SYNC					0x55
#define EOFSYNC                 0x66


/* IDs sent from PC to MC */
#define PC_ID_REQ_FW_VERSION					0
#define PC_ID_SET_STATUS_AUTOTX_STATUS			1
#define PC_ID_SET_STATUS_AUTOTX_CURRENTS_ADC	2
#define PC_ID_SET_STATUS_AUTOTX_CURRENTS_VALS	3
#define PC_ID_REQ_CURRENT_TRIP_ADC				4
#define PC_ID_REQ_CURRENT_TRIP_VAL				5
#define PC_ID_SET_CURRENT_TRIP_ADC				6
#define PC_ID_SET_CURRENT_TRIP_VAL				7
#define PC_ID_SET_CAL_CURRENTS_AMP2ADC			8
#define PC_ID_SET_CAL_CURRENTS_ADC2AMP			9
#define PC_ID_REQ_CAL_CURRENTS_AMP2ADC			10
#define PC_ID_REQ_CAL_CURRENTS_ADC2AMP			11
#define PC_ID_SET_STATUS_AUTOTX_POWERS_ADC		12
#define PC_ID_SET_STATUS_AUTOTX_POWERS_VALS		13
#define PC_ID_REQ_POWERS_TRIP_ADC				14
#define PC_ID_REQ_POWERS_TRIP_VALS				15
#define PC_ID_SET_POWERS_TRIP_ADC				16
#define PC_ID_SET_POWERS_TRIP_VAL				17
#define PC_ID_SET_CAL_POWERS_W2ADC				18
#define PC_ID_SET_CAL_POWERS_ADC2W				19
#define PC_ID_REQ_CAL_POWERS_W2ADC				20
#define PC_ID_REQ_CAL_POWERS_ADC2W				21
#define PC_ID_REQ_TEMPS							22
#define PC_ID_REQ_TEMP_TRIP						23
#define PC_ID_SET_TEMP_TRIP						24

/* IDs sent from MC to PC */
#define MC_ID_STATUS				0
#define MC_ID_CURRENT_ADC_VALS		1
#define MC_ID_CURRENT_VALS			2
#define MC_ID_CURRENT_TRIP_ADC		3
#define MC_ID_CURRENT_TRIP_VAL		4
#define MC_ID_CURRENT_CAL_ADC2AMP	5
#define MC_ID_CURRENT_CAL_AMP2ADC	6
#define MC_ID_POWER_ADC_VALS		7
#define MC_ID_POWER_TRIP_ADC		8
#define MC_ID_POWER_TRIP_VALS		9
#define MC_ID_POWER_CAL_ADC2W		10
#define MC_ID_POWER_CAL_W2ADC		11
#define MC_ID_POWER_VALS			12
#define MC_ID_TEMPERATURE			13
#define MC_ID_TEMPERATURE_TRIP		14
#define MC_ID_FW_VERSION			15


typedef volatile struct{
	uint8_t id;
	uint8_t dlc;
	uint8_t data[16];  // maximum length of received messages excluding EOFSYNC
}Com_Message_struct;


extern void comm_RX_process();


/**
 *  @brief   Transmit the data from controller to host via UART
 *  @param   none
 *  @return  none
 */
extern void comm_tx_version();  // transmit version number of firmware
extern void uart_tx_temperatures();
extern void uart_tx_temperatureTrip();

extern void uart_tx_currentVals();
extern void uart_tx_currentADCVals();
extern void uart_tx_currentTripADC();
extern void uart_tx_currentTripVal();
extern void uart_tx_currentCalibrationADC2Amp();
extern void uart_tx_currentCalibrationAmp2ADC();

extern void uart_tx_powerVals();
extern void uart_tx_powerADCVals();
extern void uart_tx_powerTripADC();
extern void uart_tx_powerTripVals();
extern void uart_tx_powerCalibrationADC2W();
extern void uart_tx_powerCalibrationW2ADC();

extern void uart_tx_status();  // status of TX/RX, PSU On/Off, Error/Trip



#endif /* COMMPROTOCOL_H_ */