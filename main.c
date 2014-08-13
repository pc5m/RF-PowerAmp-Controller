/*
 * control_atmega644.c
 *
 * Created: 16-4-2014 13:59:44
 *  Author: PC5M, C. Mobach
 */ 

#include "generalDefine.h"
#include <avr/io.h>
#include <stdbool.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <math.h>
#include <util/delay.h>
#include "lcd.h"
#include "adc.h"
#include "display.h"
#include "lm75.h"
#include "uart.h"
#include "comm.h"

#define REL_TX      PB0
#define BIAS_ENABLE PB1
#define LED_TX      PB2
#define LED_ERROR   PB3

#define PTT_IN      PD2
#define OVERLOAD_IN   PD3
#define PSU_ENABLE  PD6
#define BUTTON_IN   PD5 // TP21

#define UART_BAUD_RATE      115200
uint8_t uartTxBuffer[10];

/*
 * main_get_ptt() - get PTT input state, return ON (1) if input is low, return OFF (0) if input is high
 */
unsigned char main_get_ptt(void)
{
	return bit_get(PIND, BIT(PTT_IN)) ? OFF : ON;
}

/*
* main_get_button() - get button state, ON = pressed, OFF = released
*/
unsigned char main_get_button(void)
{
	return bit_get(PIND, BIT(BUTTON_IN)) ? OFF : ON;
}

/*
* main_get_overload() - get overload state, ON = overload, OFF = no overload
*/
unsigned char main_get_overload(void)
{
	return bit_get(PIND, BIT(OVERLOAD_IN)) ? OFF : ON;
}

/*
* set_LEDerror () - set error led
*/
void main_set_LEDerror(unsigned char on)
{
	if (on)
		bit_set(PORTB, BIT(LED_ERROR));
	else
		bit_clear(PORTB, BIT(LED_ERROR));
}

/*
* main_set_LEDtx () - set tx led
*/
void main_set_LEDtx (unsigned char on)
{
	if (on)
		bit_set(PORTB, BIT(LED_TX));
	else
		bit_clear(PORTB, BIT(LED_TX));
}



/*
* main_set_bias () - set PA BIAS 
*/
void main_set_bias(unsigned char on) {
	if (on) {
		bit_set(PORTB, BIT(BIAS_ENABLE));
		bit_set(SSPAstatus,BIT(FLAG_BIAS_ON));
	}
	else {
		bit_clear(PORTB, BIT(BIAS_ENABLE));
		bit_clear(SSPAstatus,BIT(FLAG_BIAS_ON));
	}
}

/*
* mian_set_RELtx (on) - set relais in TX 
*/
void mian_set_RELtx(unsigned char on) {
	if (on)
		bit_set(PORTB, BIT(REL_TX));
	else
		bit_clear(PORTB, BIT(REL_TX));
}

/*
* main_set_TX (on) - set bias, led and relais in TX or RX position
*/
void main_set_TX (unsigned char on) {
	main_set_bias(on);
	main_set_LEDtx(on);
	mian_set_RELtx(on);
	if (on) 
		bit_set(SSPAstatus,BIT(FLAG_TX_ON));
	else
		bit_clear(SSPAstatus,BIT(FLAG_TX_ON));
}

/*
* main_set_PSU (on) - enable powersupply
*/
void main_set_PSU(unsigned char on) {
	if (on)
	{
		bit_clear(PORTD, BIT(PSU_ENABLE));
		bit_set(SSPAstatus,BIT(FLAG_PSU_ON));
	}
	else
	{
		bit_set(PORTD, BIT(PSU_ENABLE));
		bit_clear(SSPAstatus,BIT(FLAG_PSU_ON));
	}
}

void main_set_ERROR(unsigned char on)
{
	if (on)
	{
		bit_set(SSPAstatus,BIT(FLAG_ERROR));
	    main_set_LEDerror(ON);
	}
	else
	{
		bit_clear(SSPAstatus,BIT(FLAG_ERROR));
		main_set_LEDerror(OFF);
	}
}

void main_set_trip_temperature (uint8_t val)
{
	trip_values.temp_trip = val;
	eeprom_update_byte(&EEtrip_values.temp_trip, val);
}

void main_set_trip_current_val (uint8_t val) {
	trip_values.Imod_trip_AMP = val;
    eeprom_update_word(&EEtrip_values.Imod_trip_AMP, val );
}

void main_set_trip_current_adc (uint8_t moduleID, uint16_t val) {
	switch (moduleID)
	{
		case MODULE_A: trip_values.ImodA_trip_ADC = val; eeprom_update_word(&EEtrip_values.ImodA_trip_ADC, val ); break;
		case MODULE_B: trip_values.ImodB_trip_ADC = val; eeprom_update_word(&EEtrip_values.ImodB_trip_ADC, val ); break;
		case MODULE_C: trip_values.ImodC_trip_ADC = val; eeprom_update_word(&EEtrip_values.ImodC_trip_ADC, val ); break;
		case MODULE_D: trip_values.ImodD_trip_ADC = val; eeprom_update_word(&EEtrip_values.ImodD_trip_ADC, val ); break;
		default:
		break;
	}
}

void main_set_cal_currents_amp2adc (uint8_t moduleID, float val) {
	switch (moduleID)
	{
		case MODULE_A: cal_values.ImodA_AMP2ADC = val; eeprom_update_float(&EEcal_values.ImodA_AMP2ADC, val ); break;
		case MODULE_B: cal_values.ImodB_AMP2ADC = val; eeprom_update_float(&EEcal_values.ImodB_AMP2ADC, val ); break;
		case MODULE_C: cal_values.ImodC_AMP2ADC = val; eeprom_update_float(&EEcal_values.ImodC_AMP2ADC, val ); break;
		case MODULE_D: cal_values.ImodD_AMP2ADC = val; eeprom_update_float(&EEcal_values.ImodD_AMP2ADC, val ); break;
		default: break;
	}
}

void main_set_cal_currents_adc2amp (uint8_t moduleID, float val) {
	switch (moduleID)
	{
		case MODULE_A: cal_values.ImodA_ADC2AMP = val; eeprom_update_float(&EEcal_values.ImodA_ADC2AMP, val ); break;
		case MODULE_B: cal_values.ImodB_ADC2AMP = val; eeprom_update_float(&EEcal_values.ImodB_ADC2AMP, val ); break;
		case MODULE_C: cal_values.ImodC_ADC2AMP = val; eeprom_update_float(&EEcal_values.ImodC_ADC2AMP, val ); break;
		case MODULE_D: cal_values.ImodD_ADC2AMP = val; eeprom_update_float(&EEcal_values.ImodD_ADC2AMP, val ); break;
		default: break;
	}
}


// set power trip values in memory and in EEPROM
// Input: powerID: ID of the trip power to be set, val: trip value to be set	
void main_set_trip_powers_val(uint8_t powerID, uint16_t val) {
	switch (powerID)
	{
		case POWER_FWD:  trip_values.Pfwrd_trip_W = val; eeprom_update_word(&EEtrip_values.Pfwrd_trip_W, val); break;
		case POWER_REFL: trip_values.Prefl_trip_W = val; eeprom_update_word(&EEtrip_values.Prefl_trip_W, val); break; 
		case POWER_IN:   trip_values.Pin_trip_W = val;   eeprom_update_word(&EEtrip_values.Pin_trip_W, val);   break;
		default:break;
	}
} // main_set_trip_powers_val(uint8_t powerID, uint16_t val)


// set swr trip values in memory and in EEPROM
// Input: val: trip value to be set
void main_set_trip_swr_val(float val) {
	trip_values.swr_trip = val;
	eeprom_update_float(&EEtrip_values.swr_trip, val);
} // main_set_trip_swr_val(float val)


// set power trip ADC values in memory and in EEPROM
// Input: powerID: ID of the trip power to be set, val: trip ADC value to be set
void main_set_trip_powers_adc(uint8_t powerID, uint16_t val) {
	switch (powerID)
	{
		case POWER_FWD:  trip_values.Pfwrd_trip_ADC = val; eeprom_update_word(&EEtrip_values.Pfwrd_trip_ADC, val); break;
		case POWER_REFL: trip_values.Prefl_trip_ADC = val; eeprom_update_word(&EEtrip_values.Prefl_trip_ADC, val); break;
		case POWER_IN:   trip_values.Pin_trip_ADC = val;   eeprom_update_word(&EEtrip_values.Pin_trip_ADC, val);   break;
		// case POWER_SWR:  trip_values.SWR_trip_ADC = val;   eeprom_update_word(&EEtrip_values.SWR_trip_ADC, val);   break;
		default: break;
	}
} // main_set_trip_powers_adc(uint8_t powerID, uint16_t val)


// Get the uint16_t corresponds with linear interpolation using y = rc * x + b. Used RC and B determined based on array position.
// nrOfVals = number of datapoints in xArray.
uint16_t main_Interpolate (uint16_t x, uint16_t *xArray, float *RCArray, float *BArray, uint8_t nrOfVals){
	uint8_t i;
	for (i = nrOfVals-2; i > 0 ; i--)
	{
		if (x >= xArray[i] ) break;
	}
	return(RCArray[i]*x+BArray[i]);
}

// set_cal_powers_adc2w (uint8_t powerID, message) - set calibration values for power, ADC to Watt conversion in memory and in EEPROM
// Input: powerID: ID of the calibration power to be set, message: Calibration points and 
void main_set_powerCalibrationADC2W_RC_B(uint8_t powerID, uint8_t NrOfCalibrationPoints, uint16_t* xArray, float* RCArray, float* BArray){
	uint8_t i;
	switch (powerID)
	{
	case POWER_FWD:  
		calPower_values.Pfwrd_nr = NrOfCalibrationPoints; 
		eeprom_update_byte(&EEcalPower_values.Pfwrd_nr, NrOfCalibrationPoints); 
		for (i = 0; i < 5 ; i++)
		{
			calPower_values.Pfwrd_ADC[i] = xArray[i];
			eeprom_update_word(&EEcalPower_values.Pfwrd_ADC[i], xArray[i]); 
		}
		for (i = 0; i < 4 ; i++)
		{
			calPower_values.Pfwrd_ADC2W_B[i] = BArray[i];
			eeprom_update_float(&EEcalPower_values.Pfwrd_ADC2W_B[i], BArray[i]); 
			calPower_values.Pfwrd_ADC2W_RC[i] = RCArray[i];
			eeprom_update_float(&EEcalPower_values.Pfwrd_ADC2W_RC[i], RCArray[i]); 
		}
		break;
	case POWER_REFL:
		calPower_values.Prefl_nr = NrOfCalibrationPoints;
		eeprom_update_byte(&EEcalPower_values.Prefl_nr, NrOfCalibrationPoints); 
		for (i = 0; i < 5 ; i++)
		{
			calPower_values.Prefl_ADC[i] = xArray[i];
			eeprom_update_word(&EEcalPower_values.Prefl_ADC[i], xArray[i]); 
		}
		for (i = 0; i < 4 ; i++)
		{
			calPower_values.Prefl_ADC2W_B[i] = BArray[i];
			calPower_values.Prefl_ADC2W_RC[i] = RCArray[i];
			eeprom_update_float(&EEcalPower_values.Prefl_ADC2W_B[i], BArray[i]); 
			eeprom_update_float(&EEcalPower_values.Prefl_ADC2W_RC[i], RCArray[i]);
		}
		break;
	case POWER_IN:
		calPower_values.Pin_nr = NrOfCalibrationPoints;
		eeprom_update_byte(&EEcalPower_values.Pin_nr, NrOfCalibrationPoints); 
		for (i = 0; i < 5 ; i++)
		{
			calPower_values.Pin_ADC[i] = xArray[i];
			eeprom_update_word(&EEcalPower_values.Pin_ADC[i], xArray[i]); 
		}
		for (i = 0; i < 4 ; i++)
		{
			calPower_values.Pin_ADC2W_B[i] = BArray[i];
			calPower_values.Pin_ADC2W_RC[i] = RCArray[i];			
			eeprom_update_float(&EEcalPower_values.Pin_ADC2W_B[i], BArray[i]);
			eeprom_update_float(&EEcalPower_values.Pin_ADC2W_RC[i], RCArray[i]);
		}
		break;
	default : break;
	}
}

void main_SSPA_IO_init(void) {
	// define BIAS_ENABLE, LED_TX, LED_ERROR,PSU_ENABLE as output and set all to defined state 
	main_set_LEDerror(OFF);  // led error is off
	main_set_TX(OFF); // PA bias is disabled, led tx is off (RX state), Coax relais TX off (rx state)
	main_set_PSU(OFF);        // Disable powersupply
	bit_set(DDRB, ( BIT(REL_TX) | BIT (BIAS_ENABLE) | BIT(LED_TX) | BIT (LED_ERROR) ) );
	bit_set(DDRD, BIT (PSU_ENABLE) ) ;
	// define PTT_IN and OVERLOAD_IN, BUTTON_IN as input enable pull up resistors
	bit_clear(DDRD, (BIT(PTT_IN) | BIT (OVERLOAD_IN) | BIT(BUTTON_IN) ) ); 	// define PTT_IN and OVERLOAD_IN as input 
	bit_set(PORTD,BIT(PTT_IN) | BIT(BUTTON_IN)  );                          // enable pull up resistor on PTT_IN and BUTTON_IN
	bit_clear(PORTD,BIT (OVERLOAD_IN) );                                      // disable pull up resistor on OVERLOAD_IN
}

// Calculate actual currents, taken into account calibration values
// ADC and full calibration data should be available when entering this routine
void main_CalculateCurrents(void)
{
	current.moduleA = cal_values.ImodA_ADC2AMP * adc_values.iModuleA_ADC;
	current.moduleB = cal_values.ImodB_ADC2AMP * adc_values.iModuleB_ADC;
	current.moduleC = cal_values.ImodC_ADC2AMP * adc_values.iModuleC_ADC;
	current.moduleD = cal_values.ImodD_ADC2AMP * adc_values.iModuleD_ADC;
	current.moduleTotal = current.moduleA + current.moduleB + current.moduleC + current.moduleD;
}

// Calculate actual power and SWR, taken into account calibration values
// ADC and full calibration data should be available when entering this routine
void main_CalculatePowerAndSWR(void)
{

    power.fwrd = main_Interpolate(adc_values.pwrFwrd_ADC, calPower_values.Pfwrd_ADC, calPower_values.Pfwrd_ADC2W_RC, calPower_values.Pfwrd_ADC2W_B,calPower_values.Pfwrd_nr);
	power.refl = main_Interpolate(adc_values.pwrRefl_ADC, calPower_values.Prefl_ADC, calPower_values.Prefl_ADC2W_RC, calPower_values.Prefl_ADC2W_B,calPower_values.Prefl_nr);
	power.input = main_Interpolate(adc_values.pwrIn_ADC, calPower_values.Pin_ADC, calPower_values.Pin_ADC2W_RC, calPower_values.Pin_ADC2W_B,calPower_values.Pin_nr);
	
    if (power.fwrd > 100){
		float w;
		w = sqrt(power.refl/power.fwrd);
		if (w > 0.9) w = 0.9; // limit swr to 1 : 19
		power.swr = (1+w)/(1-w);
	} else power.swr = 999;   // used later to not display SWR when Forward power is too low.
}




	
enum ErrorStates CheckForError(void)
{ 
	if (adc_values.iModuleA_ADC > trip_values.ImodA_trip_ADC) return (ImodA);
	if (adc_values.iModuleB_ADC > trip_values.ImodB_trip_ADC) return (ImodB);
	if (adc_values.iModuleC_ADC > trip_values.ImodC_trip_ADC) return (ImodC);
	if (adc_values.iModuleD_ADC > trip_values.ImodD_trip_ADC) return (ImodD);
	
	if (adc_values.pwrFwrd_ADC > trip_values.Pfwrd_trip_ADC) return (Pfwrd);
	if (adc_values.pwrRefl_ADC > trip_values.Prefl_trip_ADC) return (Prefl);
	if (adc_values.pwrIn_ADC   > trip_values.Pin_trip_ADC)   return (Pin);
	
	if ((power.swr > trip_values.swr_trip) && (power.swr != 999)) return (SWR);
	
	// Check for trip main_temperature, in case sensor not attached: temp returned = 255;
    // TODO: Temperature below 0 degree not correct !
	if ((temp_values.tempA > trip_values.temp_trip) && (temp_values.tempA != 255)) return (TempA);
	if ((temp_values.tempB > trip_values.temp_trip) && (temp_values.tempB != 255)) return (TempB);
	if ((temp_values.tempC > trip_values.temp_trip) && (temp_values.tempC != 255)) return (TempC);
	if ((temp_values.tempD > trip_values.temp_trip) && (temp_values.tempD != 255)) return (TempD);
	
	return (NoError);
}

enum ErrorStates CheckForOverload(void) {
	enum ErrorStates error;
	error = CheckForError();
	SSPAstatus |= (uint8_t)error;
	return(error);
}

void main_temperature()
{
	temp_values.tempA = lm75_Temp(LM75_A);
	temp_values.tempB = lm75_Temp(LM75_B);
	temp_values.tempC = lm75_Temp(LM75_C);
	temp_values.tempD = lm75_Temp(LM75_D);
	
	// get maximum temp of connected sensors, in case sensor not attached (255 value) is not taken into account
    temp_values.tempMax = 255;
	if (temp_values.tempA  != 255) temp_values.tempMax = temp_values.tempA;
	if ((temp_values.tempB != 255) && (temp_values.tempB > temp_values.tempMax )) temp_values.tempMax = temp_values.tempB;
	if ((temp_values.tempC != 255) && (temp_values.tempC > temp_values.tempMax )) temp_values.tempMax = temp_values.tempC;
	if ((temp_values.tempD != 255) && (temp_values.tempD > temp_values.tempMax )) temp_values.tempMax = temp_values.tempD;
}

/**
 @brief    Determines if button was pressed and released
 @param    none                
 @return   TRUE if pressed and released
*/
unsigned char main_button_pressed(void) {
	static int pressedCounter, releasedCounter;
	static bool pressed; 
	if (main_get_button() == TRUE) pressedCounter++; else releasedCounter++;
	if (pressedCounter > 10) {
       releasedCounter = 0;
	   pressedCounter = 10;
	   pressed = TRUE;
	}
	if (releasedCounter > 10) {
		releasedCounter = 10;
		pressedCounter = 0;
		if (pressed == TRUE) {
			pressed = FALSE;
			return (TRUE);
		}

	}
	return (FALSE);
}//button_pressed()

enum activeMenus nextActiveMenu(){
	switch (activeMenu)
	{
		case Imod_menu: return (Pall_menu); break;
		case Pall_menu: return (Gen_Menu); break;
		case Gen_Menu: return (Temp_menu); break;
		case Temp_menu: return (Imod_menu); break;
	}
	return(Pall_menu);  //should never be reached, to keep compiler happy
}//nextActiveMenu()



void main_LoadEEpromVals() 
{
	eeprom_read_block(&trip_values, &EEtrip_values, sizeof(EEtrip_values));
	eeprom_read_block(&cal_values, &EEcal_values, sizeof(EEcal_values));
	eeprom_read_block(&calPower_values, &EEcalPower_values, sizeof(EEcalPower_values));
}

void main_ProcessSerialCommunication() 
{
	if (controlConnected == TRUE)
	{
		comm_uart_tx_status();
		if (autoTransmitCurrentVals == TRUE) {
			comm_uart_tx_currentVals();
		}
		if (autoTransmitCurrentADC == TRUE) {
			comm_uart_tx_currentADCVals();
		}
		if (autoTransmitPowerVals == TRUE) {
			comm_uart_tx_powerVals();
		}
		if (autoTransmitPowerADC == TRUE) {
			comm_uart_tx_powerADCVals();
		}
		if (autoTransmitTemperature == TRUE) {
			comm_uart_tx_temperatures();				
		}
	}
	comm_RX_process();   // check if a command is received and act upon
}






int main(void)
{
	uint16_t temperatureCounterDisplay = 0;
	uint8_t errorNotCancelled = TRUE;
	uint16_t errorDisplayRateIndex = 0;
	MCUCR=(1<<JTD); // disable JTAG on portC
	MCUCR=(1<<JTD); // disable JTAG on portC
    main_SSPA_IO_init();      // initialize unit: BIAS = OFF, PSU = ON, RX STAT;
	main_LoadEEpromVals();
	lcd_init(LCD_DISP_ON);
	display_InitBargraph();
	adc_init();
	lm75_init();
	uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) ); 
	sei(); // needed for uart libray
	activeMenu = Imod_menu;  //default menu is current display of all  4 modules
	nextMenu = Gen_Menu;
	main_temperature(); // get temperature
	comm_tx_version();
	if (main_get_button() == TRUE) decay = 0; //no peak hold when menu button is pressed at startup
	display_welcomeMessage();
	_delay_ms(3000);
	lcd_clrscr();
	main_set_PSU(ON);
	while(TRUE)
    {
		   //-	Get ADC input vals for power measurements
		   //-	Get ADC input vals for current measurements
		   //-	Once in 10? Cycles get one temperature measurement
		   //-	Check for overload, if overload goto error procedure
		   //-	Check PA enable button, if checked enable PSU’s
		   //-	Send data to display, incl. bargraph
		   //-	Check Menu button, if checked goto next menu
		   //-	Check PTT, if checked  and PA enable : enable PA bias
			adc_GetData();
			main_CalculateCurrents();
			main_CalculatePowerAndSWR();
			temperatureCounterDisplay ++;
			if (temperatureCounterDisplay > 800) {  //400 gives every 2 seconds an update
				main_temperature();
				temperatureCounterDisplay = 0;
			}
			activeError = CheckForOverload();
			if (activeError != NoError)   // ERROR !
			{
				main_set_PSU(OFF);
                main_set_TX(OFF);
			    main_set_ERROR(ON);
				display_error();
				errorNotCancelled = TRUE;
				while(errorNotCancelled)    //continue to display error on error menu, until menu button pressed  
				{
					adc_GetData();
					main_CalculateCurrents();
					main_CalculatePowerAndSWR();
					errorDisplayRateIndex++; //slow down error refresh rate, to get better display (due to clear screen) 
					if (errorDisplayRateIndex >= 100) {
						display_error();   
						errorDisplayRateIndex = 0;
					}
					main_ProcessSerialCommunication(); //if connected than transmit serial data
					if (main_button_pressed() == TRUE) 
					{
						errorNotCancelled = FALSE;
						main_set_PSU(ON);
						main_set_ERROR(OFF);
						lcd_clrscr();
					}
				}
			}
			display_Menu(nextMenu);
			activeMenu = nextMenu;
			display_Bargraph(activeMenu);
			
			if (main_get_ptt() == ON)
				main_set_TX(ON);
			else
				main_set_TX(OFF);
				 	
			if (main_button_pressed() == TRUE) nextMenu = nextActiveMenu();

			main_ProcessSerialCommunication();  //if connected than transmit serial data
    }
}