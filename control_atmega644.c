/*
 * control_atmega644.c
 *
 * Created: 16-4-2014 13:59:44
 *  Author: cmobach
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
#include "CommProtocol.h"

#define REL_TX      PB0
#define BIAS_ENABLE PB1
#define LED_TX      PB2
#define LED_ERROR   PB3

#define PTT_IN      PD2
#define OVERLOAD_IN   PD3
#define PSU_ENABLE  PD6
#define BUTTON_IN   PD5 // TP21


/* 9600 baud */
#define UART_BAUD_RATE      115200


uint8_t uartTxBuffer[10];

/*
 * get_ptt() - get PTT input state, return ON (1) if input is low, return OFF (0) if input is high
 */
unsigned char get_ptt(void)
{
	return bit_get(PIND, BIT(PTT_IN)) ? OFF : ON;
}

/*
* get_button() - get button state, ON = pressed, OFF = released
*/
unsigned char get_button(void)
{
	return bit_get(PIND, BIT(BUTTON_IN)) ? OFF : ON;
}

/*
* get_overload() - get overload state, ON = overload, OFF = no overload
*/
unsigned char get_overload(void)
{
	return bit_get(PIND, BIT(OVERLOAD_IN)) ? OFF : ON;
}

/*
* set_LEDerror () - set error led
*/
void set_LEDerror(unsigned char on)
{
	if (on)
		bit_set(PORTB, BIT(LED_ERROR));
	else
		bit_clear(PORTB, BIT(LED_ERROR));
}

/*
* set_LEDtx () - set tx led
*/
void set_LEDtx (unsigned char on)
{
	if (on)
		bit_set(PORTB, BIT(LED_TX));
	else
		bit_clear(PORTB, BIT(LED_TX));
}



/*
* set_BIAS () - set PA BIAS 
*/
void set_BIAS(unsigned char on) {
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
* set_RELtx (on) - set relais in TX 
*/
void set_RELtx(unsigned char on) {
	if (on)
		bit_set(PORTB, BIT(REL_TX));
	else
		bit_clear(PORTB, BIT(REL_TX));
}

/*
* set_TX (on) - set bias, led and relais in TX or RX position
*/
void set_TX (unsigned char on) {
	set_BIAS(on);
	set_LEDtx(on);
	set_RELtx(on);
	if (on) 
		bit_set(SSPAstatus,BIT(FLAG_TX_ON));
	else
		bit_clear(SSPAstatus,BIT(FLAG_TX_ON));
}

/*
* set_PSU (on) - enable powersupply
*/
void set_PSU(unsigned char on) {
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

void set_ERROR(unsigned char on)
{
	set_LEDerror(ON);
	if (on)
	{
		bit_set(SSPAstatus,BIT(FLAG_ERROR));
	}
	else
	{
		bit_clear(SSPAstatus,BIT(FLAG_ERROR));
	}
}

void SSPA_IO_init(void) {
	// define BIAS_ENABLE, LED_TX, LED_ERROR,PSU_ENABLE as output and set all to defined state 
	set_LEDerror(OFF);  // led error is off
	set_TX(OFF); // PA bias is disabled, led tx is off (RX state), Coax relais TX off (rx state)
	set_PSU(ON);        // Enable powersupply
	bit_set(DDRB, ( BIT(REL_TX) | BIT (BIAS_ENABLE) | BIT(LED_TX) | BIT (LED_ERROR) ) );
	bit_set(DDRD, BIT (PSU_ENABLE) ) ;
	// define PTT_IN and OVERLOAD_IN, BUTTON_IN as input enable pull up resistors
	bit_clear(DDRD, (BIT(PTT_IN) | BIT (OVERLOAD_IN) | BIT(BUTTON_IN) ) ); 	// define PTT_IN and OVERLOAD_IN as input 
	bit_set(PORTD,BIT(PTT_IN) | BIT(BUTTON_IN)  );                          // enable pull up resistor on PTT_IN and BUTTON_IN
	bit_clear(PORTD,BIT (OVERLOAD_IN) );                                      // disable pull up resistor on OVERLOAD_IN
}

void getCalibrationData(void)
{
	// todo: get values from EEPROM , for testing the values set in general defines are used
}

// Calculate actual currents, taken into account calibration values
// ADC and full calibration data should be available when entering this routine
void CalculateCurrents(void)
{
	current.moduleA = cal_values.ImodA_ADC2AMP * adc_values.iModuleA_ADC;
	current.moduleB = cal_values.ImodB_ADC2AMP * adc_values.iModuleB_ADC;
	current.moduleC = cal_values.ImodC_ADC2AMP * adc_values.iModuleC_ADC;
	current.moduleD = cal_values.ImodD_ADC2AMP * adc_values.iModuleD_ADC;
	current.moduleTotal = current.moduleA + current.moduleB + current.moduleC + current.moduleD;
}

// Calculate actual power and SWR, taken into account calibration values
// ADC and full calibration data should be available when entering this routine
void CalculatePowerAndSWR(void)
{
	power.fwrd = cal_values.Pfwrd_ADC2W * adc_values.pwrFwrd_ADC;
	power.refl = cal_values.Prefl_ADC2W * adc_values.pwrRefl_ADC;
	power.input = cal_values.Pin_ADC2W * adc_values.pwrIn_ADC;
	power.swr = 1.6; // todo: calculate swr with sqrt
}


	
enum ErrorStates CheckForError(void)
{ 
	
	// if (adc_values.iModuleA_ADC > trip_values.ImodA_trip_ADC) return (ImodA);
	// if (adc_values.iModuleB_ADC > trip_values.ImodB_trip_ADC) return (ImodB);
	// if (adc_values.iModuleC_ADC > trip_values.ImodC_trip_ADC) return (ImodC);
	// if (adc_values.iModuleD_ADC > trip_values.ImodD_trip_ADC) return (ImodD);
	
	if (adc_values.pwrFwrd_ADC > trip_values.Pfwrd_trip_ADC) return (Pfwrd);
	if (adc_values.pwrRefl_ADC > trip_values.Prefl_trip_ADC) return (Prefl);
	if (adc_values.pwrIn_ADC   > trip_values.Pin_trip_ADC)   return (Pin);
	// todo trip value for SWR ?
	
	// Check for trip temperature, in case sensor not attached: temp returned = 255;
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
 
//ISR (TIMER0_OVF_vect)
//{
	//bit_flip(PORTD,BIT(PD5));
	//TCNT0 = 76;
	//lcd_puts("PC5M");
//}
//
//void initTimer()
//{
	//bit_set(TCCR0B, BIT(CS02) | BIT(CS00)) ;  /* set clock for counter 0  prescaler to 1024 */
	//TCNT0 = 76; // give 10mS interrupt
	//bit_set(TIMSK0,BIT(TOIE0)) ;     /* enable timer overflow 0 as interrupt source */
//}

void temperature()
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
unsigned char button_pressed(void) {
	static int pressedCounter, releasedCounter;
	static bool pressed, released;
	if (get_button() == TRUE) pressedCounter++; else releasedCounter++;
	if (pressedCounter > 10) {
       releasedCounter = 0;
	   pressedCounter = 10;
	   pressed = TRUE;
	   released = FALSE;
	}
	if (releasedCounter > 10) {
		releasedCounter = 10;
		pressedCounter = 0;
		if (pressed == TRUE) {
			released = TRUE;
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

void SetErrorState()
{
	
}



int main(void)
{
	uint16_t temperatureCounterDisplay = 0;
		
	MCUCR=(1<<JTD); // disable JTAG on portC
	MCUCR=(1<<JTD); // disable JTAG on portC
    SSPA_IO_init();      // initialize unit: BIAS = OFF, PSU = ON, RX STATE
	lcd_init(LCD_DISP_ON);
	lcd_clrscr();
	adc_init();
	lm75_init();
	uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) ); 
	sei(); // needed for uart libray
	activeMenu = Imod_menu;  //default menu is current display of all  4 modules
	temperature(); // get temperature
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
			CalculateCurrents();
			CalculatePowerAndSWR();
			temperatureCounterDisplay ++;
			if (temperatureCounterDisplay > 800) {  //400 gives every 2 seconds an update
				temperature();
				temperatureCounterDisplay = 0;
			}
			activeError = CheckForOverload();
			if (activeError != NoError)   // ERROR !
			{
				set_PSU(OFF);
                set_TX(OFF);
			    set_ERROR(ON);
				display_error();
				while(TRUE)    //continue to display error on error menu, until power cycle 
				{
					adc_GetData();
					CalculateCurrents();
					CalculatePowerAndSWR();
					display_error();
				//	uart_tx_status();
				}
			}
			display_Menu(activeMenu);
			
			if (get_ptt() == ON)
			     set_TX(ON);
			else 
                 set_TX(OFF);	
				 	
			if (button_pressed() == TRUE) activeMenu = nextActiveMenu();
			set_LEDtx(ON);  // just to check it is on
			uart_tx_status();
			
//			uart_tx_temperatures();
			//uart_tx_temperatureTrip();
//
//			uart_tx_currentVals();
			//uart_tx_currentADCVals();
			//uart_tx_currentTripADC();
			//uart_tx_currentTripVal();
			//uart_tx_currentCalibrationADC2Amp();
			//uart_tx_currentCalibrationAmp2ADC();
////
			//uart_tx_powerADCVals();
//			uart_tx_powerVals();
			//uart_tx_powerTripADC();
			//uart_tx_powerTripVals();
			//uart_tx_powerCalibrationADC2W();
			//uart_tx_powerCalibrationW2ADC();
			comm_RX_process();

    }
}