/*
 * generalDefine.h
 *
 * Created: 10-12-2013
 * Author: cmobach
 */ 


#ifndef generalDefine_H_
#define generalDefine_H_

/*************************************************************************
 Title	:   C include file general define's and macro's
 Author:    Carel Mobach
 File:	    $Id: generalDefine.h,v 1.0.0.0 2013/12/10 14:22:00 $
 Software:  AVR-GCC 4.5.1
 Hardware:  ATMEGA644
***************************************************************************/

/**
 
 @brief Basic routines 
      
 @author Carel Mobach

*/

#include <stdint.h>
#include <avr/eeprom.h>

#define FW_VERSION_NR 1
#define F_CPU 18432000UL      /*18,432 MHz external oscillator */

#define ON 1
#define OFF 0
#define TRUE 1
#define FALSE 0



#define bit_get(p,m) ((p) & (m))
#define bit_set(p,m) ((p) |= (m))
#define bit_clear(p,m) ((p) &= ~(m))
#define BIT(x) (0x01 << (x))
#define bit_flip(p,m) ((p) ^= (m))
#define bit_write(c,p,m) (c ? bit_set(p,m) : bit_clear(p,m))

#define ConvertTo_float(b1,b2,b3,b4) (float)(((b1)<<24)|((b2)<<16)|((b3)<<8)|(b4))
// convertTo_uint16(MSB, LSB)
#define ConvertTo_uint16(b1,b2) (uint16_t)(((b1)<<8)|(b2))

/*
** constants/macros
*/
#define DDR(x) (*(&x - 1))    /* address of data direction register of port x */
#define PIN(x) (*(&x - 2))    /* address of input register of port x          */
#define PORT(x) (x)

/*
** global variables
*/

typedef struct {
	uint16_t iModuleA_ADC;
	uint16_t iModuleB_ADC;
	uint16_t iModuleC_ADC;
	uint16_t iModuleD_ADC;
	uint16_t pwrFwrd_ADC;
	uint16_t pwrRefl_ADC;
	uint16_t pwrIn_ADC;
} adcValuesStruct;

typedef struct {
	float moduleA;
	float moduleB;
	float moduleC;
	float moduleD;
	float moduleTotal;
} currentValuesStruct;

typedef struct {
	float fwrd;
	float refl;
	float input;
	float swr;
} powerValuesStruct;

typedef struct
{
	uint8_t tempA;
	uint8_t tempB;
	uint8_t tempC;
	uint8_t tempD;
	uint8_t tempMax;
}  tempValuesStruct;

typedef struct {
	uint16_t ImodA_trip_ADC; // ADC value ? trip current [A]
	uint16_t ImodB_trip_ADC; // ADC value ? trip current [A]
	uint16_t ImodC_trip_ADC; // ADC value ? trip current [A]
	uint16_t ImodD_trip_ADC; // ADC value  ? trip current [A]
	uint16_t Pfwrd_trip_ADC; // ADC value  ? trip forward power [W]
	uint16_t Prefl_trip_ADC; // ADC value  ? trip reflected power [W]
	uint16_t Pin_trip_ADC;   // ADC value  ? trip input power [W]
	uint16_t SWR_trip_ADC;   // ADC value  ? trip SWR
	uint16_t Imod_trip_AMP;  // current trip value  [A]
	uint16_t Pfwrd_trip_W;   // forward power trip value  [W]
	uint16_t Prefl_trip_W;   // reflected power trip value  [W]
	uint16_t Pin_trip_W;     // input power trip value  [W]
	float    swr_trip;       // SWR  trip value
	uint8_t  temp_trip;      // temperature trip  value [?C]
} tripValuesStruct;

typedef struct {
	float    ImodA_ADC2AMP;  // factor to convert  ADC value to current in [A]
	float    ImodA_AMP2ADC;  // factor to convert current [A] to ADC value

	float    ImodB_ADC2AMP;  // factor to convert  ADC value to current in [A]
	float    ImodB_AMP2ADC;	 // factor to convert current [A] to ADC value

	float    ImodC_ADC2AMP;  // factor to convert  ADC value to current in [A]
	float    ImodC_AMP2ADC;	 // factor to convert current [A] to ADC value

	float    ImodD_ADC2AMP;  // factor to convert  ADC value to current in [A]
	float    ImodD_AMP2ADC;	 // factor to convert current [A] to ADC value

	uint16_t Pfwrd_max_ADC;  // ADC value  ? maximum forward power [W]
	uint16_t Pfwrd_max_W;    // maximum value of power, used in display
	float    Pfwrd_ADC2W; 	 // factor to convert ADC value to forward power  [W]
	float    Pfwrd_W2ADC;	 // factor to convert forward power  [W] to ADC value

	uint16_t  Prefl_max_ADC; // ADC value  ? maximum forward power [W]
	uint16_t  Prefl_max_W;   // maximum value of power, used in display
	float     Prefl_ADC2W; 	 // factor to convert ADC value to reflected power  [W]
	float     Prefl_W2ADC;	 // factor to convert reflected power  [W] to ADC value
	uint16_t  Pin_max_ADC;   // ADC value  ? maximum forward power [W]
	uint16_t  Pin_max_W; 	 // maximum value of power, used in display
	float     Pin_ADC2W;     // factor to convert ADC value to input power [W]
	float     Pin_W2ADC;     // factor to convert input power  [W] to ADC value
} calValuesStruct;

#define NrOfPowerCalibrationPoints 5

typedef struct {
	uint8_t  Pfwrd_nr;			// Nr of  Pforwards calibration points
	uint8_t  Prefl_nr;			// Nr of  Prefl calibration points
	uint8_t  Pin_nr;			// Nr of  Pin calibration points	
	
	uint16_t Pfwrd_ADC[NrOfPowerCalibrationPoints];       // Array of ADC's calibration values
	float    Pfwrd_ADC2W_RC[NrOfPowerCalibrationPoints-1];  // Array of RC's corresponding to ADC calibration values
	float    Pfwrd_ADC2W_B[NrOfPowerCalibrationPoints-1];   // Array of B's corresponding to ADC calibration values

	uint16_t Prefl_ADC[NrOfPowerCalibrationPoints];       // Array of ADC's calibration values
	float    Prefl_ADC2W_RC[NrOfPowerCalibrationPoints-1];  // Array of RC's corresponding to ADC calibration values
	float Prefl_ADC2W_B[NrOfPowerCalibrationPoints-1];   // Array of B's corresponding to ADC calibration values
	
	uint16_t Pin_ADC[NrOfPowerCalibrationPoints];       // Array of ADC's calibration values
	float    Pin_ADC2W_RC[NrOfPowerCalibrationPoints-1];  // Array of RC's corresponding to ADC calibration values
	float Pin_ADC2W_B[NrOfPowerCalibrationPoints-1];   // Array of B's corresponding to ADC calibration values

	uint16_t Pfwrd_W[NrOfPowerCalibrationPoints];       // Array of W's calibration values
	float    Pfwrd_W2ADC_RC[NrOfPowerCalibrationPoints-1];  // Array of RC's corresponding to W calibration values
	float Pfwrd_W2ADC_B[NrOfPowerCalibrationPoints-1];   // Array of B's corresponding to W calibration values

	uint16_t Prefl_W[NrOfPowerCalibrationPoints];       // Array of W's calibration values
	float    Prefl_W2ADC_RC[NrOfPowerCalibrationPoints-1];  // Array of RC's corresponding to W calibration values
	float Prefl_W2ADC_B[NrOfPowerCalibrationPoints-1];   // Array of B's corresponding to W calibration values

	uint16_t Pin_W[NrOfPowerCalibrationPoints];       // Array of W's calibration values
	float    Pin_W2ADC_RC[NrOfPowerCalibrationPoints-1];  // Array of RC's corresponding to W calibration values
	float Pin_W2ADC_B[NrOfPowerCalibrationPoints-1];   // Array of B's corresponding to W calibration values
} calPowerValuesStruct;


enum activeMenus {Imod_menu, Pall_menu, Gen_Menu, Temp_menu};
	
// ErrorStates, make sure enum is using LSB 4 bit's, will be OR-ed with SSPAstatus value for status signalling
enum ErrorStates {
	NoError = 0, 
	ImodA   = 1, 
	ImodB   = 2,
	ImodC   = 3,
	ImodD   = 4, 
    Pfwrd   = 5, 
	Prefl   = 6, 
	Pin     = 7, 
	SWR     = 8, 
    TempA   = 9, 
	TempB   = 10, 
	TempC   = 11, 
	TempD   = 12 } ;
	
	
// declare global variable;
extern calValuesStruct cal_values;
extern tripValuesStruct trip_values;
extern tempValuesStruct temp_values;
extern adcValuesStruct adc_values;
extern currentValuesStruct current;  // calibrated I current values
extern powerValuesStruct power;      //calibrated P values
extern calPowerValuesStruct calPower_values;

extern enum activeMenus activeMenu;
extern enum activeMenus nextMenu;
extern enum ErrorStates activeError;

extern uint8_t SSPAstatus;   // status with flags
#define FLAG_ERROR   7
#define FLAG_TX_ON   6
#define FLAG_PSU_ON  5
#define FLAG_BIAS_ON 4

#define MODULE_A 0
#define MODULE_B 1
#define MODULE_C 2
#define MODULE_D 3

#define POWER_FWD  0
#define POWER_REFL 1
#define POWER_IN   2
#define POWER_SWR  3

extern uint8_t autoTransmitCurrentVals;
extern uint8_t autoTransmitCurrentADC;
extern uint8_t autoTransmitPowerVals;
extern uint8_t autoTransmitPowerADC;
extern uint8_t autoTransmitTemperature;
extern uint8_t controlConnected;

/* EEPROM global values */
extern tripValuesStruct EEMEM EEtrip_values;
extern calValuesStruct EEMEM EEcal_values;
extern calPowerValuesStruct EEMEM EEcalPower_values;

#endif /* generalDefine_H_ */