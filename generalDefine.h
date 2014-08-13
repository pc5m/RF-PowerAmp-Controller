/*
 * generalDefine.h
 *
 * Created: 10-12-2013
 * Author: C. Mobach,  PC5M
 */ 


#ifndef generalDefine_H_
#define generalDefine_H_


#include <stdint.h>
#include <avr/eeprom.h>

/*
** constants/macros
*/

#define FW_VERSION_NR 1
#define F_CPU 18432000UL      /*18,432 MHz external oscillator */

#define NrOfPowerCalibrationPoints 5

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
#define ConvertTo_uint16(b1,b2) (uint16_t)(((b1)<<8)|(b2))


#define DDR(x) (*(&x - 1))    /* address of data direction register of port x */
#define PIN(x) (*(&x - 2))    /* address of input register of port x          */
#define PORT(x) (x)

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

/*
** type definitions
*/

// structure holding all ADC values 
typedef struct {
	uint16_t iModuleA_ADC;
	uint16_t iModuleB_ADC;
	uint16_t iModuleC_ADC;
	uint16_t iModuleD_ADC;
	uint16_t pwrFwrd_ADC;
	uint16_t pwrRefl_ADC;
	uint16_t pwrIn_ADC;
} adcValuesStruct;

// structure holding the module currents values [A]
typedef struct {
	float moduleA;
	float moduleB;
	float moduleC;
	float moduleD;
	float moduleTotal;
} currentValuesStruct;

// structure holding the power and SWR values
typedef struct {
	float fwrd;
	float refl;
	float input;
	float swr;
} powerValuesStruct;

// structure holding the temperature values 
typedef struct
{
	uint8_t tempA;
	uint8_t tempB;
	uint8_t tempC;
	uint8_t tempD;
	uint8_t tempMax;
}  tempValuesStruct;

// structure holding the trip values (ADC as well as real value)
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

// structure holding the calibration factors for currents
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

	uint16_t  Prefl_max_ADC; // ADC value  ? maximum forward power [W]
	uint16_t  Prefl_max_W;   // maximum value of power, used in display

	uint16_t  Pin_max_ADC;   // ADC value  ? maximum forward power [W]
	uint16_t  Pin_max_W; 	 // maximum value of power, used in display
} calValuesStruct;

// structure holding the calibration factors for powers
typedef struct {
	uint8_t  Pfwrd_nr;			// Nr of  Pforwards calibration points
	uint8_t  Prefl_nr;			// Nr of  Prefl calibration points
	uint8_t  Pin_nr;			// Nr of  Pin calibration points	
	
	uint16_t Pfwrd_ADC[NrOfPowerCalibrationPoints];        // Array of ADC's calibration values (forward power)
	float    Pfwrd_ADC2W_RC[NrOfPowerCalibrationPoints-1]; // Array of RC's corresponding to ADC calibration values (forward power)
	float    Pfwrd_ADC2W_B[NrOfPowerCalibrationPoints-1];  // Array of B's corresponding to ADC calibration values (forward power)

	uint16_t Prefl_ADC[NrOfPowerCalibrationPoints];        // Array of ADC's calibration values (reflected power)
	float    Prefl_ADC2W_RC[NrOfPowerCalibrationPoints-1]; // Array of RC's corresponding to ADC calibration values (reflected power)
	float Prefl_ADC2W_B[NrOfPowerCalibrationPoints-1];     // Array of B's corresponding to ADC calibration values (reflected power)
	
	uint16_t Pin_ADC[NrOfPowerCalibrationPoints];          // Array of ADC's calibration values (input power)
	float    Pin_ADC2W_RC[NrOfPowerCalibrationPoints-1];   // Array of RC's corresponding to ADC calibration values (input power)
	float Pin_ADC2W_B[NrOfPowerCalibrationPoints-1];       // Array of B's corresponding to ADC calibration values (input power)

	uint16_t Pfwrd_W[NrOfPowerCalibrationPoints];          // Array of W's calibration values (forward power) 
	float    Pfwrd_W2ADC_RC[NrOfPowerCalibrationPoints-1]; // Array of RC's corresponding to W calibration values (forward power)
	float Pfwrd_W2ADC_B[NrOfPowerCalibrationPoints-1];     // Array of B's corresponding to W calibration values (forward power)

	uint16_t Prefl_W[NrOfPowerCalibrationPoints];          // Array of W's calibration values (reflected power)
	float    Prefl_W2ADC_RC[NrOfPowerCalibrationPoints-1]; // Array of RC's corresponding to W calibration values (reflected power)
	float Prefl_W2ADC_B[NrOfPowerCalibrationPoints-1];     // Array of B's corresponding to W calibration values (reflected power)

	uint16_t Pin_W[NrOfPowerCalibrationPoints];            // Array of W's calibration values (input power)
	float    Pin_W2ADC_RC[NrOfPowerCalibrationPoints-1];   // Array of RC's corresponding to W calibration values (input power)
	float Pin_W2ADC_B[NrOfPowerCalibrationPoints-1];       // Array of B's corresponding to W calibration values (input power)
} calPowerValuesStruct;


/*
** Enumerations
*/

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
	
	
/*
** global variables
*/
extern calValuesStruct cal_values;    //calibration factors for currents
extern tripValuesStruct trip_values;  //trip values
extern tempValuesStruct temp_values;  //temperature values
extern adcValuesStruct adc_values;    //raw ADC values for all inputs
extern currentValuesStruct current;   // calibrated I current values
extern powerValuesStruct power;       //calibrated P values
extern calPowerValuesStruct calPower_values; //calibration factors for power and SWR

extern enum activeMenus activeMenu;
extern enum activeMenus nextMenu;
extern enum ErrorStates activeError;

extern uint8_t SSPAstatus;   // SSPA status with flags

extern uint16_t decay;       // Decay factor for peak hold

extern uint8_t autoTransmitCurrentVals;  //when set, current values are transmitted regularly via serial port
extern uint8_t autoTransmitCurrentADC;   //when set, current ADC values are transmitted regularly via serial port
extern uint8_t autoTransmitPowerVals;    //when set, power values are transmitted regularly via serial port
extern uint8_t autoTransmitPowerADC;     //when set, power ADC values are transmitted regularly via serial port
extern uint8_t autoTransmitTemperature;  //when set, temperature values are transmitted regularly via serial port
extern uint8_t controlConnected;         //when set, SSPA is connected to client application

/* EEPROM global values */
extern tripValuesStruct EEMEM EEtrip_values;         //trip values as stored in local EEPROM
extern calValuesStruct EEMEM EEcal_values;           //calibration values for current as stored in local EEPROM
extern calPowerValuesStruct EEMEM EEcalPower_values; //calibration values for power as stored in local EEPROM

#endif /* generalDefine_H_ */