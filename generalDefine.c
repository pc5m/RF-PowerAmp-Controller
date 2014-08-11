/*
 * generalDefine.c
 *
 * Created: 21-4-2014 9:50:50
 *  Author: cmobach
 */ 

#include "generalDefine.h"

// define global variable;



calValuesStruct cal_values;  //No defaults set, default will be read from EEPROM on startup 
	
calValuesStruct EEcal_values = {
		.ImodA_ADC2AMP = 0.0145,
		.ImodA_AMP2ADC = 68.266,
		.ImodB_ADC2AMP = 0.0145,
		.ImodB_AMP2ADC = 68.266,
		.ImodC_ADC2AMP = 0.0145,
		.ImodC_AMP2ADC = 68.266,
		.ImodD_ADC2AMP = 0.0145,
		.ImodD_AMP2ADC = 68.266,

		.Pfwrd_max_ADC = 1024,
		.Pfwrd_max_W   = 1200,

		.Prefl_max_ADC = 1024,
		.Prefl_max_W   = 100,

		.Pin_max_ADC   = 1024,
		.Pin_max_W     = 10
		};
		
		
calPowerValuesStruct calPower_values;  //No defaults set, default will be read from EEPROM on startup 		
		
calPowerValuesStruct EEcalPower_values = {
	.Pfwrd_nr = 5,		// Nr of  Pforwards calibration points
    .Prefl_nr = 5,			// Nr of  Prefl calibration points
	.Pin_nr = 5,			// Nr of  Pin calibration points
	
	.Pfwrd_ADC =      { 0, 100, 250, 500, 1000 },  // Array of ADC's calibration values
	.Pfwrd_ADC2W_RC = { 2.5, 2.33, 1.2, 0.6    },  // Array of RC's corresponding to ADC calibration values
	.Pfwrd_ADC2W_B =  { 0, 16.7, 300, 600      },  // Array of B's corresponding to ADC calibration values
    
	.Prefl_ADC =      { 0, 100, 250, 500, 1000 },  // Array of ADC's calibration values
    .Prefl_ADC2W_RC = { 0.25, 0.23, 0.12, 0.06 },  // Array of RC's corresponding to ADC calibration values
    .Prefl_ADC2W_B =  { 0, 1.67, 30, 60        },  // Array of B's corresponding to ADC calibration values

    .Pin_ADC =        { 0, 100, 250, 500, 1000    }, // Array of ADC's calibration values
    .Pin_ADC2W_RC =   { 0.02, 0.027, 0.012, 0.006 }, // Array of RC's corresponding to ADC calibration values
    .Pin_ADC2W_B =    { 0, -0.667, 3, 6           }, // Array of B's corresponding to ADC calibration values

    .Pfwrd_W =        { 0, 250, 600, 900, 1200 },  // Array of W's calibration values
    .Pfwrd_W2ADC_RC = { 0.4, 0.43, 0.83, 1.67  },  // Array of RC's corresponding to W calibration values
    .Pfwrd_W2ADC_B =  { 0, -7.14, -250, -1000  },  // Array of B's corresponding to W calibration values

    .Prefl_W  =       { 0, 25, 60, 90, 120        }, // Array of W's calibration values
    .Prefl_W2ADC_RC = { 4, 4.29, 8.33, 16.67      }, // Array of RC's corresponding to W calibration values
    .Prefl_W2ADC_B =  { 0, -7.14, -250.00, -1000  }, // Array of B's corresponding to W calibration values

    .Pin_W =          { 0, 2, 6, 9, 12            }, // Array of W's calibration values
    .Pin_W2ADC_RC =   { 50, 37.5, 83.33, 166.67   }, // Array of RC's corresponding to W calibration values
    .Pin_W2ADC_B =    { 0, 25, -250, 1000         }  // Array of B's corresponding to W calibration values
 };		
		
	
tripValuesStruct trip_values;  //No defaults set, default will be read from EEPROM on startup
	
tripValuesStruct EEtrip_values = {
		.ImodA_trip_ADC = 820,  // ADC value ? trip current [A]
		.ImodB_trip_ADC = 820,  // ADC value ? trip current [A]
		.ImodC_trip_ADC = 820,  // ADC value ? trip current [A]
		.ImodD_trip_ADC = 820,  // ADC value  ? trip current [A]
		.Pfwrd_trip_ADC = 981,  // ADC value  ? trip forward power [W]
		.Prefl_trip_ADC = 820,  // ADC value  ? trip reflected power [W]
		.Pin_trip_ADC   = 820,  // ADC value  ? trip input power [W]
		.SWR_trip_ADC   = 0,    // ADC value  ? trip SWR
		.Imod_trip_AMP  = 12,   // current trip value  [A]
		.Pfwrd_trip_W   = 1150, // forward power trip value  [W]
		.Prefl_trip_W   = 80,   // reflected power trip value  [W]
		.Pin_trip_W     = 8,    // input power trip value  [W]
		.swr_trip       = 2,    // SWR  trip value
		.temp_trip      = 50 };   // temperature trip  value [?C]

tempValuesStruct temp_values = {
	.tempA = 25, 
	.tempB = 26, 
	.tempC = 27, 
	.tempD = 28,
	.tempMax = 28 };

//Raw ADC current mod A
adcValuesStruct adc_values = {
	.iModuleA_ADC = 500, 
	.iModuleB_ADC = 600, 
	.iModuleC_ADC = 700, 
	.iModuleD_ADC = 800,
    .pwrFwrd_ADC  = 1000, 
	.pwrRefl_ADC  = 1000, 
	.pwrIn_ADC    = 1000 };
	
currentValuesStruct current = {
	.moduleA = 7.3, 
	.moduleB = 8.8, 
	.moduleC = 10.3, 
	.moduleD = 11.7};
	
powerValuesStruct power = {
	.fwrd = 1113.3, 
	.refl = 83, 
	.input = 7.3, 
	.swr = 1.5 };

enum activeMenus activeMenu = Imod_menu;
enum activeMenus nextMenu = Gen_Menu;
enum ErrorStates activeError = NoError;

uint8_t SSPAstatus = 0;

uint8_t autoTransmitCurrentVals = FALSE;
uint8_t autoTransmitPowerVals = FALSE;
uint8_t controlConnected = FALSE;
uint8_t autoTransmitPowerADC  = FALSE;
uint8_t autoTransmitCurrentADC  = FALSE;
uint8_t autoTransmitTemperature  = FALSE;

uint16_t decay = 250; // 250 standard decay value, to be overridden at startup (holding menu button)