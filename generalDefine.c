/*
 * generalDefine.c
 *
 * Created: 21-4-2014 9:50:50
 *  Author: cmobach
 */ 

#include "generalDefine.h"

// define global variable;

calValuesStruct cal_values = {
	.ImodA_max_ADC = 1024,
	.ImodA_max_AMP = 15,
	.ImodA_ADC2AMP = 0.0145,
	.ImodA_AMP2ADC = 68.266,
	
	.ImodB_max_ADC = 1024,
	.ImodB_max_AMP = 15,
	.ImodB_ADC2AMP = 0.0146,
	.ImodB_AMP2ADC = 68.267,

	.ImodC_max_ADC = 1024,
	.ImodC_max_AMP = 15,
	.ImodC_ADC2AMP = 0.0147,
	.ImodC_AMP2ADC = 68.268,

	.ImodD_max_ADC = 1024,
	.ImodD_max_AMP = 15,
	.ImodD_ADC2AMP = 0.0148,
	.ImodD_AMP2ADC = 68.269,

	.Pfwrd_max_ADC = 1024,
	.Pfwrd_max_W   = 1200,
	.Pfwrd_ADC2W   = 1.171875,
	.Pfwrd_W2ADC   = 0.853,

	.Prefl_max_ADC = 1024,
	.Prefl_max_W   = 100,
	.Prefl_ADC2W   = 0.0977,
	.Prefl_W2ADC   = 10.24,

	.Pin_max_ADC   = 1024,
	.Pin_max_W     = 10,
	.Pin_ADC2W     = 0.00978,
	.Pin_W2ADC     = 102.4 };
	
tripValuesStruct trip_values = {
	.ImodA_trip_ADC = 819,  // ADC value ? trip current [A]
	.ImodB_trip_ADC = 820,  // ADC value ? trip current [A]
	.ImodC_trip_ADC = 821,  // ADC value ? trip current [A]
	.ImodD_trip_ADC = 822,  // ADC value  ? trip current [A]
	.Pfwrd_trip_ADC = 981,  // ADC value  ? trip forward power [W]
	.Prefl_trip_ADC = 819,  // ADC value  ? trip reflected power [W]
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
enum ErrorStates activeError = NoError;

uint8_t SSPAstatus = 0;

uint8_t autoTransmitCurrentVals = FALSE;
uint8_t autoTransmitPowerVals = FALSE;
uint8_t controlConnected = FALSE;
uint8_t autoTransmitPowerADC  = FALSE;
uint8_t autoTransmitCurrentADC  = FALSE;
uint8_t autoTransmitTemperature  = FALSE;