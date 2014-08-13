/*
 * adc.c
 *
 * Created: 19-4-2014 20:04:45
 *  Author: C. Mobach,  PC5M
 */ 

#include "adc.h"
#include "generalDefine.h"
#include <avr/sfr_defs.h> // for loop_until_bit_clear
#include <avr/io.h>



void adc_init()
{
	// External reference at AREF pin
	// Divide by 128 for 144KHz clock
	// Single conversion, Enable ADC
	// Disable digital input on ADC input pins
	bit_set(ADCSRA,BIT(ADPS0) | BIT(ADPS1) | BIT(ADPS2) );
	bit_set(ADCSRA,BIT(ADEN));
	bit_set(DIDR0, BIT(ADC0D) | BIT(ADC1D) |BIT(ADC2D) |BIT(ADC3D) |BIT(ADC4D) |
	         BIT(ADC5D) |BIT(ADC6D) | BIT(ADC7D) );
}

static void adc_select(uint8_t ADCchannel)
{
	ADMUX = (ADMUX & 0xE0) | (ADCchannel & 0x1F);   //select channel (MUX0-4 bits)
}

static uint16_t adc_read(void)
{
	bit_set(ADCSRA,BIT(ADSC));  // ADC Start Conversion
	loop_until_bit_is_clear(ADCSRA,ADSC); //wait till conversion is finished 
	return (ADCW);
}


uint16_t adc_peakhold (uint16_t newADC, uint16_t* lastMaxADC, uint16_t* DecayCounter, const uint16_t Decay)
{
	if (newADC > *lastMaxADC){
		*DecayCounter = 0;
		*lastMaxADC = newADC;	
	} 
	else {
		(*DecayCounter)++;
		if (*DecayCounter > Decay) {
			*DecayCounter = Decay;
			*lastMaxADC = newADC;
		}
	}
	return (*lastMaxADC);
}

void adc_GetData()
{
	static uint16_t pwrIn_ADC_lastmax, pwrIn_ADC_decayCounter;
	static uint16_t pwrFwrd_ADC_lastmax, pwrFwrd_ADC_decayCounter;
	static uint16_t pwrPrefl_ADC_lastmax, pwrPrefl_ADC_decayCounter;
	
	adc_select(ADC_ImodA);
	adc_values.iModuleA_ADC = adc_read();
	adc_select(ADC_ImodB);
	adc_values.iModuleB_ADC = adc_read();
	adc_select(ADC_ImodC);
	adc_values.iModuleC_ADC = adc_read();
	adc_select(ADC_ImodD);
	adc_values.iModuleD_ADC = adc_read();
	adc_select(ADC_Pfwd);
	//adc_values.pwrFwrd_ADC = adc_read();
	adc_values.pwrFwrd_ADC = adc_peakhold( adc_read(), &pwrFwrd_ADC_lastmax, &pwrFwrd_ADC_decayCounter, decay);
	adc_select(ADC_Prefl);
	//adc_values.pwrRefl_ADC = adc_read();
	adc_values.pwrRefl_ADC = adc_peakhold( adc_read(), &pwrPrefl_ADC_lastmax, &pwrPrefl_ADC_decayCounter, decay);
	adc_select(ADC_Pin);
	//adc_values.pwrIn_ADC = adc_read();
	adc_values.pwrIn_ADC = adc_peakhold( adc_read(), &pwrIn_ADC_lastmax, &pwrIn_ADC_decayCounter, decay);
}

