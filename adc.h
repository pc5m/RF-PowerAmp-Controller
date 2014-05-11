/*
 * adc.h
 *
 * Created: 19-4-2014 19:41:25
 *  Author: cmobach
 */ 


#ifndef ADC_H_
#define ADC_H_

#define ADC_ImodA 0
#define ADC_ImodB 1
#define ADC_ImodC 2
#define ADC_ImodD 3
#define ADC_Pfwd  4
#define ADC_Prefl 5
#define ADC_Pin   6

#include <stdint.h>



/** 
 *  @name Functions
 */


/**
 @brief    Initialize ADC
 @param    none             
 @return   none
*/
extern void adc_init();



/**
 @brief    Get all the data from the ADC channels
 @param    none             
 @return   none
*/
extern void adc_GetData();


#endif /* ADC_H_ */