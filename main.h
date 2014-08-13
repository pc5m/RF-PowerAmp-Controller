/*
 * main.h
 *
 * Created: 11-5-2014 15:06:04
 *  Author: C. Mobach,  PC5M
 */ 

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>

extern void main_set_trip_temperature (uint8_t);
extern void main_set_trip_current_val (uint8_t);
extern void main_set_trip_current_adc (uint8_t,uint16_t);
extern void main_set_cal_currents_amp2adc (uint8_t,float);
extern void main_set_cal_currents_adc2amp (uint8_t,float);
extern void main_set_trip_powers_val (uint8_t,uint16_t);
extern void main_set_trip_powers_adc (uint8_t,uint16_t);
extern void main_set_trip_swr_val (float);
extern void main_set_powerCalibrationADC2W_RC_B(uint8_t, uint8_t, uint16_t* , float*, float*);

#endif /* MAIN_H_ */