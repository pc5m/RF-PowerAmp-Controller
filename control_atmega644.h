/*
 * control_atmega644.h
 *
 * Created: 11-5-2014 15:06:04
 *  Author: cmobach
 */ 


#ifndef CONTROL_ATMEGA644_H_
#define CONTROL_ATMEGA644_H_

#include <stdint.h>

extern void set_trip_temperature (uint8_t);

extern void set_trip_current_val (uint8_t);
extern void set_trip_current_adc (uint8_t,uint16_t);
extern void set_cal_currents_amp2adc (uint8_t,float);
extern void set_cal_currents_adc2amp (uint8_t,float);

extern void set_trip_powers_val (uint8_t,uint16_t);
extern void set_trip_powers_adc (uint8_t,uint16_t);
extern void set_cal_powers_w2adc (uint8_t,float);
extern void set_cal_powers_adc2w (uint8_t,float);

extern void set_trip_swr_val (float);

#endif /* CONTROL_ATMEGA644_H_ */