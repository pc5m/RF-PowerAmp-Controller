#include <stdint.h>


#define LM75_A 0b10010000 // Base address = write = A0(7) = 0	  A1(6) = 0		A2(5) = 0  
#define LM75_B 0b10010010 // Base address = write = A0(7) = +VSS  A1(6) = 0	    A2(5) = 0 
#define LM75_C 0b10010100 // Base address = write = A0(7) = 0	  A1(6) = +VSS	A2(5) = 0 
#define LM75_D 0b10010110 // Base address = write = A0(7) = +VSS  A1(6) = +VSS	A2(5) = 0 


/**
 @brief initialize the I2C master interace. Need to be called only once 
 @param  void
 @return none
 */
void lm75_init(void);


/**
 @brief get temperature 1 degree resolution of LM75
 @param  LM75 write (base) address
 @return uint8_t, temperature
 */
uint8_t lm75_Temp(uint8_t lm75_address);




