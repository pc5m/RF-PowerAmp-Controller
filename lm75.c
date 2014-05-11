# include "LM75.h"
# include "i2cmaster.h"
#include <stdint.h>

void lm75_init(void)
{
	i2c_init();
}

uint8_t lm75_Temp(uint8_t lm75_address)
{
	uint8_t returnCode, temp;
	returnCode=i2c_start(lm75_address+I2C_READ);
	if(returnCode==0) {
		temp  = i2c_readAck();
		// temp_05 =i2c_readNak();  //get 0.5 degree value, not needed
		i2c_stop();
		return(temp);
	}
	else return(255);
}
