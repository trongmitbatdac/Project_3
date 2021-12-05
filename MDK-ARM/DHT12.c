#include "DHT12.h"
#include <stdio.h>
#include "i2c-lcd.h"
//#include "delay.h"

extern I2C_HandleTypeDef hi2c2;

#define DHT12_ADDRESS 0xB8<<1

void DHT_read(void)
{
	//HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}