#ifndef _GP2Y10_H__
#define __GP2Y10_H__
#include "stm32f10x.h"
//#include "gpio.h"
//#include "delay.h"
//#include "uart.h"

#define GP2Y10_CONTROL PAout(0)
#define GP2Y10_PIN     GPIO_PIN_0
#define GP2Y10_CLKLINE RCC_APB2Periph_GPIOA
#define GP2Y10_PORT GPIOA

extern void GP2Y10Configuraiton(void);
extern void GetGP2Y10Value(uint16_t *AQI, uint16_t *PM25);
#endif