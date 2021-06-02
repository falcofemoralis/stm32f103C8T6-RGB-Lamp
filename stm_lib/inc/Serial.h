#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#ifndef _serialh
#define _serialh
void SER_init (void);
int sendchar (int c) ;
int getkey (void);
#endif
