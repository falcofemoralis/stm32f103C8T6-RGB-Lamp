#include "stm32f10x.h"
#include "Serial.h"
#define USARTx USART1
/*
STM32F10x.h definitions */
/*------------------------------------------------
 Initialize UART pins, Baudrate
 *---------------------------------------*/

void SER_init (void) {
	int i;
	/* Configure UART1 for 115200 baud */
	RCC->APB2ENR |= (1UL << 2); /*Enable GPIOA clock*/
	GPIOA->CRH   &= ~(0xFFUL <<  4); /* clear PA9, PA10*/
	GPIOA->CRH   |=  (0x0BUL <<  4); /* USART1 Tx (PA9) output push-pull*/
	GPIOA->CRH   |=  (0x04UL <<  8); /* USART1 Rx (PA10) input floating */
	RCC->APB2ENR |=  (1UL <<  14); /*Enable USART#1clock*/
	//8Mhz / 115200 = 69=045h
	USARTx->BRR= 0x0045; /* Configure 115200 baud @8MHz*/
	USARTx->CR3 = 0x0000;
	USARTx->CR2 = 0x0000; /*no parity 1 stop bit,*/
	for (i = 0; i < 0x1000; i++) __NOP(); /* avoid unwanted output */
	USARTx->CR1 = 0x200C; //10 0000 0000 1100 , enable USART,8 bit,enable RX,enable TX     ‬
}
/*------------------------------------------------
 Write character to Serial Port
----------------------------*/
int sendchar (int c) {
	while (!(USARTx->SR & 0x0080));
	USARTx->DR = (c & 0x1FF);
	return (c);
}
/*------------------------------------------------
 Read character from Serial Port (blocking read)
-----------------------------*/
int getkey (void) {
	 while (!(USARTx->SR & 0x0020));
	 return (USARTx->DR);
}
