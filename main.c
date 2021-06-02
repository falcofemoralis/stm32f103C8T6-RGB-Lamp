#include "stm32F10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"

#define RED 0
#define GREEN 1
#define BLUE 2
#define YELLOW 3
#define PURPLE 4
#define CYAN 5
#define WHITE 6

#define REDPWM TIM_SetCompare2,TIM_SetCompare1,TIM_SetCompare3,TIM_SetCompare2,TIM3,TIM3,TIM4,TIM2,j
#define GREENPWM TIM_SetCompare4,TIM_SetCompare1,TIM_SetCompare4,TIM_SetCompare3,TIM3,TIM4,TIM4,TIM2,j
#define BLUEPWM TIM_SetCompare3,TIM_SetCompare2,TIM_SetCompare1,TIM_SetCompare4,TIM3,TIM4,TIM2,TIM2,j
/**
 * Встановлюємо порядок та яскравість світлодіодів
 */
//яскравість (максимальне значення is 5000)
#define brightness 2000
//Порядок кольорів
int standartColors[50]={RED,GREEN,BLUE,YELLOW,PURPLE,CYAN,WHITE},colorsCount=2;
/*
 *
 */

//USART
int USART_colorsCount=0,USART_colors[50];
char received[50], rec_len=0;
//USB
extern volatile char Receive_Buffer[64];
extern volatile int Receive_length,length;
char Send_Buffer[64], usbReceived[32];
int packet_sent=1,packet_receive=1,buff=0;
int USB_colors[50],USB_colorsCount=0;
//other
int ForCompare = brightness;


void delay_init(){
    RCC->APB2ENR|=RCC_APB2ENR_TIM1EN;
	TIM1->PSC= 18000-1;
	TIM1->CR1 |= TIM_CR1_OPM;
}

void delay_ms(uint16_t value)
{
   value*=4;
   TIM1->ARR = value;
   TIM1->CNT = 0;
   TIM1->CR1 = TIM_CR1_CEN;
   while((TIM1->SR & TIM_SR_UIF)==0){}
   TIM1->SR &= ~TIM_SR_UIF;
}

void USART1_Send(char chr) {
	while(!(USART1->SR & USART_SR_TC));
	USART1->DR = chr;
}

void USART1_Send_String(char* str) {
	int i=0;
	while(str[i])
	USART1_Send(str[i++]);
}

void USART_init(void){
	RCC->APB2ENR|= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
	GPIOA->CRH &= !GPIO_CRH_CNF9;
	GPIOA->CRH |=  GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1 | GPIO_CRH_CNF10_0;
	//baudrate = 9600
	USART1->BRR = 0x1D4C;
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
	USART1->CR1 |= USART_CR1_UE;
	USART1->CR1 |= USART_CR1_RXNEIE;
	NVIC_EnableIRQ(USART1_IRQn);
	USART1_Send_String("Enter 1 color (red,green,blue,yellow,purple,cyan,white) and ok to finish\r\n");
}

void processCommand(char *cmd)
{
	if(strncmp(cmd, "red", 3) == 0){
		USART1_Send_String(":red\r\n");
		USART_colors[USART_colorsCount]=0;
		USART_colorsCount++;
	}

	else if(!strncmp(cmd, "green", 5)){
		USART1_Send_String(":green\r\n");
		USART_colors[USART_colorsCount]=1;
		USART_colorsCount++;
	}

	else if(!strncmp(cmd, "blue", 4)){
		USART1_Send_String(":blue\r\n");
		USART_colors[USART_colorsCount]=2;
		USART_colorsCount++;
	}

	else if(!strncmp(cmd, "yellow", 6)){
		USART1_Send_String(":yellow\r\n");
		USART_colors[USART_colorsCount]=3;
		USART_colorsCount++;
	}

	else if(!strncmp(cmd, "purple", 6)){
		USART1_Send_String(":purple\r\n");
		USART_colors[USART_colorsCount]=4;
		USART_colorsCount++;

	}

	else if(!strncmp(cmd, "cyan", 4)){
		USART1_Send_String(":cyan\r\n");
		USART_colors[USART_colorsCount]=5;
		USART_colorsCount++;
	}

	else if(!strncmp(cmd, "white", 5)){
		USART1_Send_String(":white\r\n");
		USART_colors[USART_colorsCount]=6;
		USART_colorsCount++;
	}

	else if(!strncmp(cmd, "ok", 2)){
		USART1_Send_String(":All colors accepted\r\n");
		colorsCount = USART_colorsCount;
		for(int i=0;i<USART_colorsCount;i++)
			standartColors[i]=USART_colors[i];
		USART_colorsCount = 0;
	}
	else{
		USART1_Send_String(":Wrong color!\r\n");;
	}
}

void USART1_IRQHandler(void) {
	if (USART1->SR & USART_SR_RXNE) {
		USART1->SR&=~USART_SR_RXNE;
		short unsigned int dat = USART1->DR;
		if(dat==13)
		{
			received[rec_len++]=0;
			processCommand(received);
			rec_len=0;
		}
		else
		{
			USART1_Send(dat);
			received[rec_len++]=dat;
		}
	}
}

void PWM_init(){
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	    GPIO_InitTypeDef pwmGPIO;
	    TIM_TimeBaseInitTypeDef tim;
	    TIM_OCInitTypeDef pwm;

	    pwmGPIO.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_5|GPIO_Pin_4|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	    pwmGPIO.GPIO_Mode = GPIO_Mode_AF_PP;
	    pwmGPIO.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_Init(GPIOB, &pwmGPIO);

	    pwmGPIO.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	    pwmGPIO.GPIO_Mode = GPIO_Mode_AF_PP;
	    pwmGPIO.GPIO_Speed = GPIO_Speed_50MHz;
	    GPIO_Init(GPIOA, &pwmGPIO);

	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE);

	    tim.TIM_Period = 5000;
	   	tim.TIM_Prescaler = 0;
	   	tim.TIM_ClockDivision = 0;
	   	tim.TIM_CounterMode = TIM_CounterMode_Up;
	    TIM_TimeBaseInit(TIM2, &tim);
	    tim.TIM_Period = 5000;
	    tim.TIM_Prescaler = 0;
	    tim.TIM_ClockDivision = 0;
	    tim.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM3, &tim);
		tim.TIM_Period = 5000;
		tim.TIM_Prescaler = 0;
		tim.TIM_ClockDivision = 0;
		tim.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM4, &tim);

		pwm.TIM_OCMode = TIM_OCMode_PWM1;
		pwm.TIM_OutputState = TIM_OutputState_Enable;
		pwm.TIM_Pulse = 0;
		pwm.TIM_OCPolarity = TIM_OCPolarity_High;

	    // TIM2, channel 1 PA0
	    TIM_OC1Init(TIM2, &pwm);
	    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	    // TIM2, channel 2 PA1
	    TIM_OC2Init(TIM2, &pwm);
	    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	    // TIM2, channel 3 PA2
	    TIM_OC3Init(TIM2, &pwm);
	    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	    // TIM2, channel 4 PA3
	    TIM_OC4Init(TIM2, &pwm);
	    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	    //load TIM4 settings
	    TIM_Cmd(TIM2, ENABLE);
	    TIM_ARRPreloadConfig(TIM2, ENABLE);

	    // TIM3, channel 1 PB4
	    TIM_OC1Init(TIM3, &pwm);
	    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	    // TIM3, channel 2 PB5 remap port
	    TIM_OC2Init(TIM3, &pwm);
	    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	    // TIM3, channel 3 PB0
	    TIM_OC3Init(TIM3, &pwm);
	    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	    // TIM3, channel 4 PB1
	    TIM_OC4Init(TIM3, &pwm);
	    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	    //load TIM3 settings
	    TIM_Cmd(TIM3, ENABLE);
	    TIM_ARRPreloadConfig(TIM3, ENABLE);

	    // TIM4, channel 1 PB6
	    TIM_OC1Init(TIM4, &pwm);
	    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	    // TIM4, channel 2 PB7
	    TIM_OC2Init(TIM4, &pwm);
	    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	    // TIM4, channel 3 PB8
	    TIM_OC3Init(TIM4, &pwm);
	    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	    // TIM4, channel 4 PB9
	    TIM_OC4Init(TIM4, &pwm);
	    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	    //load TIM4 settings
	    TIM_Cmd(TIM4, ENABLE);
	    TIM_ARRPreloadConfig(TIM4, ENABLE);
}

void SetPWM(void(*SetCompare1)(TIM_TypeDef*,int),void(*SetCompare2)(TIM_TypeDef*,int),void(*SetCompare3)(TIM_TypeDef*,int),void(*SetCompare4)(TIM_TypeDef*,int),
		TIM_TypeDef* T1,TIM_TypeDef*T2,TIM_TypeDef*T3,TIM_TypeDef*T4,int i){
	SetCompare1(T1,i);
	SetCompare2(T2,i);
	SetCompare3(T3,i);
	SetCompare4(T4,i);
}

void LEDoff(int color,int j,int currentColor){
	switch(color){
	case 0:
		if(currentColor!=3 && currentColor!=4 && currentColor!=6)SetPWM(REDPWM);
		break;
	case 1:
		if(currentColor!=3 && currentColor!=5 && currentColor!=6)SetPWM(GREENPWM);
		break;
	case 2:
		if(currentColor!=4 && currentColor!=5 && currentColor!=6)SetPWM(BLUEPWM);
		break;
	case 3:
		if(currentColor!=0 && currentColor!=4 && currentColor!=6)SetPWM(REDPWM);
		if(currentColor!=1 && currentColor!=5 && currentColor!=6)SetPWM(GREENPWM);
		break;
	case 4:
		if(currentColor!=0 && currentColor!=4 && currentColor!=6)SetPWM(REDPWM);
		if(currentColor!=2 && currentColor!=5 && currentColor!=6)SetPWM(BLUEPWM);;
		break;
	case 5:
		if(currentColor!=1 && currentColor!=3 && currentColor!=6)SetPWM(GREENPWM);
		if(currentColor!=2 && currentColor!=4 && currentColor!=6)SetPWM(BLUEPWM);
		break;
	case 6:
		if(currentColor!=0 && currentColor!=3 && currentColor!=4)SetPWM(REDPWM);
		if(currentColor!=1 && currentColor!=3 && currentColor!=5)SetPWM(GREENPWM);
		if(currentColor!=2 && currentColor!=4 && currentColor!=5)SetPWM(BLUEPWM);
		break;
	default:
		break;
	}
}

void LEDon(int color,int j,int previousColor){
	switch(color){
	case 0:
		if(previousColor!=3 && previousColor!=4 && previousColor!=6)SetPWM(REDPWM);
		break;
	case 1:
		if(previousColor!=3 && previousColor!=5 && previousColor!=6)SetPWM(GREENPWM);
		break;
	case 2:
		if(previousColor!=4 && previousColor!=5 && previousColor!=6)SetPWM(BLUEPWM);
		break;
	case 3:
		if(previousColor!=1 && previousColor!=5 && previousColor!=6)SetPWM(GREENPWM);
		if(previousColor!=0 && previousColor!=4 && previousColor!=6)SetPWM(REDPWM);
		break;
	case 4:
		if(previousColor!=0 && previousColor!=3 && previousColor!=6)SetPWM(REDPWM);
		if(previousColor!=2 && previousColor!=5 && previousColor!=6)SetPWM(BLUEPWM);
		break;
	case 5:
		if(previousColor!=1 && previousColor!=3 && previousColor!=6)SetPWM(GREENPWM);
		if(previousColor!=2 && previousColor!=4 && previousColor!=6)SetPWM(BLUEPWM);
		break;
	case 6:
		if(previousColor!=0 && previousColor!=3 && previousColor!=4)SetPWM(REDPWM);
		if(previousColor!=1 && previousColor!=3 && previousColor!=5)SetPWM(GREENPWM);
		if(previousColor!=2 && previousColor!=4 && previousColor!=5)SetPWM(BLUEPWM);
		break;
	default:
		break;
	}
}

int prevColor=-1;
void LED(int colors[],int colors_count){
	  for(int color=0;color<colors_count;color++){
		if(colors[color]!=prevColor) {
			for (int i=ForCompare,j=0; i>=0; i--,j++) {
				LEDoff(prevColor,i,colors[color]);
				LEDon(colors[color],j,prevColor);
				delay_ms(1);
			}
			prevColor = colors[color];
		}
	  for (int i=0; i<ForCompare/2; i++) delay_ms(1);
 }
}

void Button_init(){
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	  __enable_irq();
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	  GPIO_InitTypeDef GPIO_InitStructure;
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10);

	  EXTI_InitTypeDef EXTI_InitStructure;
	  EXTI_InitStructure.EXTI_Line = EXTI_Line10;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);

	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	  NVIC_InitTypeDef NVIC_InitStructure;
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
}

void  EXTI15_10_IRQHandler()
{
  if (EXTI_GetITStatus(EXTI_Line10) == SET)
  {
	  srand(RTC_GetCounter());
	   for(int i = 0;i<colorsCount;i++){
		int n = rand()%7;
		standartColors[i] = n;
	  }
	delay_ms(100);
    EXTI_ClearITPendingBit(EXTI_Line10);
  }
}

void USB_init_func(){
	Set_System();
	Set_USBClock();
	USB_Interrupts_Config();
	USB_Init();
}

void processCommandUSB(char *cmd)
{
	if(strncmp(cmd, "red", 3) == 0){
		CDC_Send_DATA(":red\r",5);
		USB_colors[USB_colorsCount]=0;
		USB_colorsCount++;
	}

	else if(!strncmp(cmd, "green", 5)){
		CDC_Send_DATA(":green\r",7);
		USB_colors[USB_colorsCount]=1;
		USB_colorsCount++;
	}

	else if(!strncmp(cmd, "blue", 4)){
		CDC_Send_DATA(":blue\r",6);
		USB_colors[USB_colorsCount]=2;
		USB_colorsCount++;
	}

	else if(!strncmp(cmd, "yellow", 6)){
		CDC_Send_DATA(":yellow\r",8);
		USB_colors[USB_colorsCount]=3;
		USB_colorsCount++;
	}

	else if(!strncmp(cmd, "purple", 6)){
		CDC_Send_DATA(":purple\r",8);
		USB_colors[USB_colorsCount]=4;
		USB_colorsCount++;

	}

	else if(!strncmp(cmd, "cyan", 4)){
		CDC_Send_DATA(":cyan\r",6);
		USB_colors[USB_colorsCount]=5;
		USB_colorsCount++;
	}

	else if(!strncmp(cmd, "white", 5)){
		CDC_Send_DATA(":white\r",7);
		USB_colors[USB_colorsCount]=6;
		USB_colorsCount++;
	}

	else if(!strncmp(cmd, "ok", 2)){
		CDC_Send_DATA(":All colors accepted\r",21);
		colorsCount = USB_colorsCount;
		for(int i=0;i<USB_colorsCount;i++)
			standartColors[i]=USB_colors[i];
		USB_colorsCount = 0;
	}
	else if(!strncmp(cmd, "help", 4)){
		CDC_Send_DATA("\rEnter 1 color (red,green,blue,",31);
		delay_ms(50);
		CDC_Send_DATA("yellow,purple,cyan,white) ",27);
		delay_ms(50);
		CDC_Send_DATA("and ok to finish\r",17);
	}
	else{
		CDC_Send_DATA(":Wrong color\r",13);
	}
}

void USB_IRQhandler()
{
		 if (Receive_length  != 0){
			if(Receive_Buffer[0]=='\r')
			  {
				   processCommandUSB(usbReceived);
			       Receive_length = 0;
			       buff=0;
			  }
			  else{
			      CDC_Send_DATA ((uint8_t*)Receive_Buffer,Receive_length);
			      usbReceived[buff++] = Receive_Buffer[0];
			 }
			Receive_length=0;
		 }
}

void RTC_Init()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR |
	RCC_APB1Periph_BKP, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	RCC_LSICmd(ENABLE);
	while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY)== RESET);
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
	RCC_RTCCLKCmd(ENABLE);
	RTC_WaitForSynchro();
	RTC_WaitForLastTask();
	RTC_ITConfig(RTC_IT_SEC, ENABLE);
	RTC_WaitForLastTask();
	RTC_SetPrescaler(32767);
	RTC_WaitForLastTask();
}

void RTC_IRQHandler(void)
{
	if (RTC_GetITStatus(RTC_IT_SEC) != RESET){
		 RTC_ClearITPendingBit(RTC_IT_SEC);
		 RTC_WaitForLastTask();
	}
}

int main()
{
  delay_init();
  USART_init();
  PWM_init();
  Button_init();
  USB_init_func();
  RTC_Init();

  while(1)
  {
	 LED(standartColors,colorsCount);
	}
}
