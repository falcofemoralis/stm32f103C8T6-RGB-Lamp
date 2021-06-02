//------------------------------------------------------
/* File:       Library for HD44780 compatible displays  */
/* Version:	   v1.1  						 			*/
/* Language	   ANSI C			   		  	 			*/
/* Author:     GrAnd/www.MakeSystem.net		 			*/
/* Tested on:  AVR		  			 	 	 		 	*/
/* License:	   GNU LGPLv2.1		 		 	 			*/
//------------------------------------------------------
/* Copyright (C)2012 GrAnd. All right reserved 			*/
//------------------------------------------------------

/*
  [hd44780.h - Library for HD44780 compatible displays]
  [Copyright (C)2012 GrAnd. All right reserved]

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Lesser General Public
	License as published by the Free Software Foundation; either
	version 2.1 of the License, or (at your option) any later version.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
	Lesser General Public License for more details.

	You should have received a copy of the GNU Lesser General Public
	License along with this library; if not, write to the Free Software
	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

Contact information :
						mail@makesystem.net
						http://makesystem.net/?page_id=2
*/

#include "hd44780.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#if ( USE_PROGRESS_BAR )
static int8u_t progress_bar[NUMBER_OF_CELL_ELEMENTS] = {0x00,0x10,0x18,0x1C,0x1E,0x1F};
static int8u_t current_bar_load;
#endif

//-------------------------------
// LOW LEVEL FUNCTIONS
//-------------------------------
//static void DELAY(int16u_t ms);
static void LCD_STROBE(int16u_t loop);
static void HIGHBITS(int8u_t data);
static void LOWBITS(int8u_t data);

//volatile int16u_t MCU_CLK_VALUE,MCU_CLK;

//-------------------------------
/* DELAY FUNCTION */
//-------------------------------
void DELAY( int32u_t ms)
{
  volatile int16u_t beta;
  ms=10*(ms+1);
  volatile int32u_t alfa;
 for(alfa=0;alfa<ms;alfa++)
  for(beta=0;beta<MCU_CLK;beta++){__NOP();}
  ;
}

const  char Decode2Rus[255-192+1]= {
0x41,0xA0,0x42,0xA1,0xE0,0x45,0xA3,0xA4,
0xA5,0xA6,0x4B,0xA7,0x4D,0x48,0x4F,0xA8,
0x50,0x43,0x54,0xA9,0xAA,0x58,0xE1,0xAB,
0xAC,0xE2,0xAD,0xAE,0xAD,0xAF,0xB0,0xB1,
0x61,0xB2,0xB3,0xB4,0xE3,0x65,0xB6,0xB7,
0xB8,0xB9,0xBA,0xBB,0xBC,0xBD,0x6F,0xBE,
0x70,0x63,0xBF,0x79,0xE4,0x78,0xE5,0xC0,
0xC1,0xE6,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7 };


//-------------------------------
/* INITIATE TRANSFER OF DATA/COMMAND TO LCD */
//-------------------------------
static void LCD_STROBE(int16u_t loop)
{
 ENABLE(LCD_WIRE_E,E);
 DELAY(MCU_WAIT_CYCLES);
 DISABLE(LCD_WIRE_E,E); // Enter
 DELAY(loop);
}

//-------------------------------
/* PUT HIGH BITS */
//-------------------------------
static void HIGHBITS(int8u_t data)
{
 if(data & 0x80) ENABLE(LCD_WIRE_D7,D7); else DISABLE(LCD_WIRE_D7,D7);
 if(data & 0x40) ENABLE(LCD_WIRE_D6,D6); else DISABLE(LCD_WIRE_D6,D6);
 if(data & 0x20) ENABLE(LCD_WIRE_D5,D5); else DISABLE(LCD_WIRE_D5,D5);
 if(data & 0x10) ENABLE(LCD_WIRE_D4,D4); else DISABLE(LCD_WIRE_D4,D4);
}

//-------------------------------
/* PUT LOW BITS */
//-------------------------------
static void LOWBITS(int8u_t data)
{
 if(data & 0x08) ENABLE(LCD_WIRE_D7,D7); else DISABLE(LCD_WIRE_D7,D7);
 if(data & 0x04) ENABLE(LCD_WIRE_D6,D6); else DISABLE(LCD_WIRE_D6,D6);
 if(data & 0x02) ENABLE(LCD_WIRE_D5,D5); else DISABLE(LCD_WIRE_D5,D5);
 if(data & 0x01) ENABLE(LCD_WIRE_D4,D4); else DISABLE(LCD_WIRE_D4,D4);
}


//-------------------------------
/* PUT DATA/COMMAND TO LCD */
//-------------------------------
void lcd_cmd(int8u_t data, int16u_t loop)
{/* LCD ELEMENTARY COMMAND */
 HIGHBITS(data);
 LCD_STROBE(0);
 LOWBITS(data);
 LCD_STROBE(loop); // busy delay
// DELAY(200);
}

 				   	  	   	   //-------------------------------
							   /*         LCDlib API          */
							   //-------------------------------

//-------------------------------
/* LCD CLEAR SCREEN */
//-------------------------------
void lcd_clrscr(void)
{
 lcd_cmd(0x01,2); // clear screen
}

//-------------------------------
/* LCD RETURN CURSOR */
//-------------------------------
void lcd_return(void)
{
 lcd_cmd(0x02,2); // return cursor
}

//-------------------------------
/* GO TO SPECIFIED MEMORY ADDRESS */
//-------------------------------
void lcd_goto(int8u_t line, int8u_t address)
{/* GO TO SPECIFIED ADDRESS */
 switch(line)
 {
  case     1: lcd_cmd(0x80|address,0); break;
  case     2: lcd_cmd(0xC0|address,0); break;
  case CGRAM: lcd_cmd(0x40|address,0); break; // CGRAM address
 }
}

//-------------------------------
/* WRITE ENTIRE STRING 
   TO SPECIFIED MEMORY */
//-------------------------------
void lcd_prints(const char *p)
{/* WRITE A STRING TO LCD */
 ENABLE(LCD_WIRE_RS,RS);
  while(*p)
  {
#if ( USE_FORMATTED_OUTPUT )
//-------------------------------
// new line
//-------------------------------
   if((*p == '\n'))
   {
	DISABLE(LCD_WIRE_RS,RS);
	lcd_goto(2,0);
	ENABLE(LCD_WIRE_RS,RS);
	p++;
   }
//-------------------------------
// return
//-------------------------------
   else if((*p == '\r'))
   {
	DISABLE(LCD_WIRE_RS,RS);
	lcd_return();
	ENABLE(LCD_WIRE_RS,RS);
	p++;
   }
//-------------------------------
// tab
//-------------------------------
   else if((*p == '\t'))
   {
	DISABLE(LCD_WIRE_RS,RS);
	switch(TAB_SPACE)
    {
	 case 8: lcd_cmd(0x14,0); // cursor right shift
     	  	 lcd_cmd(0x14,0); // cursor right shift
     		 lcd_cmd(0x14,0); // cursor right shift
     		 lcd_cmd(0x14,0); // cursor right shift
     case 4: lcd_cmd(0x14,0); // cursor right shift
     	  	 lcd_cmd(0x14,0); // cursor right shift
     case 2: lcd_cmd(0x14,0); // cursor right shift
     case 1: lcd_cmd(0x14,0); // cursor right shift
    }
	ENABLE(LCD_WIRE_RS,RS);
	p++;
   }
//-------------------------------
// display
//-------------------------------
   else
#endif
 if(*p>=192)lcd_cmd(Decode2Rus[(*p++)-192],0);
 else lcd_cmd(*p++,0);
 }
 DISABLE(LCD_WIRE_RS,RS);
}

//-------------------------------
/* WRITE A SINGLE CHARACTER 
   TO SPECIFIED MEMORY */
//-------------------------------
void lcd_putc(int8u_t data)
{/* WRITE A CHARACTER TO LCD */
 ENABLE(LCD_WIRE_RS,RS);
 lcd_cmd(data,0);
 DISABLE(LCD_WIRE_RS,RS);
}

//-------------------------------
/* LOAD USER-DEFINED CHARACTER IN CGRAM */
//-------------------------------
void lcd_load(int8u_t* vector, int8u_t position)
{/* USE CGRAM CHAR SPACE: 0 to 7 */
 int8u_t i;
 lcd_goto(CGRAM,position*DRAW_CHAR_SIZE);
 for(i=0;i<DRAW_CHAR_SIZE;i++)
  lcd_putc(vector[i]);
}

//-------------------------------
/* DISPLAY USER-DEFINED CHARACTER ON DDRAM */
//-------------------------------
void lcd_drawchar( int8u_t* vector, 
	 			   int8u_t position, 
	 			   int8u_t line, 
				   int8u_t address )
{/* USE CGRAM CHAR SPACE */
 lcd_load(vector,position);
 lcd_goto(line,address);
 lcd_putc(position);
}

//-------------------------------
/* ERASE A SINGLE CHARACTER 
   FROM DISPLAY */
//-------------------------------
void lcd_backspace(void)
{/* ERASE LEFT CHAR */
 lcd_cmd(0x10,0); // �������� ������ �� ���� ������� �����
 lcd_putc(' '); // �������, ����� ���� ���������� ������������� ������
 lcd_cmd(0x10,0); // �������� ������ �� ���� ������� �����
}

//-------------------------------
/* SCROLL DISPLAY 
   TO SPECIFIED DIRECTION */
//-------------------------------
void lcd_scroll(int8u_t direction)
{
 switch(direction)
 {
  case RIGHT : lcd_cmd(0x1C,0); break; // scroll display to right
  case LEFT  : lcd_cmd(0x18,0); break; // scroll display to left
 }
}

//-------------------------------
/* SHIFT CURSOR 
   TO SPECIFIED DIRECTION */
//-------------------------------
void cursor_shift(int8u_t direction)
{
 switch(direction)
 {
  case RIGHT : lcd_cmd(0x14,0); break; // shift cursor to right
  case LEFT  : lcd_cmd(0x10,0); break; // shift cursor to left
 }
}

//-------------------------------
/* DISPLAY A INTEGER NUMER */
//-------------------------------
void lcd_itostr(int32s_t value)
{/* DISPLAY A INTEGER NUMER: +/- 2147483647 */
 int32s_t i;
 if(value<0)
 {
  lcd_putc('-');
  value=-value;
 }
 for(i=1;(value/i)>9;i*=10);
 lcd_putc(value/i+'0');
 i/=10;
 while(i)
 {
  lcd_putc((value%(i*10))/i+'0');
  i/=10;
 }
}

//-------------------------------
/* DISPLAY A 4-DIGIT INTEGER NUMER */
//-------------------------------
void lcd_numTOstr(int16u_t value, int8u_t nDigit)
{/* DISPLAY n-DIGIT INTEGER NUMER */
 switch(nDigit)
 {
  case 4: lcd_putc((value/1000)+'0');
  case 3: lcd_putc(((value/100)%10)+'0');
  case 2: lcd_putc(((value/10)%10)+'0');
  case 1: lcd_putc((value%10)+'0');
 }
}

#if ( USE_PROGRESS_BAR )
//-------------------------------
/* PRELOAD PROGRESS BAR ELEMENTS IN CGRAM */
//-------------------------------
void lcd_readybar(void)
{
 int8u_t i,j;
 for(i=0;i<NUMBER_OF_CELL_ELEMENTS;i++)
 {
  lcd_goto(CGRAM,(i*DRAW_CHAR_SIZE));
  for(j=0;j<PROGRESS_BAR_HEIGHT;j++)
   lcd_putc(progress_bar[i]);
 }
 lcd_goto(1,0);
}

//-------------------------------
/* DRAW PROGRESS BAR ON DDRAM */
//-------------------------------
void lcd_drawbar(int8u_t next_bar_load)
{
 int8u_t i = current_bar_load;
 int8u_t cell = (current_bar_load/FULL_ELEMENT); // find current cell position in progress bar
 if(next_bar_load > NUMBER_OF_BAR_ELEMENTS ) next_bar_load = NUMBER_OF_BAR_ELEMENTS;
 if( next_bar_load > current_bar_load )
 {
//-------------------------------
// increment progress bar code //
//-------------------------------
  lcd_goto(DRAW_PROGRESS_BAR_ON_LINE, cell); // goto current cell position
  while( i != next_bar_load )
  {
   i++;
   if( CELL_RATIO(i) == 0 ) lcd_putc( FULL_ELEMENT );
    else lcd_putc( CELL_RATIO(i) );
   if( CELL_RATIO(i) ) cursor_shift(LEFT);
  }
 }
 else
 {
//-------------------------------
// decrement progress bar code //
//-------------------------------
  if(CELL_RATIO(i) == 0) cell--;
  lcd_goto(DRAW_PROGRESS_BAR_ON_LINE, cell); // goto current cell position
  lcd_cmd(ENTRY_MODE_DEC_NO_SHIFT,0); // decrement lcd cursor
  while( i != next_bar_load )
  {
   i--;
   lcd_putc( CELL_RATIO(i) );
   if( CELL_RATIO(i) ) cursor_shift(RIGHT);
  }
  lcd_cmd(ENTRY_MODE_INC_NO_SHIFT,0); // increment lcd cursor
 }
//-------------------------------
//       store new value       //
//-------------------------------
 current_bar_load = next_bar_load;
}

//-------------------------------
/*  CLEAR ENTYRE PROGRESS BAR  */
//-------------------------------
void lcd_clearbar(void)
{
 int8u_t i;
 lcd_goto(DRAW_PROGRESS_BAR_ON_LINE, (PROGRESS_BAR_WIDTH - 1));
 lcd_cmd(ENTRY_MODE_DEC_NO_SHIFT,0); // decrement
 for(i=0;i<PROGRESS_BAR_WIDTH;i++)
  lcd_putc(' ');
 lcd_cmd(ENTRY_MODE_INC_NO_SHIFT,0); // increment
 current_bar_load = 0;
}
#endif

//-------------------------------
/* CONFIGURE 4-BIT DISPLAY INTERFACE */
//-------------------------------
void lcd_config(int8u_t param)
{/* CONFIGURE THE DISPLAY */
 HIGHBITS(param); // 4-bit, two lines, 5x8 pixel
  LCD_STROBE(4000); // change 8-bit interface to 4-bit interface
  LCD_STROBE(1000); // init 4-bit interface
 LOWBITS(param);
  LCD_STROBE(1000);
}

//-------------------------------
/* INITIALIZE ENTIRE DISPLAY */
//-------------------------------
 void InitGPIO(GPIO_TypeDef* GPIOx, u8 GPIO_Pin)
  {
  GPIO_InitTypeDef GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin =(u16) 1<<GPIO_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 // GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOx,&GPIO_InitStructure);
  }

 void EnablePort(GPIO_TypeDef* GPIOx)
  {
  uint32_t RCC_AHB1Periph=0;
  
  if(GPIOx==GPIOA)RCC_AHB1Periph |=RCC_APB2Periph_GPIOA;
  if(GPIOx==GPIOB)RCC_AHB1Periph |=RCC_APB2Periph_GPIOB;
  if(GPIOx==GPIOC)RCC_AHB1Periph |=RCC_APB2Periph_GPIOC;
  if(GPIOx==GPIOD)RCC_AHB1Periph |=RCC_APB2Periph_GPIOD;
  if(GPIOx==GPIOE)RCC_AHB1Periph |=RCC_APB2Periph_GPIOE;
//  if(GPIOx==GPIOH)RCC_AHB1Periph |=RCC_APB2Periph_GPIOH;
  
  RCC_APB2PeriphClockCmd(RCC_AHB1Periph, ENABLE);
  }

void lcd_init(void)
{  

  EnablePort(LCD_WIRE_E);
 EnablePort(LCD_WIRE_RS);
 EnablePort(LCD_WIRE_D4);
 EnablePort(LCD_WIRE_D5);
 EnablePort(LCD_WIRE_D6);
 EnablePort(LCD_WIRE_D7);
   
    InitGPIO(LCD_WIRE_E,E);  
    InitGPIO(LCD_WIRE_RS,RS);
    InitGPIO(LCD_WIRE_D4,D4);
    InitGPIO(LCD_WIRE_D5,D5);
    InitGPIO(LCD_WIRE_D6,D6);
    InitGPIO(LCD_WIRE_D7,D7);
     
    //DELAY(1000);

    RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
 // MCU_CLK_VALUE=RCC_Clocks.SYSCLK_Frequency / 1000000;
 // MCU_CLK=((MCU_CLK_VALUE - 1)/4);//MCU_CLK_VALUE;

  //DELAY(20000);
 // for(int i=0;i<2;i++)
  { lcd_config(DEFAULT_DISPLAY_CONFIG); // 1, Data Lenght, Number of lines, character font
 //DELAY(10000);
 lcd_cmd(DEFAULT_DISPLAY_CONTROL,0); // 1, lcd, cursor, blink
 //DELAY(1000);
 lcd_cmd(DEFAULT_ENTRY_MODE,0); // 1,increment/decrement,display shift on/off
// DELAY(1000);
 lcd_cmd(0x01,2); // clear display
 //DELAY(1000);
 lcd_cmd(0x02,2); // 1, return home cursor
// DELAY(1000);
#if (USE_PROGRESS_BAR)
 lcd_readybar();
#endif
    }
}

//-------------------------------
/* END OF FILE */
//-------------------------------
