// SPI functions for communicating with Attiny2313 microcontroller
// Peter Backlund 02.01.13

//#include <avr/iox16d4.h>
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

#include "lcd_driver.h"          //function prototypes defined here
#include "usart_driver.h"

#define NUM_LCD_CHARS 16 //number of chars that the LCD has in a line

//#define F_CPU 2000000L

// definition for precision of floats
# define PRECISION 3

#define CD  1
#define CR 250
#define HOME 0x80
//#define HOME2 0xC0

//#define CMD 0x00
//#define CLR 0x01
//#define HOME 0x02
#define HOME2 0xC0

#define USART USARTC0
 
void clear_display(void){
	while(!USART_IsTXDataRegisterEmpty(&USART));
//	USART_PutChar(&USART, CMD);
//	_delay_ms(1);
	USART_PutChar(&USART, CR);
	//USART_PutChar(&USART, CLR);
	_delay_us(50);   //obligatory waiting for slow LCD (1.64mS)
	USART_PutChar(&USART, CD);
	_delay_us(50);   //obligatory waiting for slow LCD (1.64mS)
}         

void cursor_home(void){
	while(!USART_IsTXDataRegisterEmpty(&USART));
	USART_PutChar(&USART, HOME);
	_delay_ms(2);   //obligatory waiting for slow LCD (1.64mS)
}         
  
void home_line2(void){
	while(!USART_IsTXDataRegisterEmpty(&USART));
	USART_PutChar(&USART, CR);
	_delay_ms(2);
	while(!USART_IsTXDataRegisterEmpty(&USART));
	USART_PutChar(&USART, HOME2);
	_delay_ms(2);   //obligatory waiting for slow LCD (1.64mS)
}                           
 
void char2lcd(char a_char){
	//sends a char to the LCD
	//usage: char2lcd('H');  // send an H to the LCD
	while(!USART_IsTXDataRegisterEmpty(&USART));
	USART_PutChar(&USART, a_char);
	_delay_us(100);   //obligatory waiting for slow LCD (40uS)
}
  
  
void string2lcd(char *lcd_str){
	//sends a string to LCD
	uint8_t count;
	for (count=0; count<=(strlen(lcd_str)-1); count++){
		while(!USART_IsTXDataRegisterEmpty(&USART));
		USART_PutChar(&USART, lcd_str[count]);
	  	_delay_us(100);   //obligatory waiting for slow LCD (40uS)
	}                  
} 

void lcd_init(void){
        //initalize the LCD to receive data
	uint8_t i;
	_delay_ms(15);   
	for(i=0; i<=2; i++){ //do funky initalize sequence 3 times
		while(!USART_IsTXDataRegisterEmpty(&USART));
		USART_PutChar(&USART, 0x30);
	  	_delay_ms(2);
	}

	while(!USART_IsTXDataRegisterEmpty(&USART));
	USART_PutChar(&USART, 0x38);
	_delay_ms(2);   

	while(!USART_IsTXDataRegisterEmpty(&USART));
	USART_PutChar(&USART, 0x08);
	_delay_ms(2);

	while(!USART_IsTXDataRegisterEmpty(&USART));
	USART_PutChar(&USART, CR);
	_delay_ms(2);   

	while(!USART_IsTXDataRegisterEmpty(&USART));
	USART_PutChar(&USART, 0x06);
	_delay_ms(2);

	while(!USART_IsTXDataRegisterEmpty(&USART));
	USART_PutChar(&USART, 0x0C);    // cursor off
	_delay_ms(2);
} 

void int2lcd(uint8_t val){
	char temp[10];
	itoa(val, temp, 10);
	string2lcd(temp);
}

void int162lcd(uint16_t val){
	char temp[10];
	utoa(val, temp, 10);
	string2lcd(temp);
}
void long2lcd(uint32_t val){
	char temp[10];
	ultoa(val, temp, 10);
	string2lcd(temp);
}
/**
 * Double to lcd
 */

void double2lcd(double num)
{
	int whole_part = num;
	int digit = 0, reminder =0;
	int log_value = log10(num), index = log_value;
	long wt =0;
	int i;

	// String containg result
	char str[20];
	//char* str = new char[20];

	//Initilise stirng to zero
	memset(str, 0 ,20);

	//Extract the whole part from float num
	for(i = 1 ; i < log_value + 2 ; i++)
	{
		wt  =  pow(10.0,i);
		reminder = whole_part  %  wt;
		digit = (reminder - digit) / (wt/10);

		//Store digit in string
		str[index--] = digit + 48;              // ASCII value of digit  = digit + 48
		if (index == -1)
			break;    
	}

	index = log_value + 1;
	str[index] = '.';

	double fraction_part  = num - whole_part;
	double tmp1 = fraction_part,  tmp =0;

	//Extract the fraction part from  num
	for( i= 1; i < PRECISION; i++)
	{
		wt =10; 
		tmp  = tmp1 * wt;
		digit = tmp;

		//Store digit in string
		str[++index] = digit +48;           // ASCII value of digit  = digit + 48
		tmp1 = tmp - digit;
	}    

	string2lcd(str);
	// return str;
}

void volt2lcd(uint16_t val){
	uint16_t upper, lower;
	char ups[10];
	char los[10];

	upper = val/1000;
	lower = val%1000;

	itoa(upper, ups, 10);
	itoa(lower, los, 10);

	string2lcd(ups);
	string2lcd(".");
	if(lower < 100){
		string2lcd("0");
	}
	string2lcd(los);

}
