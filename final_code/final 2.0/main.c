/* main.c
 * 2/28/13
 *
 * Peter Backlund
 *
 * edited from spi_polled_example.c and usart_example_polled.c from the atmel
 * application notes.
 *
 *  Hardware setup:
 *
 *    - PC2 (RXD0)
 *    - PC3 (TXD0) 
 *    - PC4 PD4 (SS)
 *    - PC5 PD5 (MOSI)
 *    - PC6 PD6 (MISO)
 *    - PC7 PD7 (SCK)
 *
 */
#include <avr/io.h>
//#include <avr/interrupt.h>
#include <avr/common.h>
//#include <avr/iox16d4.h>
#include <math.h>

#include "avr_compiler.h"
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "spi_driver.h"
#include "usart_driver.h"
#include "lcd_driver.h"
#include "tc_driver.h"
#include "adc_driver.h"
//#include "afunc.h"

//#define CR 250
//#define CD 1
//#define L1 0X80
//#define L2 0XC0


/* Definition of the CPU clock speed and TC prescaler setting. */
#define CPU_PRESCALER   1

/*! \brief The number of test data bytes. */
#define NUM_BYTES     4

/*! Define that selects the Usart used in example. */
#define USART USARTC0

#define PWM_TOP 0x00C8

/* Global variables */

/*! \brief SPI master module on PORT C. */
SPI_Master_t spiMasterC;

/*! \brief SPI slave module on PORT D. */
SPI_Slave_t spiSlaveD;

/*! \brief SPI Data packet */
SPI_DataPacket_t dataPacket;


/*! \brief Result of the test. */
bool success = true;

static uint32_t samplecount = 0;
/*! \brief Compare value for PWM signal on PE1. */
volatile uint16_t buckFreq = 0x0064;
static uint8_t buckInk = 20;

/* PWM frequency */
volatile uint8_t curLim = 5;

/*! \brief holds the previous value read from the encoder. */
volatile uint8_t prev_change = 0;
volatile uint8_t enc_val;

const char *disp_mode[] = {"Damping", "Current Power", "Total Energy", "Output Voltage", "Acceleration", "samples"};
const char *disp_units[] = {" %", " W", " Joules", " Volts", " g's", " s"};
volatile uint16_t disp_val[6] = {10, 20, 0, 40, 50, 0};
volatile uint8_t disp_idx = 0; // mod this by 3 to maintain the display mode

/* Put converterted values here. */
/* current out, voltage out, accelerometer, */
volatile uint16_t adc_samples[3][5];
volatile uint16_t adc_weighted[3];
/* adc_ratio holds the ratios of the voltage divider
 * divided by 4 for quarter second interrupts */
static int16_t adc_ratio[3] = 	{12, //current out:	1V/50mV
				500, // voltage out:	1V/20V
			       	125}; // Accelerometer:	1V/5V
//volatile uint16_t weighted_numerators[3];
volatile uint8_t adc_idx = 0;

/*! \brief declare function prototypes. */
void USART_writes(const char* myval);
void TC0_init(void);
void TC1_init(void);
void readEncoders(void);
void update_display(void);
void update_display_top(void);
void update_display_bottom(void);
uint16_t av_calc(uint16_t new_point);

int main(void)
{
	/* Enable global interrupts. */
	sei();

	// Initialize 
	TC0_init();
	TC1_init();

	
	// add variables for getting/decoding encoder
//	static int8_t enc_val = 0;

	/* Set up the UART stuff first */

	/* This PORT setting is only valid to USARTC0 if other USARTs is used a
	 * different PORT and/or pins is used. */
	/* PIN3 (TXD0) as output. */
	PORTC.DIRSET = PIN3_bm;

	/* PC2 (RXD0) as input. */
	PORTC.DIRCLR = PIN2_bm;

	/* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(&USART, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);

	/* Set Baudrate to 9600 bps:
	 * Use the default I/O clock fequency that is 2 MHz.
	 * Do not use the baudrate scale factor
	 *
	 * Baudrate select = (1/(16*(((I/O clock frequency)/Baudrate)-1)
	 *                 = 12
	 */
	USART_Baudrate_Set(&USART, 12 , 0);

	/* Enable both RX and TX. */
	USART_Rx_Enable(&USART);
	USART_Tx_Enable(&USART);

	/* Now set up the SPI stuff */

	/* Init SS pin as output with wired AND and pull-up. */
	PORTC.DIRSET = PIN4_bm;
	PORTC.PIN4CTRL = PORT_OPC_WIREDANDPULL_gc;

	/* Set SS output to high. (No slave addressed). */
	PORTC.OUTSET = PIN4_bm;

	/* Initialize SPI master on port C. */
	SPI_MasterInit(&spiMasterC,
	               &SPIC,
	               &PORTC,
	               false,
	               SPI_MODE_0_gc,
	               SPI_INTLVL_OFF_gc,
	               false,
	               SPI_PRESCALER_DIV4_gc);

	// initialize the display
	//lcd_init();
	clear_display();
	update_display();

	adc_init(&ADCA);

	PORTD.DIRSET = PIN0_bm;
	PORTD.OUTSET = PIN0_bm;

	while(1){
		/* Transmit the UART stuff first */

		if(enc_val){
			clear_display();			
			switch(enc_val){
				case 1: if(disp_idx == 0){
						disp_idx = 5;
					}
					else {
						disp_idx--; 
					}
					//disp_idx %= 5;
					update_display();
					break;
				case 2: disp_idx++;  
					disp_idx %= 6;
					update_display();
					break;
				case 3: if(disp_idx){
						if(buckFreq != 0){
							buckFreq -= buckInk;
							buckFreq %= PWM_TOP;
							TC_SetCompareA( &TCE0, buckFreq );
							int2lcd(100-buckFreq/2);
						}
					} 
					else {
						if(curLim != 0){
							curLim--;
						}
						update_display();
					}	
					break;
				case 4: if(disp_idx){
						if(buckFreq == PWM_TOP - buckInk){
							buckFreq = PWM_TOP;
							TC_SetCompareA( &TCE0, buckFreq );
						}
						else if(buckFreq < PWM_TOP){
							buckFreq += buckInk;
							TC_SetCompareA( &TCE0, buckFreq );
							int2lcd(100-buckFreq/2);
						}
					}
					else{
						if(curLim != 10){
							curLim++;
						}
						update_display();
					}	
					break;
			}
			enc_val = 0;
		}

	}
}

/****************************************************************/
//	Put initialization routines here
/****************************************************************/

/* 			TC0_init()
 *  			
 * Set up TC0 for PWM output to the current limiter*/
void TC0_init(){
	/* Enable output on PE0. */
	PORTE.DIR = 0x01;

	/* Set the TC period. */
	TC_SetPeriod( &TCE0, PWM_TOP);

	/* Configure the TC for single slope mode. */
	TC0_ConfigWGM( &TCE0, TC_WGMODE_SS_gc );

	/* Enable Compare channel A. */
	TC0_EnableCCChannels( &TCE0, TC0_CCAEN_bm );

	/* Start timer by selecting a clock source. */
	TC0_ConfigClockSource( &TCE0, TC_CLKSEL_DIV1_gc );

	/* Output new compare value. */
	TC_SetCompareA( &TCE0, buckFreq );

}

/* 			TC1_init()
 *  			
 * Set up TC1 to trigger an interrupt every 4ms. */
void TC1_init(){
	/* Set period ( TOP value ) for 1ms intervals. 
	 * 250 KHz * 0xFA = 1 ms intervals. */
	TC_SetPeriod( &TCC1, 0xFA );

	/* Enable overflow interrupt at low level. */
	TC1_SetOverflowIntLevel( &TCC1, TC_OVFINTLVL_LO_gc );
	PMIC.CTRL |= PMIC_LOLVLEN_bm;


	/* Start Timer/Counter by selecting a clock source. 
	 * Use clock/8 to get a frequency of 250 KHz. */
	TC1_ConfigClockSource( &TCC1, TC_CLKSEL_DIV8_gc );
}

/****************************************************************/
//	Put functions here
/****************************************************************/

/*			readEncoders()
 * Read the data from the encoders, if an encoder has made a full
 * rotation return a non-zero
 */
void readEncoders(void){

	static uint8_t enc_dec[] = {0b00000001, 0b00000100, 0b00000010, 0b00001000};

	static uint8_t change;
	static uint8_t nchange;
	static uint8_t prev_chng;
	static int8_t dir;
	//int8_t CNT = 0;

	PORT_t *ssPort = &PORTC;
	/* Now transmit via SPI */

	/* PHASE 1: Transceive individual bytes. */
	
	/* MASTER: Pull SS line high. This has to be done since
	 *         SPI_MasterTransceiveByte() does not control the SS line(s). */
	SPI_MasterSSHigh(ssPort, PIN4_bm);
	
	// read encoders

	//transmit out byte
	change = SPI_MasterTransceiveByte(&spiMasterC, 0xFF);
	
	//_delay_ms(5);
	if(change != prev_chng){ //if change has changed let's see what happened
		if(change == 0xFF ){   //if change is back to being all 1's
			enc_val = dir;  //increment or decrement the count
			dir = 0;		 //reset the direction variable
			prev_chng = change;
		}//if
		else {
			nchange = ~change; 	 //get the inverse of change, we'll be using it alot
			//the last state change is in before it goes back to
			// 0xFF can tell us which way the dial was turned
			if(nchange & enc_dec[0] ){//ccw turn of left dial
				dir = 3;
				//dir = 1; //count should be decremented
			}
			else if(nchange & enc_dec[1]){//ccw turn of right dial
				dir = 1;
				//dir = 3; //count should be decremented
			}
			else if(nchange & enc_dec[2]){//cw turn of left dial
				dir = 4;
				//dir = 2; //count should be incremented
			}
			else if(nchange & enc_dec[3]){//cw turn of right dial
				dir = 2;
				//dir = 4; //count should be incremented
			}
		}
		prev_chng = change; //set the prev_change so we don't have to go through this 
	}			  //every time.
	//prev_chng = change;


	SPI_MasterSSLow(ssPort, PIN4_bm);

	//return 0;

}

/*************************************************************************/
//				update_display	
//If this function is called we must have updated one of the variables to 
//be displayed, clear the display and rewrite.
// 16x2 lcd of form:
// Display Mode:
// Current Values
/*************************************************************************/
void update_display(void){
	clear_display();
	update_display_top();
	update_display_bottom();
}

void update_display_top(void){

	cursor_home();
	string2lcd(disp_mode[disp_idx]);
}

void update_display_bottom(void){

	char *s;
	int i;
	//uint16_t t_val = disp_val[disp_idx];

	home_line2();

	if(!disp_idx){
		//i = buckFreq/PWM_TOP;
		i = curLim*10;
		itoa(i, s, 10);
		string2lcd(s);
	}
//	else if(disp_idx == 5){
//		int162lcd(disp_val[disp_idx]);
//	}
	else{
		//double2lcd(disp_val[disp_idx]);
		volt2lcd(disp_val[disp_idx]);
		
		//if(disp_idx == 3){
		//	long2lcd(samplecount);

		//}
	}
	string2lcd(disp_units[disp_idx]);
}

/*************************************************************************/
//				av_calc()
// Calculate the cumulative moving average to a new datapoint.
/*************************************************************************/
uint16_t av_calc(uint16_t new_point){

	static uint16_t num_points = 5;
	uint16_t new_avg = 0;
	uint8_t i;

//	if(num_points < 50){
//		num_points++;
//	}

	for(i=num_points-1; i>=1; i--){
		adc_samples[adc_idx][i] = adc_samples[adc_idx][i-1];
		new_avg += adc_samples[adc_idx][i];
	}
	adc_samples[adc_idx][0] = new_point;

	new_avg += new_point;

	new_avg /= num_points;

	return new_avg;
}


/****************************************************************/
//	Put interrupt service routines here
/****************************************************************/

/* It's been 1ms, scan the next adc. */
ISR(TCC1_OVF_vect){
	
	static uint8_t ms = 0;
	static uint8_t qs = 0;
	uint32_t rval = 0;
	static uint16_t cnt = 0;
	uint16_t val;
//	static uint8_t pflag;
	static uint8_t pcnt;
	
	readEncoders();
	
	// toggle PC1 for an 80% duty cycle
	if((ms%10 == 0) && curLim){
		PORTD.OUTSET = PIN0_bm;
//		pflag = 1;
		pcnt = 0;
		//PORTC.DIRCLR = PIN1_bm;
	}
	else if(pcnt == curLim){
		PORTD.OUTCLR = PIN0_bm;

	}
	pcnt++;
	//pcnt %= 10;
//	else if((ms%curLim == 0) && pflag){
//		PORTD.OUTCLR = PIN0_bm;
//		//PORTC.DIRSET = PIN1_bm;
//		pflag = 0;
//	}

	// every ~250 ms, update display
	if(ms%250 == 0){
		ms = 0;
		qs++;
		//		update_display_bottom();

		if(qs%4 == 0){
			/* It's been a quarter second, time to 
			 * update the display values */
			
			/* Set the current output voltage */
			val = adc_weighted[0]*20; 		// get voltage
			disp_val[3] = val;
			//if(val >= 500){
			//	disp_val[3] = val;
			//}
			//else{
			//	disp_val[3] = 0;
			//}

			/* Update power out */
			//val *= adc_weighted[0];		// multiply by current
			//disp_val[1] = val;		// store in display 1
			disp_val[1] = 0;		// store in display 1

			/* Update energy out */
			//disp_val[2] += val/1000;
			disp_val[2] += 1;

			/* Set current acceleration. */
			//disp_val[4] = adc_weighted[2];
			disp_val[4] = adc_weighted[1]*5;

			/* If display's not set on damping
			 * update. */
			//if(disp_idx == 3){
			//if(disp_idx && (disp_idx != 5)){
			if(disp_idx ){
				update_display();
			}

			//adc_idx++;
			//adc_idx %= 2;
			
		//	disp_val[5]++;
		}

		rval = adc_get_result(&ADCA)/10;
		adc_weighted[adc_idx] = av_calc(rval);
		
		cnt++;

		//samplecount++;
		adc_start_conversion(&ADCA, adc_idx);
	}
	ms++;

	// Start the next conversion
}
