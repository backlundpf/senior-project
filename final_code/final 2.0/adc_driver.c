/** Includes */
//#include <avr/boot.h>
//#include <avr/iox16d4.h>
#include <avr/io.h>

#include "adc_driver.h"


/** Functions */

/** The adc_init() function initializes the ADC by setting the calibration register
 *  and enabling the adc
 * **/
void adc_init(ADC_t *adc) {


	//string2lcd("success ");

	// Set port A pins 0-3 as inputs
	PORTA.DIRCLR = 0b00000011;
	// Set port B pin 0 as input
	PORTB.DIRCLR = 0x01;

	// Set a high current limit and freerun mode with 12 bit left adjusted results
	adc->CTRLB = (HIGH << CURRLIMIT);


	// Set the reference voltage to external reference AREF
	adc->REFCTRL = (INTVCC << REFSEL);
	
	// Figure out event controller stuff
	
	// Set prescaler to peripheral clock/8
	adc->PRESCALER = (DIV4);

	// Set the calibration bit
	//adc->CAL |= boot_signature_byte_get(0x02);
	//adc->CAL = ADCACAL0;

	// Set the channels up for gain of 2 and single ended input
	ADCA.CH0.CTRL = (0 << GAIN) | (SINGLEENDED << INPUTMODE);

	// Set up the interrupt
	//ADCA.CH0.INTCTRL = 0x03;

	// Enable that shit
	adc_enable(adc);

}

/** adc_enable() enables the adc. duh.
 */
void adc_enable(ADC_t *adc){
	adc->CTRLA |= ADC_ENABLE;
}

/**
 * Start one-shot conversion on ADC channel(s)
 *
 * ch_mask Mask of ADC channel(s):
 * \arg \c ADC_CHn , where \c n specifies the channel. (These can be OR'ed
 * together.)
 *
 * The ADC must be enabled for this function to have any effect.
 */
void adc_start_conversion(ADC_t *adc, uint8_t ch)
{
//	char temp[16];
//
//	//itoa(adc->INTFLAGS, temp, 2);
//	//itoa(ch, temp, 2);
//	itoa(ADCA.CH0.INTFLAGS, temp, 2);
//	string2lcd(temp);

	ADCA.CH0.MUXCTRL |= (ch << 3);
	ADCA.CH0.CTRL |= ADC_START;
	adc->CTRLA |= ADC_START;

}

/**
 * Get result from adc conversion
 */
uint16_t adc_get_result(ADC_t *adc){
	uint16_t reg = 0;
	uint16_t rval = 0;
	uint32_t temp = 0;

	// Read lower 8 bits first
	reg = ADCA.CH0RESL;

	// Read higher 8 bits
	reg |= (ADCA.CH0RESH & 0x0F) << 8;

	/* Convert to get the actual voltage
	 * Vin = (RES*VREF/TOP) - dV */
	temp = reg;
	temp *= VREF;
	temp /= VTOP;
	temp -= VdV;

	rval = temp;
	return rval;
}
