/* Header file for ADC_driver
 *
 */

#define HIGH 		0b11
#define CURRLIMIT 	5

#define INT1V		0b000
#define INTVCC		0b001
#define AREFB		0b011
#define REFSEL		4

#define DIV4		0
#define DIV8		0b001

#define GAIN 		2
#define SINGLEENDED	0b01
#define INPUTMODE	0

#define ADC_ENABLE	1

#define ADC_START	0b10000000


#define VREF		20625
#define VTOP		4096
#define VdV		1031

//#define VREF		1000
//#define VTOP		4096
//#define VdV		50

void adc_init(ADC_t *adc);
void adc_start_conversion(ADC_t *adc, uint8_t count);
void adc_enable(ADC_t *adc);
uint16_t adc_get_result(ADC_t *adc);
