/*
 * isr.h
 *
 * Created: 13.02.2017 08:07:19
 *  Author: DF4IAH
 */ 


#ifndef ISR_H_
#define ISR_H_


enum ADC_STATES {
	ADC_STATE_PRE_LDR = 0,
	ADC_STATE_VLD_LDR,
	ADC_STATE_PRE_TEMP,
	ADC_STATE_VLD_TEMP,

	ADC_STATE__COUNT
};


/* Helper functions */
void asm_break(void);


/* ISR */
void __vector_21__bottom(uint8_t reason, uint16_t adc_val);


#endif /* ISR_H_ */
