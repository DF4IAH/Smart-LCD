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

enum TWI_FSM_STATUS {
	TWI_FSM_STATUS_BLOCKING_BM = _BV(0),
};


/* Helper functions */
void asm_break(void);


#endif /* ISR_H_ */
