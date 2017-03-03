/**
 * \file
 *
 * \brief Interrupt Service Routines (ISR)
 *
 */

/**
 * \mainpage ISR
 *
 * \par Interrupt Service Routines (ISR)
 *
 * The Interrupt Service Routines are global functions which are called
 * when enabled interrupts are happening. Each routine is connected to its
 * sub-module where the interrupt is originated.
 *
 * The ISR routine has always a part of code which needs to block any other
 * interrupts to happen until this critical section is done. Often the ISR
 * contains a second (bottom) part, that is non-critical to any new interruption
 * and the global interrupt enable is activated again before entering this
 * bottom part.
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Static and global helper functions for the ISR entry functions
 * -# Interrupt Service Routine vectors
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include "twi.h"

#include "isr.h"


/* External vars */

extern uint_fast32_t		g_timer_abs_msb;
extern uint8_t				g_adc_state;
extern float				g_adc_ldr;
extern float				g_adc_temp;


/* Forward declarations */

static void s_bad_interrupt(void);


/* Helper functions */

void asm_break(void)
{
	__asm__ __volatile__ ("break" ::: "memory");
	nop();
}


/* ISR routines */

static void s_bad_interrupt(void)
{
	asm_break();
}


ISR(__vector_1, ISR_BLOCK)  // variants: ISR_BLOCK, ISR_NOBLOCK, ISR_NAKED
{	/* INT0 */
	s_bad_interrupt();
}

ISR(__vector_2, ISR_BLOCK)
{	/* INT1 */
	s_bad_interrupt();
}

ISR(__vector_3, ISR_BLOCK)
{	/* PCINT0 */
	s_bad_interrupt();
}

ISR(__vector_4, ISR_BLOCK)
{	/* PCINT1 */
	s_bad_interrupt();
}

ISR(__vector_5, ISR_BLOCK)
{	/* PCINT2 */
	s_bad_interrupt();
}

ISR(__vector_6, ISR_BLOCK)
{	/* WDT - Watchdog Timeout */
	s_bad_interrupt();
}

ISR(__vector_7, ISR_BLOCK)
{	/* TIMER 2 COMP-A */
	s_bad_interrupt();
}

ISR(__vector_8, ISR_BLOCK)
{	/* TIMER 2 COMP-B */
	s_bad_interrupt();
}

ISR(__vector_9, ISR_BLOCK)
{	/* TIMER 2 OVF - Overflow */
	s_bad_interrupt();
}

ISR(__vector_10, ISR_BLOCK)
{	/* TIMER 1 CAPT */
	s_bad_interrupt();
}

ISR(__vector_11, ISR_BLOCK)
{	/* TIMER 1 COMP-A */
	s_bad_interrupt();
}

ISR(__vector_12, ISR_BLOCK)
{	/* TIMER 1 COMP-B */
	s_bad_interrupt();
}

ISR(__vector_13, ISR_BLOCK)
{	/* TIMER 1 OVF - Overflow */
	++g_timer_abs_msb;
}

ISR(__vector_14, ISR_BLOCK)
{	/* TIMER 0 COMP-A */
	s_bad_interrupt();
}

ISR(__vector_15, ISR_BLOCK)
{	/* TIMER 0 COMP-B */
	s_bad_interrupt();
}

ISR(__vector_16, ISR_BLOCK)
{	/* TIMER 0 OVF - Overflow */
	s_bad_interrupt();
}

ISR(__vector_17, ISR_BLOCK)
{	/* SPI, STC - Serial Transfer Complete */
	s_bad_interrupt();
}

ISR(__vector_18, ISR_BLOCK)
{	/* USART, RX - Complete */
	s_bad_interrupt();
}

ISR(__vector_19, ISR_BLOCK)
{	/* USART, UDRE - Data Register Empty */
	s_bad_interrupt();
}

ISR(__vector_20, ISR_BLOCK)
{	/* USART, TX - Complete */
	s_bad_interrupt();
}

ISR(__vector_21, ISR_BLOCK)
{	/* ADC */
	uint16_t adc_val;
	uint8_t  reason = g_adc_state;

	/* CLI part */
	adc_val  = ADCL;
	adc_val |= ADCH << 8;

	//TIFR1 |= _BV(TOV1);							// Reset Timer1 overflow status bit (no ISR for TOV1 activated!)

	switch (g_adc_state) {
		case ADC_STATE_PRE_LDR:
		// drop one ADC value after switching MUX
		g_adc_state = ADC_STATE_VLD_LDR;
		break;

		case ADC_STATE_VLD_LDR:
		adc_set_admux(ADC_MUX_TEMPSENSE | ADC_VREF_1V1 | ADC_ADJUSTMENT_RIGHT);
		g_adc_state = ADC_STATE_PRE_TEMP;
		break;

		case ADC_STATE_PRE_TEMP:
		// drop one ADC value after switching MUX
		g_adc_state = ADC_STATE_VLD_TEMP;
		break;

		case ADC_STATE_VLD_TEMP:
		// fall-through

		default:
		adc_set_admux(ADC_MUX_ADC0 | ADC_VREF_1V1 | ADC_ADJUSTMENT_RIGHT);
		g_adc_state = ADC_STATE_PRE_LDR;
	}

	uint16_t adc_ldr_last  = g_adc_ldr;
	uint16_t adc_temp_last = g_adc_temp;

	/* SEI part */
	sei();

	__vector_21__bottom(reason, adc_val, adc_ldr_last, adc_temp_last);
}

/* do not static this function to avoid code inlining that would inherit many push operations in the critical section */
void __vector_21__bottom(uint8_t reason, uint16_t adc_val, uint16_t adc_ldr_last, uint16_t adc_temp_last)
{
	/* Low pass filtering and enhancing the data depth */
	if (reason == ADC_STATE_VLD_LDR) {
		float calc = g_adc_ldr ?  0.90f * g_adc_ldr + 0.10f * adc_val : adc_val;			// load with initial value if none is set before

		cli();
		g_adc_ldr = calc;

	} else if (reason == ADC_STATE_VLD_TEMP) {
		float calc = g_adc_temp ?  0.9995f * g_adc_temp + 0.0005f * adc_val : adc_val;		// load with initial value if none is set before

		cli();
		g_adc_temp = calc;
	}
}

ISR(__vector_22, ISR_BLOCK)
{	/* EEREADY */
	s_bad_interrupt();
}

ISR(__vector_23, ISR_BLOCK)
{	/* ANALOG COMP */
	s_bad_interrupt();
}

ISR(__vector_24, ISR_BLOCK)
{	/* TWI */
	uint8_t tws = TWSR & (0b1111 << TWS4);
	uint8_t twd = TWDR;
	uint8_t twcr_cur = TWCR;

	uint8_t twcr_new = __vector_24__bottom(tws, twd, twcr_cur);
	TWCR = _BV(TWINT) | twcr_new;
}

ISR(__vector_25, ISR_BLOCK)
{	/* SPM READY - Store Program Memory Ready */
	s_bad_interrupt();
}
