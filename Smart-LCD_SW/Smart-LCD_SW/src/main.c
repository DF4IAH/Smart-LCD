/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
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

#if 0
#include <compiler.h>
#include <sysclk.h>
#include <gpio.h>
#include <delay.h>
#include <adc.h>
#endif

#include "main.h"


static uint8_t		runmode								= (uint8_t) 0;			// global runmode



/* INIT section */

static void s_tc_init(void)
{
#if 0
	/* LCD backlight PWM signal generation */
	struct pwm_config pwm_vctcxo_cfg;
	pwm_init(&pwm_vctcxo_cfg, PWM_TCC0, PWM_CH_D, 500);							// Init PWM structure and enable timer
	pwm_start(&pwm_vctcxo_cfg, 45);												// Start PWM. Percentage with 1% granularity is to coarse, use driver access instead
	tc_write_cc_buffer(&TCC0, TC_CCD, (uint16_t) (0.5f + 65536 * 1.5f/3.3f));	// Initial value for VCTCXO @ 1.5 V
#endif
}

static void s_tc_start(void)
{
#if 0
	/* ADC clock */
	tc_write_clock_source(&TCC0, TC_TC0_CLKSEL_DIV1_gc);						// ADC clock for LDR
#endif
}


static void s_adc_init(void)
{
	adc_init(ADC_PRESCALER_DIV128);

	irqflags_t flags = cpu_irq_save();
	adc_set_admux(ADC_MUX_ADC0 | ADC_VREF_1V1 | ADC_ADJUSTMENT_LEFT);
	//adc_set_mux(ADC_MUX_ADC0);
	//adc_set_voltage_reference(ADC_VREF_1V1);
	adc_set_autotrigger_source( ADC_AUTOTRIGGER_SOURCE_TC1_OVERFLOW);
	cpu_irq_restore(flags);
}


static void s_reset_global_vars(void)
{
}

/* ISR routines */
void bad_interrupt(void)
{
	__asm__ __volatile__ ("break" ::: "memory");
}

ISR(__vector_1)
{
	bad_interrupt();
}



static void s_task(void)
{
	/* TASK when woken up */

}

void halt(void)
{
	/* MAIN Loop Shutdown */

	runmode = 0;
}



int main (void)
{
	uint8_t retcode = 0;
	
	/* Init of sub-modules */
	//pmic_init();
	sysclk_init();
	s_tc_init();
	s_adc_init();

	/* All interrupt sources prepared here - IRQ activation */
	cpu_irq_enable();
	
	board_init();

	/* Start of sub-modules */
	s_tc_start();			// All clocks and PWM timers start here

	/* Insert application code here, after the board has been initialized. */
	 reset_cause_t rc = reset_cause_get_causes();
	 if (rc & CHIP_RESET_CAUSE_EXTRST || 
		 rc & CHIP_RESET_CAUSE_BOD_CPU || 
		 rc & CHIP_RESET_CAUSE_POR) {
		 s_reset_global_vars();
	 }
	 
    while (runmode) {
	    s_task();
	    //sleepmgr_enter_sleep();
    }
    
    cpu_irq_disable();
    //sleepmgr_enter_sleep();
    
    return retcode;
}
