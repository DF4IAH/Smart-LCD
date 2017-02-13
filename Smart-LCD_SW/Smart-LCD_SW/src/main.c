/**
 * \file
 *
 * \brief The main-loop, init and shutdown of the application 
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
#include "isr.h"

#include "main.h"



/* GLOBAL section */

uint8_t				g_adc_state							= 0;
float				g_adc_ldr							= 0.f;
float				g_adc_ldr_last						= 0.f;
float				g_adc_temp							= 0.f;

float				g_temp								= 0.f;
float				g_temp_lcd_last						= 0.f;



/* MAIN STATIC section */

static uint8_t		runmode								= 0;			// global runmode



/* HELPERS */

static void s_reset_global_vars(void)
{
	irqflags_t flags	= cpu_irq_save();
	
	g_adc_state			= ADC_STATE_PRE_LDR;
	g_adc_ldr			= 0.f;
	g_adc_ldr_last		= 0.f;
	g_adc_temp			= 0.f;
	g_temp				= 0.f;
	g_temp_lcd_last		= 0.f;
	
	cpu_irq_restore(flags);
}


/* INIT section */

static void s_tc_init(void)
{
	irqflags_t flags = cpu_irq_save();

	sysclk_set_prescalers(SYSCLK_PSDIV_1);
	
	/* Timer Synchronous Mode - prepare */
	GTCCR   = _BV(TSM)							// Timer Synchronous Mode active
		    | _BV(PSRASY)						// Timer 2   prescaler is synced
		    | _BV(PSRSYNC);						// Timer 0/1 prescaler is synced

	
	/* TC0: not in use */
	{
		sysclk_disable_module(0, PRTIM0);
	}

	/* TC1 - OC1A: Audio output @ 16-bit counter PWM, used: 10-bit resolution - overflows with 15625 Hz */
	{
		sysclk_enable_module(0, PRTIM1);

		TCCR1A  = (0b10  << COM1A0)		 		// HI --> LO when compare value is reached - non-inverted PWM mode
				| (0b11  << WGM10);				// WGM: 0b0111 = Fast PWM 10 bit

		TCCR1B  = ( 0b01 << WGM12)		 
				| (0b001 << CS10);				// CLKio DIV 1 = 16 MHz
		   
		TCCR1C  = 0;

		TCNT1H  = 0b00000000           ;		// Clear current value for synchronous start (when restarting without reset)
		TCNT1L	=            0b00000000;
	
		OCR1AH  =       0b10           ;		// Mid-range compare value for zero audio output
		OCR1AL  =            0b00000000;
	
		TIMSK1  = 0;							// no interrupts (when restarting without reset)
		TIFR1   = 0b00100111;					// clear all flags (when restarting without reset)
	}

	/* TC2 - OC2A: LCD backlight w/ 8-bit resolution - overflows with abt. 61 Hz */
	{
		sysclk_enable_module(0, PRTIM2);
	
		TCCR2A  = (0b10  << COM2A0)				// HI --> LO when compare value is reached - non-inverted PWM mode
				| (0b11  << WGM20);				// WGM: 0b011 = Fast PWM mode 8 bit

		TCCR2B  = ( 0b0  << WGM22)
				| (0b111 << CS20);				// CLKio DIV 1024 = 15625 Hz
	
		TCNT2   = 0;							// Clear current value for synchronous start (when restarting without reset)
	
		OCR2A   = 0x40;							// LCD backlight dimmed down to 25% 

		TIMSK2  = 0;							// no interrupts (when restarting without reset)
		TIFR2   = 0b00000111;					// clear all flags (when restarting without reset)
	
		ASSR    = 0;							// no async TOSC1 mode
	}

	cpu_irq_restore(flags);
}

static void s_tc_start(void)
{
	/* TC0: not in use */
	/* TC1: Audio output @ 16-bit counter PWM, used: 10-bit resolution */
	/* TC2: LCD backlight w/ 8-bit resolution */
	{
		/* Timer Synchronous Mode - trigger */
		GTCCR   = _BV(PSRSYNC);				    // trigger the sync for all counters
	}
}

static void s_tc_disable(void)
{
	irqflags_t flags = cpu_irq_save();

	/* TC0: not in use */
	{
		sysclk_disable_module(0, PRTIM0);
	}

	/* TC1 - OC1A: Audio output @ 16-bit counter PWM, used: 10-bit resolution - overflows with 15625 Hz */
	{
		// bring pin to high Z mode to reduce audible plop noise
		ioport_set_pin_dir(AUDIO_PWM, IOPORT_DIR_INPUT);
		ioport_set_pin_mode(AUDIO_PWM, IOPORT_MODE_PULLDOWN);

		TCCR1A  = 0;							// release alternate port function
		TCCR1B  = 0;
		TCCR1C  = 0;

		TIMSK1  = 0;							// no interrupts

		sysclk_disable_module(0, PRTIM1);
	}

	/* TC2 - OC2A: LCD backlight w/ 8-bit resolution - overflows with abt. 61 Hz */
	{
		ioport_set_pin_dir(LCDBL_PWM, IOPORT_DIR_OUTPUT);
		ioport_set_pin_level(LCDBL_PWM, false);	// turn backlight off

		TCCR2A  = 0;							// release alternate port function
		TCCR2B  = 0;
		
		TIMSK2  = 0;							// no interrupts
		
		ASSR    = 0;							// no async TOSC1 mode

		sysclk_disable_module(0, PRTIM2);
	}
	
	sysclk_set_prescalers(SYSCLK_PSDIV_256);

	cpu_irq_restore(flags);
}


static void s_adc_init(void)
{
	sysclk_enable_module(0, PRADC);				// enable ADC sub-module
	adc_init(ADC_PRESCALER_DIV128);

	irqflags_t flags = cpu_irq_save();

	adc_disable_digital_inputs(_BV(ADC0D));		// disable the digital input on the ADC0 port
	adc_set_admux(ADC_MUX_ADC0 | ADC_VREF_1V1 | ADC_ADJUSTMENT_RIGHT);
	//adc_set_mux(ADC_MUX_ADC0);
	//adc_set_voltage_reference(ADC_VREF_1V1);

	adc_set_autotrigger_source(ADC_AUTOTRIGGER_SOURCE_TC1_OVERFLOW);
	adc_enable_interrupt();						// enable the ADC interrupt

	cpu_irq_restore(flags);
}

static void s_adc_disable(void)
{
	irqflags_t flags = cpu_irq_save();

	adc_disable_interrupt();					// disable the ADC interrupt
	adc_set_autotrigger_source(0);
	adc_set_admux(0);
	adc_disable_digital_inputs(0);

	sysclk_disable_module(0, PRADC);			// disable ADC sub-module

	cpu_irq_restore(flags);
}

static void s_twi_init(void)
{
	sysclk_enable_module(0, PRTWI);
	
}

static void s_twi_disable(void)
{

	sysclk_disable_module(0, PRTWI);
}


/* TASK section */

static void s_task_backlight(float adc_ldr)
{
	/* calculate the 8-bit backlight PWM value based on the ADC LDR voltage */
	const uint16_t	MAX_INTENSITY		= 10000;
	const uint16_t	BL_OFF_INTENSITY	=  1000;
	const uint8_t	BL_MIN_PWM			=    26;  // 10%
	float			intensity			= MAX_INTENSITY;
	uint8_t			pwm					= 0;
	
	if (adc_ldr >= 1.f) {
		intensity = (MAX_INTENSITY >> 1) / adc_ldr;  // 1 <= adc <= 1023
	}
	
	if (intensity < BL_OFF_INTENSITY) {
		pwm = BL_MIN_PWM + (uint8_t)((255 - BL_MIN_PWM) * (intensity / BL_OFF_INTENSITY));
	}
	
	OCR2A = pwm;								// no interrupt lock needed
}

static void s_task_temp(float adc_temp)
{
	const float C_temp_coef_k			= 1.0595703f;
	const float C_temp_coef_ofs_atmel	= 1024 * 0.314f / 1.1f;
	const float C_temp_coef_ofs			= 54.0f + C_temp_coef_ofs_atmel;
	float l_temp_lcd_last;

	/* Temperature calculation for °C */
	float l_temp = 25.f + ((adc_temp	- C_temp_coef_ofs) * C_temp_coef_k);
	
	irqflags_t flags = cpu_irq_save();
	l_temp_lcd_last = g_temp_lcd_last;
	g_temp = l_temp;
	cpu_irq_restore(flags);
	
	if (abs(l_temp - l_temp_lcd_last) > 1.f) {
		flags = cpu_irq_save();
		g_temp_lcd_last = l_temp;
		cpu_irq_restore(flags);
		
		// lcd_temp(l_temp);
	}
}

static void s_task(void)
{
	/* TASK when woken up */
	irqflags_t flags		= cpu_irq_save();
	float l_adc_ldr			= g_adc_ldr;
	float l_adc_ldr_last	= g_adc_ldr_last;

	float l_adc_temp		= g_adc_temp;
	cpu_irq_restore(flags);
	
	/* calculate new backlight PWM value and set that */
	if (abs(l_adc_ldr - l_adc_ldr_last) >= 0.5f) {
		s_task_backlight(l_adc_ldr);
		
		flags = cpu_irq_save();
		g_adc_ldr_last = l_adc_ldr;
		cpu_irq_restore(flags);
	}
	
	/* calculate new current temperature */
	s_task_temp(l_adc_temp);
}

static void s_enter_sleep(uint8_t sleep_mode)
{
	SMCR  = (sleep_mode << SM0)
		  | _BV(SE);							// enable sleep command
	
	__asm__ __volatile__ ("sleep" ::: "memory");
	
	SMCR &= ~(_BV(SE));							// disable sleep command
} 


/* MAIN section */

void halt(void)
{
	/* MAIN Loop Shutdown */
	runmode = 0;
}

int main (void)
{
	uint8_t retcode = 0;
	
	/* Init of sub-modules */
	sysclk_init();
	ioport_init();
	s_tc_init();
	s_adc_init();
	ACSR |= _BV(ACD);							// disable AnalogCompare sub-module
	
	/* I/O pins go active here */
	board_init();
	
	reset_cause_t rc = reset_cause_get_causes();
	if (rc & CHIP_RESET_CAUSE_EXTRST	||
		rc & CHIP_RESET_CAUSE_BOD_CPU	||
		rc & CHIP_RESET_CAUSE_POR) {
		s_reset_global_vars();
	} else {
		/* DEBUG */
		asm_break();
	}
	
	s_twi_init();
	
	/* All interrupt sources prepared here - IRQ activation */
	cpu_irq_enable();
	
	/* Start of sub-modules */
	s_tc_start();								// All clocks and PWM timers start here
	
	
	/* main loop */
    while (runmode) {
	    s_task();
	    s_enter_sleep(SLEEP_MODE_IDLE);
    }
	
	
	cpu_irq_disable();
    
	/* disable sub-modules */
	ACSR |= _BV(ACD);							// disable AnalogCompare sub-module
	sysclk_disable_module(0, PRSPI);
	sysclk_disable_module(0, PRUSART0);
	s_twi_disable();
	s_adc_disable();
	s_tc_disable();
	
    s_enter_sleep(SLEEP_MODE_PWR_DOWN);
    
    return retcode;								// should never be reached
}
