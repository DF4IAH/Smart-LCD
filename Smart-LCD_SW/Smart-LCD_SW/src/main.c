/**
 * \file
 *
 * \brief The main-loop, init and shutdown of the application
 *
 */

/**
 * \mainpage Main module
 *
 * \par The main-loop, init and shutdown of the application
 *
 * The Main module includes the main loop of the application. Any
 * needed initialization and shutdown code is located here, also.
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
#include <stdio.h>

#include "isr.h"
#include "twi.h"
#include "lcd.h"
#include "gfx_mono/sysfont.h"

#include "main.h"


#define ADC_TEMP_DELTA	0.005f
#define ADC_LDR_DELTA   25.0f


/* GLOBAL section */

uint_fast32_t		g_timer_abs_msb						= 0;

uint8_t				g_adc_state							= 0;
float				g_adc_ldr							= 0.f;
float				g_adc_ldr_last						= 0.f;
float				g_adc_temp							= 0.f;
float				g_adc_temp_last						= 0.f;

float				g_temp								= 0.f;
uint8_t				g_lcdbl_dimmer						= 0;

uint8_t				g_u8_DEBUG11						= 0,
					g_u8_DEBUG12						= 0,
					g_u8_DEBUG13						= 0;

uint_fast32_t		g_u32_DEBUG21						= 0;

float				g_f_DEBUG31							= 0.f,
					g_f_DEBUG32							= 0.f;



/* MAIN STATIC section */

static uint8_t		runmode								= 0;			// static runmode of main.c



/* HELPERS */

static void s_reset_global_vars(void)
{
	irqflags_t flags	= cpu_irq_save();

	g_adc_state			= ADC_STATE_PRE_LDR;
	g_adc_ldr			= 0.f;
	g_adc_ldr_last		= 0.f;
	g_adc_temp			= 0.f;
	g_adc_temp_last		= 0.f;

	g_temp				= 25.f;
	g_lcdbl_dimmer		= 64;

	cpu_irq_restore(flags);
}


/* INIT section */

static void s_io_preinit(void)
{
	/* This function is called prior enabled interrupts and thus does not lock interrupts,
	 * most critical pins are handled first.
	 */

	PORTC = 0b01111110;		// PC0: LDR-ADC in-NoPU, PC1: SW_I in-PU, PC2: SW_Q in-PU, PC3: LCD-CS out-HI
	DDRC  = 0b00001000;		// PC4: I2C-SDA in-PU-AF-TWI, PC5: I2C-SCL in-PU-AF-TWI, PC6: RESET in-PU, PC7: -

	PORTB = 0b00010100;		// PB0: LCD-CD out-LO, PB1: AUDIO out-AF-OC1A, PB2: SW_P in-PU, PB3: LCDBL out-AF-OC2A,
	DDRB  = 0b11111011;		// PB4: LCD-R/!W out-HI, PB5: LCD-EN out-LO, PB6: LEDRD out-LO, LEDGN out-LO

	PORTD = 0xff;			// PD0..PD7: LCD-D0..LCD-D7 in-PU
	DDRD  = 0x00;

	// Analog input: Digital Disable Register
	DIDR0 = 0b00000001;		// PC0: LDR-ADC
}

static void s_tc_init(void)
{
	/* This function is called prior enabled interrupts and thus does not lock interrupts. */

	/* Timer Synchronous Mode - prepare for  s_tc_start(void) */
	GTCCR = _BV(TSM)							// Timer Synchronous Mode active
		  | _BV(PSRASY)							// Timer 2   prescaler is synced
		  | _BV(PSRSYNC);						// Timer 0/1 prescaler is synced

	/* TC0: not in use */
	{
	}

	/* TC1 - OC1A: Audio output @ 16-bit counter PWM, used: 10-bit resolution - overflows with 15625 Hz */
	{
		sysclk_enable_module(POWER_RED_REG0, PRTIM1_bm);

		TCCR1A  = (0b10  << COM1A0)		 		// HI --> LO when compare value is reached - non-inverted PWM mode
				| (0b10  << WGM10);				// WGM: 0b0110 = Fast PWM 9 bit

		TCCR1B  = ( 0b01 << WGM12)
				| (0b001 << CS10);				// CLKio DIV1 = 8 MHz

		TCNT1H  = 0b00000000           ;		// Clear current value for synchronous start (when restarting without reset)
		barrier();
		TCNT1L	=            0b00000000;

		OCR1AH  =        0b1           ;		// Mid-range compare value for zero audio output
		OCR1AL  =            0b00000000;

		TIMSK1  = _BV(TOIE1);					// TOIE1 interrupt
		TIFR1   = 0b00100111;					// Clear all flags (when restarting without reset)
	}

	/* TC2 - OC2A: LCD backlight w/ 8-bit resolution - overflows with abt. 61 Hz */
	{
		sysclk_enable_module(POWER_RED_REG0, PRTIM2_bm);

		TCCR2A  = (0b10  << COM2A0)				// HI --> LO when compare value is reached - non-inverted PWM mode
				| (0b11  << WGM20);				// WGM: 0b011 = Fast PWM mode 8 bit

		TCCR2B  = ( 0b0  << WGM22)
				| (0b101 << CS20);				// CLKio DIV 128 = 62500 Hz --> / 2**8 = 244 Hz looping rate

		TCNT2   = 0;							// Clear current value for synchronous start (when restarting without reset)

		OCR2A   = 0x00;							// LCD backlight dimmed down

		TIMSK2  = 0b00000000;					// No interrupts
		TIFR2   = 0b00000111;					// Clear all flags

		ASSR    = 0;							// No async. TOSC1 mode
	}
}

static void s_tc_start(void)
{
	/* TC0: not in use */
	/* TC1: Audio output @ 16-bit counter PWM, used: 10-bit resolution */
	/* TC2: LCD backlight w/ 8-bit resolution */
	{
		/* Timer Synchronous Mode - trigger */
		GTCCR = 0;								// trigger the sync for all counters
	}
}

static void s_tc_disable(void)
{
	irqflags_t flags = cpu_irq_save();

	/* TC0: not in use */
	{
		sysclk_disable_module(POWER_RED_REG0, PRTIM0_bm);
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

		sysclk_disable_module(POWER_RED_REG0, PRTIM1_bm);
	}

	/* TC2 - OC2A: LCD backlight w/ 8-bit resolution - overflows with abt. 61 Hz */
	{
		ioport_set_pin_dir(LCDBL_PWM, IOPORT_DIR_OUTPUT);
		ioport_set_pin_level(LCDBL_PWM, false);	// turn backlight off

		TCCR2A  = 0;							// release alternate port function
		TCCR2B  = 0;

		TIMSK2  = 0;							// no interrupts

		ASSR    = 0;							// no async TOSC1 mode

		sysclk_disable_module(POWER_RED_REG0, PRTIM2_bm);
	}

	sysclk_set_prescalers(SYSCLK_PSDIV_256);

	cpu_irq_restore(flags);
}


static void s_adc_init(void)
{
	sysclk_enable_module(POWER_RED_REG0, PRADC_bm);	// enable ADC sub-module

	adc_disable_digital_inputs(_BV(ADC0D));		// disable the digital input on the ADC0 port

	adc_init(ADC_PRESCALER_DIV128);
	adc_set_admux(ADC_MUX_ADC0 | ADC_VREF_1V1 | ADC_ADJUSTMENT_RIGHT);

	ADCSRA |= _BV(ADIF);						// clear interrupt status bit by setting it to clear
	adc_enable_interrupt();						// enable the ADC interrupt

	adc_set_autotrigger_source(ADC_AUTOTRIGGER_SOURCE_TC1_OVERFLOW);
	adc_enable_autotrigger();
}

static void s_adc_disable(void)
{
	adc_disable_interrupt();					// disable the ADC interrupt
	adc_disable_autotrigger();
	adc_set_autotrigger_source(0);
	adc_set_admux(0);
	adc_disable_digital_inputs(0);

	sysclk_disable_module(POWER_RED_REG0, PRADC_bm);	// disable ADC sub-module
}

static void s_twi_init(void)
{
	sysclk_enable_module(POWER_RED_REG0, PRTWI_bm);

	irqflags_t flags = cpu_irq_save();

	TWSR = (0b00 << TWPS0);						// Prescaler value = 1
	TWBR = 2;									// TWI bit-rate = 400 kBit/sec @ 8 MHz when master mode active

	TWAR  = (TWI_SLAVE_ADDR    << TWA0) /* | (TWI_SLAVE_ADDR_GCE << TWGCE)*/ ;
	TWAMR = (TWI_SLAVE_ADDR_BM << TWAM0);

	TWCR = _BV(TWEA) | _BV(TWEN) | _BV(TWIE);	// Enable Acknowledge, ENable TWI port, Interrupt Enable, no START or STOP bit
	//TWCR = _BV(TWSTA) | _BV(TWEN) | _BV(TWIE);	// TEST: Enable START, ENable TWI port, Interrupt Enable

	cpu_irq_restore(flags);
}

static void s_twi_disable(void)
{
	irqflags_t flags = cpu_irq_save();

	TWCR = _BV(TWEN);							// disable the interrupt source

	ioport_set_pin_dir(SDA_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(SDA_GPIO, IOPORT_MODE_PULLUP);

	ioport_set_pin_dir(SCL_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(SCL_GPIO, IOPORT_MODE_PULLUP);

	TWCR = 0;									// disable the TWI port

	cpu_irq_restore(flags);

	sysclk_disable_module(POWER_RED_REG0, PRTWI_bm);
}


/* UTILITIES section */

float get_abs_time(void)
{
	const float ticks_per_sec = 8e6f;
	float now;
	uint8_t l_tmr_l;
	uint8_t l_tmr_h;
	uint_fast32_t l_tmr_msb;

	irqflags_t flags = cpu_irq_save();
	l_tmr_l = TCNT1L;
	l_tmr_h = TCNT1H;
	l_tmr_msb = g_timer_abs_msb;
	cpu_irq_restore(flags);

	/* calculate ticks to sec */
	now  = ((l_tmr_h << 8) | l_tmr_l  ) / ticks_per_sec;
	now += (        512.f  * l_tmr_msb) / ticks_per_sec;
	return now;
}

void mem_set(uint8_t* buf, uint8_t count, uint8_t val)
{
	for (int i = count; i; --i) {
		*(buf++) = val;
	}
}


/* TASK section */

static void s_task_backlight(float adc_photo)
{
	char buf[16];

	/* calculate the 8-bit backlight PWM value based on the ADC LDR voltage */
	const uint16_t  BL_ADC_MAXVAL		=  1023;
	const uint16_t	BL_OFF_INTENSITY	=   950;
	uint16_t lum = (uint16_t) (BL_ADC_MAXVAL - adc_photo);

	if (lum < BL_OFF_INTENSITY) {
		OCR2A	= (uint8_t) (255.0f * (((float) lum) / (BL_OFF_INTENSITY - 1)));	// no interrupt lock needed
		TCCR2A |= (0b10  << COM2A0);

	} else {
		// too much light for backlight
		OCR2A   = 0;
		TCCR2A &= ~(0b11  << COM2A0);
	}

	sprintf(buf, "LUM=%4d", lum);
	gfx_mono_draw_string(buf, 120, 81, lcd_get_sysfont());
}

static void s_task_temp(float adc_temp)
{
	const float C_temp_coef_k			= 1.0595703f;
	const float C_temp_coef_ofs_atmel	= 1024 * 0.314f / 1.1f;
	const float C_temp_coef_ofs			= 59.25f + C_temp_coef_ofs_atmel;

	/* Temperature calculation for °C */
	float l_temp = 25.f + (adc_temp - C_temp_coef_ofs) * C_temp_coef_k;

	irqflags_t flags = cpu_irq_save();
	g_temp = l_temp;
	g_adc_temp_last = adc_temp;
	cpu_irq_restore(flags);
}

void s_task(void)
{
	/* TASK when woken up */
	irqflags_t flags		= cpu_irq_save();
	float l_adc_ldr			= g_adc_ldr;
	float l_adc_ldr_last	= g_adc_ldr_last;
	float l_adc_temp		= g_adc_temp;
	float l_adc_temp_last	= g_adc_temp_last;
	cpu_irq_restore(flags);

	/* calculate new backlight PWM value and set that */
	float ldr_diff = l_adc_ldr - l_adc_ldr_last;
	if (ldr_diff <= -ADC_LDR_DELTA || ADC_LDR_DELTA <= ldr_diff) {
		s_task_backlight(l_adc_ldr);

		flags = cpu_irq_save();
		g_adc_ldr_last = l_adc_ldr;
		cpu_irq_restore(flags);
	}

	/* calculate new current temperature */
	float temp_diff = l_adc_temp - l_adc_temp_last;
	if (temp_diff <= -ADC_TEMP_DELTA || ADC_TEMP_DELTA <= temp_diff) {
		s_task_temp(l_adc_temp);
	}

	/* animated picture */
	lcd_animation_loop();
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

	/* Rapid I/O settings */
	s_io_preinit();

	/* Init of sub-modules */
	sysclk_init();	PRR = 0b11101011;			// For debugging this module has to be powered on, again
	ioport_init();
	s_tc_init();
	s_adc_init();

	/* I/O pins go active here */
	board_init();

	reset_cause_t rc = reset_cause_get_causes();
	if (rc & CHIP_RESET_CAUSE_EXTRST	||
		rc & CHIP_RESET_CAUSE_BOD_CPU	||
		rc & CHIP_RESET_CAUSE_POR		||
		!rc) {
		s_reset_global_vars();
	} else {
		/* DEBUG */
		g_u32_DEBUG21 = rc;
		asm_break();
	}

	/* I2C interface */
	s_twi_init();

	/* All interrupt sources prepared here - IRQ activation */
	cpu_irq_enable();

	/* Start of sub-modules */
	s_tc_start();								// All clocks and PWM timers start here

	/* Initialize external components */
	lcd_init();
	lcd_test(0b11110001);						// Debugging purposes


	/* main loop */
	runmode = 1;
    while (runmode) {
	    s_task();
	    s_enter_sleep(SLEEP_MODE_IDLE);
    }


	/* Shutdown external components */
	lcd_shutdown();

	cpu_irq_disable();

	/* disable sub-modules */
	ACSR |= _BV(ACD);							// disable AnalogCompare sub-module

	sysclk_disable_module(POWER_RED_REG0, PRSPI_bm);
	sysclk_disable_module(POWER_RED_REG0, PRUSART0_bm);

	s_twi_disable();
	s_adc_disable();
	s_tc_disable();

    s_enter_sleep(SLEEP_MODE_PWR_DOWN);

    return retcode;								// should never be reached
}
