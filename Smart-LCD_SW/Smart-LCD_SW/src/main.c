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
#include <avr/eeprom.h>

#include "isr.h"
#include "twi.h"
#include "lcd.h"
#include "gfx_mono/sysfont.h"

#include "main.h"


/* GLOBAL section */

uint32_t			g_timer_abs_100us					= 0;
uint8_t				g_adc_state							= 0;
float				g_adc_light							= 0.f;
float				g_adc_temp							= 0.f;
float				g_temp								= 0.f;
bool				g_lcdbl_auto						= true;
uint8_t				g_lcdbl_dimmer						= 0;
uint8_t				g_lcd_contrast_pm					= 0;
bool				g_audio_out_on						= false;
uint8_t				g_audio_out_mod						= 0;
uint8_t				g_audio_out_length					= 0;
int16_t				g_audio_pwm_accu					= 0;
uint8_t				g_audio_pwm_ramp_dwn				= 0;
status_t			g_status							= { 0 };
showData_t			g_showData							= { 0 };
volatile buttons_t	g_buttons							= { 0 };
uint32_t			g_rotenc_events						= 0;
uint8_t				g_SmartLCD_mode						= C_SMART_LCD_MODE_UNIQUE;
gfx_mono_color_t	g_lcd_pixel_type					= GFX_PIXEL_CLR;
gfx_coord_t			g_lcd_pencil_x						= 0;
gfx_coord_t			g_lcd_pencil_y						= 0;
bool				g_led_red							= false;
bool				g_led_green							= false;
uint8_t				g_resetCause						= 0;
char				g_strbuf[48]						= { 0 };


/* MAIN STATIC section */

static uint8_t		runmode								= 0;		// Static runmode of main.c



/* HELPERS */

static void s_reset_global_vars(void)
{
	irqflags_t flags	= cpu_irq_save();
	cpu_irq_disable();

	g_adc_state			= ADC_STATE_PRE_LDR;
	g_adc_light			= 0.f;
	g_adc_temp			= 0.f;

	g_temp				= 25.f;
	g_lcdbl_dimmer		= 64;

	g_status.doAnimation = false;
	g_status.isAnimationStopped = false;

	cpu_irq_restore(flags);
}

inline
static uint8_t s_2bit_gray_to_dec(uint8_t gray)
{
	gray &= 0x03;
	return gray ^ (gray >> 1);
}


/* INIT section */

static void s_io_preinit(void)
{
	/* This function is called prior enabled interrupts and thus does not lock interrupts,
	 * most critical pins are handled first.
	 */

	PORTB = 0b00010100;												// PB0: LCD-CD out-LO, PB1: AUDIO out (OC1A), PB2: SW_P in-PU, PB3: LCDBL out-AF-OC2A,
	DDRB  = 0b11111011;												// PB4: LCD-R/!W out-HI, PB5: LCD-EN out-LO, PB6: LEDRD out-LO, LEDGN out-LO

	PORTC = 0b01111110;												// PC0: LDR-ADC in-NoPU, PC1: SW_I in-PU, PC2: SW_Q in-PU, PC3: LCD-CS out-HI
	DDRC  = 0b00001000;												// PC4: I2C-SDA in-PU-AF-TWI, PC5: I2C-SCL in-PU-AF-TWI, PC6: RESET in-PU, PC7: -

	PORTD = 0xff;													// PD0..PD7: LCD-D0..LCD-D7 in-PU
	DDRD  = 0x00;

	// Analog input: Digital Disable Register
	DIDR0 = 0b00000001;												// PC0: LDR-ADC
}

static void s_tc_init(void)
{
	/* This function is called prior enabled interrupts and thus does not lock interrupts. */

	/* Timer Synchronous Mode - prepare for  s_tc_start(void) */
	GTCCR = _BV(TSM)												// Timer Synchronous Mode active
		  | _BV(PSRASY)												// Timer 2   prescaler is synced
		  | _BV(PSRSYNC);											// Timer 0/1 prescaler is synced

	/* TC0 - Overflows with about 122 Hz for the ADC convertion */
	{
		sysclk_enable_module(POWER_RED_REG0, PRTIM0_bm);

		TCCR0A  = (0b00  << COM0A0)									// Normal port operations
				| (0b00  << WGM00);									// Counter mode

		TCCR0B  = ( 0b0  << WGM02)
				| (0b100 << CS00);									// CLKio DIV 256 = 31250 Hz --> / 2**8 = 122 Hz looping rate

		TCNT0   = 0;												// Clear current value

		OCR0A   = 0x00;												// Compare value not used

		TIFR0   = 0b00000111;										// Clear all flags
		TIMSK0  = _BV(TOIE0);										// TOIE0: overflow interrupt
	}

	/* TC1 - OC1A: 10 kHz - for audio output and millisecond counter */
	{
		sysclk_enable_module(POWER_RED_REG0, PRTIM1_bm);

		TCCR1A  = (0b00  << COM1A0)		 							// Normal port operation
				| (0b10  << WGM10);									// WGM: 0b1110 = Fast PWM, TOP := ICR1

		TCCR1B  = (0b11  << WGM12)
				| (0b001 << CS10);									// CLKio DIV1 = 8 MHz

		TCNT1H  = (uint8_t)0;										// Clear current value for synchronous start (when restarting without reset)
		barrier();
		TCNT1L	= (uint8_t)0;

		ICR1H	= (uint8_t)(C_TC1_TOPVAL >> 8);						// TOP value of the counter
		barrier();
		ICR1L	= (uint8_t)(C_TC1_TOPVAL & 0xff);

		TIFR1   = 0b00100111;										// Clear all flags (when restarting without reset)
		TIMSK1  = _BV(TOIE1);										// TOIE1 interrupt
	}

	/* TC2 - OC2A: LCD backlight w/ 8-bit resolution - overflows with abt. 61 Hz */
	{
		sysclk_enable_module(POWER_RED_REG0, PRTIM2_bm);

		ASSR    = 0;												// No async. TOSC1 mode

		TCCR2A  = (0b10  << COM2A0)									// HI --> LO when compare value is reached - non-inverted PWM mode
				| (0b11  << WGM20);									// WGM: 0b011 = Fast PWM mode 8 bit

		TCCR2B  = ( 0b0  << WGM22)
				| (0b101 << CS20);									// CLKio DIV 128 = 62500 Hz --> / 2**8 = 244 Hz looping rate

		TCNT2   = 0;												// Clear current value for synchronous start (when restarting without reset)

		OCR2A   = 0x00;												// LCD backlight dimmed down

		TIFR2   = 0b00000111;										// Clear all flags
		TIMSK2  = _BV(TOIE2);										// TOIE2: overflow interrupt
	}
}

static void s_tc_start(void)
{
	/* TC0: Overflows with about 30 Hz for the ADC convertion */
	/* TC1: Audio output @ 16-bit counter PWM, used: 10-bit resolution */
	/* TC2: LCD backlight w/ 8-bit resolution */
	{
		/* Timer Synchronous Mode - trigger */
		GTCCR = 0;													// Trigger the sync for all counters
	}
}

static void s_tc_disable(void)
{
	irqflags_t flags = cpu_irq_save();

	/* TC0 - Overflows with about 122 Hz for the ADC convertion */
	{
		TIMSK0  = 0;												// No interrupts

		TCCR0A  = 0;
		TCCR0B  = 0;

		sysclk_disable_module(POWER_RED_REG0, PRTIM0_bm);
	}

	/* TC1 - OC1A: Audio output @ 16-bit counter PWM, used: 10-bit resolution - overflows with 15625 Hz */
	{
		/* bring pin to high Z mode to reduce audible plop noise */
		ioport_set_pin_dir(AUDIO_PWM, IOPORT_DIR_INPUT);
		ioport_set_pin_mode(AUDIO_PWM, IOPORT_MODE_PULLDOWN);

		TIMSK1  = 0;												// No interrupts

		TCCR1A  = 0;												// Release alternate port function
		TCCR1B  = 0;
		TCCR1C  = 0;

		sysclk_disable_module(POWER_RED_REG0, PRTIM1_bm);
	}

	/* TC2 - OC2A: LCD backlight w/ 8-bit resolution - overflows with abt. 61 Hz */
	{
		ioport_set_pin_dir(LCDBL_PWM, IOPORT_DIR_OUTPUT);
		ioport_set_pin_level(LCDBL_PWM, false);						// Turn backlight off

		TIMSK2  = 0;												// No interrupts

		ASSR    = 0;												// No async TOSC1 mode

		TCCR2A  = 0;												// Release alternate port function
		TCCR2B  = 0;

		sysclk_disable_module(POWER_RED_REG0, PRTIM2_bm);
	}

	sysclk_set_prescalers(SYSCLK_PSDIV_256);

	cpu_irq_restore(flags);
}


static void s_adc_init(void)
{
	sysclk_enable_module(POWER_RED_REG0, PRADC_bm);	// enable ADC sub-module

	adc_disable_digital_inputs(_BV(ADC0D));							// Disable the digital input on the ADC0 port

	adc_init(ADC_PRESCALER_DIV64);
	adc_set_admux(ADC_MUX_ADC0 | ADC_VREF_1V1 | ADC_ADJUSTMENT_RIGHT);

#if 1
	/* ADC is started by TC0 timer directly - disadvantage: lower amplitude precision */
	adc_set_autotrigger_source(ADC_AUTOTRIGGER_SOURCE_TC0_OVERFLOW);
	adc_enable_autotrigger();
#else
	adc_disable_autotrigger();
#endif

	ADCSRA |= _BV(ADIF);											// Clear interrupt status bit by setting it to clear
	adc_enable_interrupt();											// Enable the ADC interrupt

	adc_start_conversion();											// Initial convertion doing an ADC set-up
}

static void s_adc_disable(void)
{
	adc_disable_interrupt();										// Disable the ADC interrupt
	adc_disable_autotrigger();
	adc_set_autotrigger_source(0);
	adc_set_admux(0);
	//adc_disable_digital_inputs(0);

	sysclk_disable_module(POWER_RED_REG0, PRADC_bm);				// Disable ADC sub-module
}

static void s_twi_init(uint8_t twi_addr, uint8_t twi_addr_bm)
{
	sysclk_enable_module(POWER_RED_REG0, PRTWI_bm);

	irqflags_t flags = cpu_irq_save();

	TWSR = (0b00 << TWPS0);											// Prescaler value = 1
	TWBR = 2;														// TWI bit-rate = 400 kBit/sec @ 8 MHz when master mode active

	TWAR  = (twi_addr    << 1) /* | (TWI_SLAVE_ADDR_GCE << TWGCE)*/ ;
	TWAMR = (twi_addr_bm << 1);

	TWCR = _BV(TWEA) | _BV(TWEN) | _BV(TWIE);						// Enable Acknowledge, Enable TWI port, Interrupt Enable, no START or STOP bit

	cpu_irq_restore(flags);
}

static void s_twi_disable(void)
{
	irqflags_t flags = cpu_irq_save();

	TWCR = _BV(TWEN);												// Disable the interrupt source

	ioport_set_pin_dir(SDA_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(SDA_GPIO, IOPORT_MODE_PULLUP);

	ioport_set_pin_dir(SCL_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(SCL_GPIO, IOPORT_MODE_PULLUP);

	TWCR = 0;														// Disable the TWI port

	cpu_irq_restore(flags);

	sysclk_disable_module(POWER_RED_REG0, PRTWI_bm);
}


/* UTILITIES section */

float get_abs_time(void)
{
	uint32_t now_us;
	uint8_t l_tmr_l;
	uint8_t l_tmr_h;
	uint32_t l_tmr_100us;

	irqflags_t flags = cpu_irq_save();
	l_tmr_l = TCNT1L;
	l_tmr_h = TCNT1H;
	l_tmr_100us = g_timer_abs_100us;
	cpu_irq_restore(flags);

	/* Calculate ticks to sec */
	now_us  = ((((uint16_t)l_tmr_h << 8) | l_tmr_l) >> 3);			// CPUclk = 8 MHz
	now_us += l_tmr_100us * 100U;

	return now_us * 1e-6f;
}

void mem_set(uint8_t* buf, uint8_t count, uint8_t val)
{
	for (int i = count; i; --i) {
		*(buf++) = val;
	}
}

void eeprom_nvm_settings_write(uint8_t flags)
{
	/* I2C_VERSION */
	if (flags & C_EEPROM_NVM_SETTING_VERSION) {
		eeprom_write_byte((uint8_t *) C_EEPROM_ADDR_VERSION,
		I2C_VERSION);
	}

	/* LCD_PM */
	if (flags & C_EEPROM_NVM_SETTING_LCD_CONTRAST) {
		eeprom_write_byte((uint8_t *) C_EEPROM_ADDR_LCD_PM,
		g_lcd_contrast_pm & 0x3F);
	}
}

void eeprom_nvm_settings_read(uint8_t flags)
{
	/* I2C_VERSION */
	if (flags & C_EEPROM_NVM_SETTING_VERSION) {
		uint8_t ver = eeprom_read_byte((const uint8_t *) C_EEPROM_ADDR_VERSION);
		if (ver != I2C_VERSION) {
			eeprom_nvm_settings_write(C_EEPROM_NVM_SETTING_VERSION);
		}
	}

	/* LCD_PM */
	g_lcd_contrast_pm = C_LCD_PM;									// Preset value
#if 0
	if (flags & C_EEPROM_NVM_SETTING_LCD_CONTRAST) {
		uint8_t val = eeprom_read_byte((const uint8_t *) C_EEPROM_ADDR_LCD_PM);
		if (val <= 0x3F) {											// Value from NVM is marked as being valid
			g_lcd_contrast_pm = val;
		} else {
			eeprom_nvm_settings_write(C_EEPROM_NVM_SETTING_LCD_CONTRAST);
		}
	}
#endif
}


/* TASK section */

static void s_task_temp(float adc_temp)
{
	const float C_temp_coef_k			= 1.0595703f;
	const float C_temp_coef_ofs_atmel	= 1024 * 0.314f / 1.1f;
	const float C_temp_coef_ofs			= 64.50f + C_temp_coef_ofs_atmel;

	/* Temperature calculation for °C */
	float l_temp = 25.f + (adc_temp - C_temp_coef_ofs) * C_temp_coef_k;

	irqflags_t flags = cpu_irq_save();
	g_temp = l_temp;
	cpu_irq_restore(flags);
}

static void s_task_backlight(float adc_light, float timestamp)
{
	/* Calculate the 8-bit backlight PWM value based on the ADC photo diode current */
	const uint16_t	BL_ADC_OFF			= 950;
	const uint16_t	BL_MIN_INTENSITY	=   2;
	static int16_t  pwm_last =  0;
	static float    ts_last = 0.f;

	if (adc_light < BL_ADC_OFF) {
		int16_t pwm_now = (int16_t) (BL_MIN_INTENSITY + (255.0f - BL_MIN_INTENSITY) * ((adc_light - BL_MIN_INTENSITY) / BL_ADC_OFF));	// no interrupt lock needed
		if ((pwm_now > (pwm_last + 1)) || (pwm_now < (pwm_last - 1)) || ((timestamp - ts_last) > 0.5f) || (timestamp < ts_last)) {
			OCR2A = (uint8_t) pwm_now;
			pwm_last = pwm_now;
			ts_last = timestamp;
		}
		TCCR2A |= (0b10 << COM2A0);
#if 1
	} else {
		/* Enough ambient light for display, turn backlight off */
		OCR2A   = 0;
		TCCR2A &= ~(0b11  << COM2A0);
#endif
	}
}

static void s_task_buttons(uint8_t but_portB, uint8_t but_portC, float timestamp)
{
	static uint8_t dec_old = 0;

	irqflags_t flags = cpu_irq_save();
	buttons_t l_buttons = g_buttons;
	cpu_irq_restore(flags);

	l_buttons.pushBut	  = but_portB & 0x04 ?  0 : 1;
	l_buttons.rotEnc_I	  = but_portC & 0x02 ?  0 : 1;
	l_buttons.rotEnc_Q	  = but_portC & 0x04 ?  0 : 1;
	l_buttons.rotEnd_quad = s_2bit_gray_to_dec((l_buttons.rotEnc_Q << 1) | l_buttons.rotEnc_I);

	int8_t delta = l_buttons.rotEnd_quad - dec_old;
	if ((delta ==  1) ||
	    (delta == -3)
	) {
		if (!l_buttons.rotEnd_quad) {
			l_buttons.counter++;
		}

	} else if ((delta == -1) ||
	           (delta ==  3)
	) {
		if (l_buttons.rotEnd_quad == 3) {
			l_buttons.counter--;
		}
	}

	dec_old = l_buttons.rotEnd_quad;

	flags = cpu_irq_save();
	g_buttons = l_buttons;
	cpu_irq_restore(flags);
}

void task(float timestamp)
{
	/* TASK when woken up */
	float l_adc_temp, l_adc_light;
	uint8_t l_portB, l_portC;
	irqflags_t flags;
	uint8_t l_SmartLCD_mode, l_doAnimation, l_isAnimationStopped;
	uint8_t more;

	flags = cpu_irq_save();
	l_adc_temp = g_adc_temp;
	l_adc_light = g_adc_light;
	l_portB = PINB;
	l_portC = PINC;
	cpu_irq_restore(flags);

	/* Calculate new current temperature */
	s_task_temp(l_adc_temp);

	/* Calculate new backlight PWM value and set that */
	if (g_lcdbl_auto) {
		s_task_backlight(l_adc_light, timestamp);
	}

	/* Detect button pushes and rotary encoder settings */
	s_task_buttons(l_portB, l_portC, timestamp);

	/* Loops as long as more data is ready to be presented */
	do {
		more = 0;

		flags = cpu_irq_save();
		l_SmartLCD_mode = g_SmartLCD_mode;
		l_doAnimation = g_status.doAnimation;						// TWI command TWI_SMART_LCD_CMD_SET_MODE can unset this flag
		l_isAnimationStopped = g_status.isAnimationStopped;
		cpu_irq_restore(flags);

		/* Show received data from I2C bus */
		if (l_isAnimationStopped) {
			if (l_SmartLCD_mode == C_SMART_LCD_MODE_SMARTLCD) {
				more = lcd_show_new_smartlcd_data();

			} else if (l_SmartLCD_mode == C_SMART_LCD_MODE_REFOSC) {
				more = lcd_show_new_refosc_data();
			}
		}

		/* When transferring from animated demo to stopped animation */
		if (!l_doAnimation) {
			static int8_t s_last_animation = true;

			if (s_last_animation) {
				s_last_animation = false;

				lcd_cls();

				if (l_SmartLCD_mode == C_SMART_LCD_MODE_REFOSC) {
					/* Come up with the data presenter for the 10 MHz-Ref.-Osc. */
					gfx_mono_generic_draw_rect(0, 0, 240, 128, GFX_PIXEL_SET);
					const char buf[] = "<==== 10 MHz.-Ref.-Osc. Smart-LCD ====>";
					gfx_mono_draw_string(buf, 3, 2, lcd_get_sysfont());
					lcd_show_template();
				}

				flags = cpu_irq_save();
				g_status.isAnimationStopped = true;
				cpu_irq_restore(flags);
			}
		}
	} while (more);
}

void enter_sleep(uint8_t sleep_mode)
{
	SMCR  = (sleep_mode << SM0)
		  | _BV(SE);												// Enable sleep command

	__asm__ __volatile__ ("sleep" ::: "memory");

	SMCR &= ~(_BV(SE));												// Disable sleep command
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
	sysclk_init();	PRR = 0b11101011;								// For debugging this module has to be powered on, again
	ioport_init();
	s_adc_init();
	s_tc_init();

	/* I/O pins go active here */
	board_init();

	reset_cause_t rc = reset_cause_get_causes();
	g_resetCause = rc;
	if (rc & CHIP_RESET_CAUSE_EXTRST	||
		rc & CHIP_RESET_CAUSE_BOD_CPU	||
		rc & CHIP_RESET_CAUSE_POR		||
		!rc) {
		s_reset_global_vars();
	} else {
		/* DEBUG */
		asm_break();
	}

	/* Read non-volatile settings */
	eeprom_nvm_settings_read(C_EEPROM_NVM_SETTING_ALL);				// Load all entries from NVM

	/* I2C interface - 10 MHz-Ref-Osc. second display */
	s_twi_init(TWI_SLAVE_ADDR_SMARTLCD, TWI_SLAVE_ADDR_BM);

	/* All interrupt sources prepared here - IRQ activation */
	cpu_irq_enable();

	/* Start of sub-modules */
	s_tc_start();													// All clocks and PWM timers start here

	/* Initialize external components */
	lcd_init();
	lcd_test(0b11111101);											// Debugging purposes


	/* Main loop */
	runmode = 1;
    while (runmode) {
	    task(get_abs_time());
	    enter_sleep(SLEEP_MODE_IDLE);
    }


	/* Shutdown external components */
	lcd_shutdown();

	cpu_irq_disable();

	/* Disable sub-modules */
	ACSR |= _BV(ACD);												// Disable AnalogCompare sub-module

	sysclk_disable_module(POWER_RED_REG0, PRSPI_bm);
	sysclk_disable_module(POWER_RED_REG0, PRUSART0_bm);

	s_twi_disable();
	s_adc_disable();
	s_tc_disable();

    enter_sleep(SLEEP_MODE_PWR_DOWN);

    return retcode;													// Should never be reached
}
