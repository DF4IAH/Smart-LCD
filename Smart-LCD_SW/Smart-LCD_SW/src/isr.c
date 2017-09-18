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

#include "main.h"
#include "twi.h"
#include "lcd.h"

#include "isr.h"


/* External vars */

extern status_t				g_status;
extern showData_t			g_showData;
extern uint8_t				g_SmartLCD_mode;
extern uint32_t				g_timer_abs_100us;
extern uint8_t				g_adc_state;
extern float				g_adc_light;
extern float				g_adc_temp;
extern uint8_t				g_lcd_contrast_pm;
extern bool					g_audio_out_on;
extern uint8_t				g_audio_out_mod;
extern uint8_t				g_audio_out_length;
extern int16_t				g_audio_pwm_accu;
extern uint8_t				g_audio_pwm_ramp_dwn;
extern bool					g_led_red;
extern bool					g_led_green;


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


ISR(__vector_1, ISR_BLOCK)											// Variants: ISR_BLOCK, ISR_NOBLOCK, ISR_NAKED
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
	uint8_t pinb = PINB;
	uint8_t pinc = PINC;

	ioport_set_pin_level(LED_RED_GPIO,   g_led_red   ?  IOPORT_PIN_LEVEL_HIGH : IOPORT_PIN_LEVEL_LOW);
	ioport_set_pin_level(LED_GREEN_GPIO, g_led_green ?  IOPORT_PIN_LEVEL_HIGH : IOPORT_PIN_LEVEL_LOW);

	/* BOTTOM */
	TIFR2 |= _BV(TOV2);
	cpu_irq_enable();

	if (g_status.isAnimationStopped && (g_SmartLCD_mode == C_SMART_LCD_MODE_REFOSC)) {
		static uint8_t state_old = 0;
		static uint8_t second_old = 0;
		static uint8_t button_ctr = 0;

		/* Signaling the grade of deviation */
		g_audio_out_on = false;
		if (g_showData.clkState_clk_state < 0xf) {
			g_led_red		= true;									// LED = red
			g_led_green		= false;
			if (state_old  != 0x02) {
				g_audio_out_length = 122;							// 1 sec
			}
			state_old = 0x02;

		} else if ((g_showData.clkState_clk_state == 0xf) && (-4 < g_showData.ppb_int) && (g_showData.ppb_int < 4)) {
			g_led_green		= true;									// LED = green
			g_led_red		= false;
			state_old		= 0x00;

			/* Acoustic phase tracker */
			if (g_showData.time_second != second_old) {
				second_old = g_showData.time_second;
				g_audio_out_length = 6;
			}

		} else {
			if (!state_old) {
				g_audio_out_length = 30;							// 1/4 sec
				g_led_red = g_led_green = false;
			}
			state_old = 0x01;
		}

		/* Sampling I/Q and push buttons - de-noise */
		if (button_ctr) {
			--button_ctr;

		} else {
			button_ctr = 12;
			bool sw_b = !(pinb & _BV(PINB2));
			bool sw_i = !(pinc & _BV(PINC1));
			bool sw_q = !(pinc & _BV(PINC2));
			if (sw_i) {												// SW-I: decrement contrast voltage
				if (g_lcd_contrast_pm) {
					--g_lcd_contrast_pm;
					lcd_contrast_update();
				}
			} else if (sw_q) {										// SW-Q: increment contrast voltage
				if (g_lcd_contrast_pm < 0x3f) {
					++g_lcd_contrast_pm;
					lcd_contrast_update();
				}
			}

			if (sw_b) {												// Pushbutton: store value in EEPROM
				eeprom_nvm_settings_write(C_EEPROM_NVM_SETTING_LCD_CONTRAST);
			}
		}
	}

	/* Beep length enables audio output */
	if (g_audio_out_length) {
		--g_audio_out_length;
		g_audio_out_on = true;
	} else {
		g_audio_out_on = false;
	}
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
{	/* TIMER 1 OVF - Overflow (frequency = 10 kHz) */
	static uint8_t	s_audio_pwm_mod		= 0;
	static bool		s_audio_toggle		= false;

	/* 100µs counter */
	++g_timer_abs_100us;

	/* Audio output */
	if (s_audio_pwm_mod) {
		if (!(g_timer_abs_100us % s_audio_pwm_mod)) {
			s_audio_toggle = !s_audio_toggle;
			ioport_set_pin_level(OC1A_GPIO, s_audio_toggle);
		}
	}

#if 0
	/* BOTTOM */
	TIFR1 |= _BV(TOV1);
	cpu_irq_enable();
#endif

	/* Preparation for next cycle */
	if (g_status.isAnimationStopped && g_audio_out_on) {
		if (g_SmartLCD_mode == C_SMART_LCD_MODE_REFOSC) {
			int16_t pitch = g_showData.clkState_phaseDeg100 >> 9;	// One pitch step each 5 deg
			if (pitch < -4) {
				pitch = -4;
			} else if (pitch > 3) {
				pitch = 3;
			}
			s_audio_pwm_mod = (uint8_t) (5 - pitch);
		} else {
			s_audio_pwm_mod = g_audio_out_mod;
		}
	} else {
		/* Turn off sound */
		s_audio_pwm_mod = 0;
	}

#if 0
	/* Start A/D convertion by entering ADC sleep-mode */
	while ((ADCSRA & (1 << ADSC))) {
		/* wait for conversion complete */
	}
	ADCSRA |= _BV(ADIF);											// Clear interrupt status bit by setting it to clear
	adc_enable_interrupt();											// Enable the ADC interrupt
	cpu_irq_enable();
	//adc_start_conversion();										// TODO ???
	enter_sleep(SLEEP_MODE_ADC);
	adc_disable_interrupt();
#endif
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
#if 0
	/* Start A/D convertion by entering ADC sleep-mode */
	while ((ADCSRA & (1 << ADSC))) {
		/* wait for conversion complete */
	}
	ADCSRA |= _BV(ADIF);											// Clear interrupt status bit by setting it to clear
	adc_enable_interrupt();											// Enable the ADC interrupt
	cpu_irq_enable();

	enter_sleep(SLEEP_MODE_ADC);
	adc_disable_interrupt();
#endif
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
	uint16_t adc_val = ADCL | (ADCH << 8);
	uint8_t  reason  = g_adc_state;

	//TIFR1 |= _BV(TOV1);											// Reset Timer1 overflow status bit (when no ISR for TOV1 activated!)

	switch (reason) {
		case ADC_STATE_PRE_LDR:
			// drop one ADC value after switching MUX
			g_adc_state = ADC_STATE_VLD_LDR;
		break;

		case ADC_STATE_VLD_LDR:
		{
			/* Low pass filtering and enhancing the data depth */
			float l_adc_light = g_adc_light;
			cpu_irq_enable();
			float calc = l_adc_light ?  0.980f * l_adc_light + 0.020f * adc_val : adc_val;	// load with initial value if none is set before
			cpu_irq_disable();
			g_adc_light = calc;

			adc_set_admux(ADC_MUX_TEMPSENSE | ADC_VREF_1V1 | ADC_ADJUSTMENT_RIGHT);
			g_adc_state = ADC_STATE_PRE_TEMP;
		}
		break;

		case ADC_STATE_PRE_TEMP:
			/* Drop one ADC value after switching MUX */
			g_adc_state = ADC_STATE_VLD_TEMP;
		break;

		case ADC_STATE_VLD_TEMP:
		{
			/* Low pass filtering and enhancing the data depth */
			float l_adc_temp  = g_adc_temp;
			cpu_irq_enable();
			float calc = l_adc_temp ?  0.998f * l_adc_temp  + 0.002f * adc_val : adc_val;	// Load with initial value if none is set before
			cpu_irq_disable();
			g_adc_temp = calc;
		}
			// fall-through.
		default:
			adc_set_admux(ADC_MUX_ADC0 | ADC_VREF_1V1 | ADC_ADJUSTMENT_RIGHT);
			g_adc_state = ADC_STATE_PRE_LDR;
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
	uint8_t tws = TWSR & (0b11111 << TWS3);
	uint8_t twd = TWDR;
	uint8_t twcr_cur = TWCR;

	uint8_t twcr_new = __vector_24__bottom(tws, twd, twcr_cur);
	TWCR = twcr_new | _BV(TWINT) | _BV(TWEN) | _BV(TWIE);
}

ISR(__vector_25, ISR_BLOCK)
{	/* SPM READY - Store Program Memory Ready */
	s_bad_interrupt();
}
