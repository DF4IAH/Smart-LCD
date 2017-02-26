/**
 * \file
 *
 * \brief User board initialization template
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <asf.h>
#include <board.h>
#include <conf_board.h>

void board_init(void)
{
#if 0
	/* all PIN settings already done in s_io_preinit(void) in main.c */

	/* PWM */
	ioport_set_pin_dir(AUDIO_PWM, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(AUDIO_PWM, IOPORT_MODE_PULLDOWN);

	ioport_set_pin_dir(LCDBL_PWM, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LCDBL_PWM, false);


	/* LCD interface */
	ioport_set_pin_dir(LCD_CD, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LCD_CD, false);

	ioport_set_pin_dir(LCD_RW, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LCD_RW, true);
	
	ioport_set_pin_dir(LCD_EN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LCD_EN, false);

	ioport_set_pin_dir(LCD_CS, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LCD_CS, false);

	ioport_set_pin_dir(LCD_RST_N, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(LCD_RST_N, IOPORT_MODE_PULLUP);
	
	ioport_set_pin_dir(LCD_D0, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(LCD_D0, IOPORT_MODE_PULLDOWN);
	
	ioport_set_pin_dir(LCD_D1, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(LCD_D1, IOPORT_MODE_PULLDOWN);
	
	ioport_set_pin_dir(LCD_D2, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(LCD_D2, IOPORT_MODE_PULLDOWN);
	
	ioport_set_pin_dir(LCD_D3, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(LCD_D3, IOPORT_MODE_PULLDOWN);
	
	ioport_set_pin_dir(LCD_D4, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(LCD_D4, IOPORT_MODE_PULLDOWN);
	
	ioport_set_pin_dir(LCD_D5, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(LCD_D5, IOPORT_MODE_PULLDOWN);
	
	ioport_set_pin_dir(LCD_D6, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(LCD_D6, IOPORT_MODE_PULLDOWN);
	
	ioport_set_pin_dir(LCD_D7, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(LCD_D7, IOPORT_MODE_PULLDOWN);
	

	/* Status LEDs */
	ioport_set_pin_dir(LED_RD, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED_RD, false);

	ioport_set_pin_dir(LED_GN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(LED_GN, false);


	/* Contacts: Knob and Push button */
	ioport_set_pin_dir(CNTCT_P, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(CNTCT_P, IOPORT_MODE_PULLDOWN);

	ioport_set_pin_dir(CNTCT_I, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(CNTCT_I, IOPORT_MODE_PULLDOWN);

	ioport_set_pin_dir(CNTCT_Q, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(CNTCT_Q, IOPORT_MODE_PULLDOWN);


	/* ADC: LDR ambient light detection */
	ioport_set_pin_dir(LDR_ADC, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(LDR_ADC, IOPORT_MODE_PULLDOWN);


	/* I2C / TWI interface */
	ioport_set_pin_dir(SDA_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(SDA_GPIO, IOPORT_MODE_PULLUP);

	ioport_set_pin_dir(SCL_GPIO, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(SCL_GPIO, IOPORT_MODE_PULLUP);
#endif
}
