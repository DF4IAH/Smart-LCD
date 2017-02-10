/**
 * \file
 *
 * \brief User board definition template
 *
 */

 /* This file is intended to contain definitions and configuration details for
 * features and devices that are available on the board, e.g., frequency and
 * startup time for an external crystal, external memory devices, LED and USART
 * pins.
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef USER_BOARD_H
#define USER_BOARD_H

#include <conf_board.h>

/**
 * \name Audio output PWM
 *
 * With the help of this 10-bit PWM output (counter value max 1023) and a following
 * low pass filter of 15 kHz this results to a 10 bit DAC for audio purposes.
 */
//@{
#define AUDIO_PWM_GPIO                  IOPORT_CREATE_PIN(PORTB, 1)
#define AUDIO_PWM                       AUDIO_PWM_GPIO
//@}

/**
 * \name LCD backlight controlled by PWM
 *
 * The backlight of the LCD panel is pulsed by a PWM signal with a frequency of
 * nearly 244 Hz. By the PWM percentage the light is dimmed, also.
 */
//@{
#define LCDBL_PWM_GPIO                  IOPORT_CREATE_PIN(PORTB, 3)
#define LCDBL_PWM                       LCDBL_PWM_GPIO
//@}


/**
 * \name LCD 8-bit parallel mode
 *
 * The MIDAS MCCOG240128A6W-FPTLW is connected to these wires.
 */
//@{
#define LCD_CD_GPIO                     IOPORT_CREATE_PIN(PORTB, 0)
#define LCD_WR0_GPIO                    IOPORT_CREATE_PIN(PORTB, 4)
#define LCD_WR1_GPIO                    IOPORT_CREATE_PIN(PORTB, 5)
#define LCD_CS_GPIO                     IOPORT_CREATE_PIN(PORTC, 3)
#define LCD_RST_N_GPIO                  IOPORT_CREATE_PIN(PORTC, 6)
#define LCD_D0_GPIO                     IOPORT_CREATE_PIN(PORTD, 0)
#define LCD_D1_GPIO                     IOPORT_CREATE_PIN(PORTD, 1)
#define LCD_D2_GPIO                     IOPORT_CREATE_PIN(PORTD, 2)
#define LCD_D3_GPIO                     IOPORT_CREATE_PIN(PORTD, 3)
#define LCD_D4_GPIO                     IOPORT_CREATE_PIN(PORTD, 4)
#define LCD_D5_GPIO                     IOPORT_CREATE_PIN(PORTD, 5)
#define LCD_D6_GPIO                     IOPORT_CREATE_PIN(PORTD, 6)
#define LCD_D7_GPIO                     IOPORT_CREATE_PIN(PORTD, 7)

#define LCD_CD                          LCD_CD_GPIO
#define LCD_WR0                         LCD_WR0_GPIO
#define LCD_RW							LCD_WR0_GPIO
#define LCD_WR1                         LCD_WR1_GPIO
#define LCD_EN                          LCD_WR1_GPIO
#define LCD_CS                          LCD_CS_GPIO
#define LCD_RST_N                       LCD_RST_N_GPIO
#define RESET_N                         LCD_RST_N_GPIO
#define LCD_D0                          LCD_D0_GPIO
#define LCD_D1                          LCD_D1_GPIO
#define LCD_D2                          LCD_D2_GPIO
#define LCD_D3                          LCD_D3_GPIO
#define LCD_D4                          LCD_D4_GPIO
#define LCD_D5                          LCD_D5_GPIO
#define LCD_D6                          LCD_D6_GPIO
#define LCD_D7                          LCD_D7_GPIO
//@}


/**
 * \name Status Duo-LED
 *
 * The red/green status LED is driven by that output wires.
 */
//@{
#define LED_RD_GPIO                     IOPORT_CREATE_PIN(PORTB, 6)
#define LED_GN_GPIO                     IOPORT_CREATE_PIN(PORTB, 7)

#define LED_RD                          LED_RD_GPIO
#define LED_GN                          LED_GN_GPIO
//@}


/**
 * \name Contacts of a turnable knob with its I/Q switches and another button contact are connected like this.
 */
//@{
#define CNTCT_P_GPIO                    IOPORT_CREATE_PIN(PORTB, 2)
#define CNTCT_I_GPIO                    IOPORT_CREATE_PIN(PORTC, 1)
#define CNTCT_Q_GPIO                    IOPORT_CREATE_PIN(PORTC, 2)

#define CNTCT_P                         CNTCT_P_GPIO
#define CNTCT_I                         CNTCT_I_GPIO
#define CNTCT_Q                         CNTCT_Q_GPIO
//@}


/**
 * \name LDR ambient light detection.
 *
 * A light sensitive resistor is connected on the ADC input.
 */
//@{
#define LDR_ADC_GPIO                    IOPORT_CREATE_PIN(PORTC, 0)

#define LDR_ADC                         LDR_ADC_GPIO
//@}


/**
 * \name I2C
 *
 * I2C bus lines.
 */
//@{
//#define I2C_TWI                      &TWI
#define I2C_USART_SCL                  &USART
#define SDA_GPIO                       IOPORT_CREATE_PIN(PORTC, 4)
#define SCL_GPIO                       IOPORT_CREATE_PIN(PORTC, 5)
//@}



// External oscillator settings.
// Uncomment and set correct values if external oscillator is used.

// External oscillator frequency
//#define BOARD_XOSC_HZ          8000000

// External oscillator type.
//!< External clock signal
//#define BOARD_XOSC_TYPE        XOSC_TYPE_EXTERNAL
//!< 32.768 kHz resonator on TOSC
//#define BOARD_XOSC_TYPE        XOSC_TYPE_32KHZ
//!< 0.4 to 16 MHz resonator on XTALS
//#define BOARD_XOSC_TYPE        XOSC_TYPE_XTAL

// External oscillator startup time
//#define BOARD_XOSC_STARTUP_US  500000


#endif // USER_BOARD_H
