/**
 * \file
 *
 * \brief TWI / I2C communication module
 *
 */

/**
 * \mainpage TWI / I2C
 *
 * \par TWI / I2C communication module
 *
 * The TWI is compatible to the I2C bus and allows communication on a two-wire
 * bi-directional bus.
 *
 * This module allows to variants: the application is a Slave on the bus for
 * working out any commands given to it. On the other hand this application needs
 * to address a digital pot to steer the LCD contrast on the display. For this
 * to work the application is the Master for the Pot device at the bus.
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
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

#include "lcd.h"
#include "main.h"

#include "twi.h"


extern uint8_t				g_animation_on;

static uint8_t				s_tx_next_len = 0;
static uint8_t				s_tx_next_d[8];
static uint8_t				s_tx_lock = 0;
static uint8_t				s_tx_len = 0;
static uint8_t				s_tx_d[8];

static uint8_t				s_rx_lock = 0;
static uint8_t				s_rx_d[8];
static uint8_t				s_rx_len = 0;


static void s_twi_tx_prepare(uint8_t msgCnt, uint8_t msg[])
{
	if (msgCnt && msg) {
		if (!s_tx_lock && !s_tx_next_len) {
			// Prepare master message buffer
			for (int idx = msgCnt; idx >= 0; --idx) {
				s_tx_d[idx] = msg[idx];
			}
			s_tx_len = msgCnt;

		} else if (s_tx_lock && !s_tx_next_len) {
			// Stash message into next message facility
			for (int idx = msgCnt; idx >= 0; --idx) {
				s_tx_next_d[idx] = msg[idx];
			}
			s_tx_next_len = msgCnt;
		} // else ... the message is lost
	}
}

static void s_twi_tx_done(void)
{
	// Called when  s_tx_lock == 0
	if (s_tx_next_len) {
		// Load	next master message
		for (int idx = s_tx_next_len; idx >= 0; --idx) {
			s_tx_d[idx] = s_tx_next_d[idx];
		}
		s_tx_len = s_tx_next_len;
		s_tx_next_len = 0;
	}
}

static void s_twi_rx_prepare(uint8_t msgCnt, uint8_t msg[])
{
	if (msgCnt && msg) {
		if (!s_rx_lock) {
			// Prepare master message buffer
			for (int idx = msgCnt; idx >= 0; --idx) {
				s_rx_d[idx] = msg[idx];
			}
			s_rx_len = msgCnt;
		}
	}
}

static uint8_t s_twi_rcvd_command_open_form(uint8_t data[], uint8_t pos)
{
	uint8_t err = 1;
	// TODO: implementation
	return err;
}

static void s_twi_rcvd_command_closed_form(uint8_t data[], uint8_t cnt)
{
	uint8_t isGCA	= !data[0];
	uint8_t cmd		=  data[1];

	if (isGCA) {
		nop();
		switch (cmd) {
			case 0b0100000:						// IDENTIFY
			// TODO: prepare ADR+R data
			break;

			default:
			{
				// do nothing
			}
		}

	} else {
		uint8_t buf[4];

		nop();
		switch (cmd) {
			case TWI_SMART_LCD_CMD_GETVER:
			buf[0] = VERSION;
			s_twi_rx_prepare(1, buf);
			g_animation_on = false;				// stop animation demo
			break;

			case TWI_SMART_LCD_CMD_SHOW_YEAR_MON_DAY:
			lcd_10mhz_ref_osc_show_date(data[2] | (data[3] << 8), data[4], data[5]);
			break;


			case 0b1000000:						// LCD reset
			// TODO: LCD communication
			break;

			case 0b1000001:						// blank screen
			// TODO: LCD communication
			break;

			case 0b1000010:						// invert off
			// TODO: LCD communication
			break;

			case 0b1000011:						// invert on
			// TODO: LCD communication
			break;

			default:
			{
				// do nothing
			}
		}
	}
}


uint8_t __vector_24__bottom(uint8_t tws, uint8_t twd, uint8_t twcr_cur)
{
	static uint8_t pos_i	= 0;
	static uint8_t pos_o	= 0;
	static uint8_t cnt_i	= 0;
	static uint8_t cnt_o	= 0;
	uint8_t twcr_new = twcr_cur;

	switch(tws) {

	/* Master Transmitter Mode */

	case 0x08:									// Start condition transmitted
		s_tx_lock = 1;
		pos_o = 0;

		cnt_o = 2;								// TEST
		s_tx_d[0] = (0x12 << TWD1) | (0b0 << TWD0);	// TEST
		s_tx_d[1] = 0x34;						// TEST
		s_tx_d[2] = 0x56;						// TEST
		s_tx_d[3] = 0x78;						// TEST
		s_tx_d[4] = 0x9a;						// TEST

		//twcr_new &= ~_BV(TWSTA);	// TODO: self-clearing?
		// fall-through.
	case 0x10:									// Repeated start condition transmitted
		nop();
		// fall-through.
	case 0x18:									// SLA+W transmitted and ACK received
		TWDR = s_tx_d[pos_o++];
		break;

	case 0x20:									// SLA+W transmitted and NACK received
		twcr_new |= _BV(TWSTO);					// Send STOP
		break;

	case 0x28:									// Data byte sent and ACK received
		if (pos_o < cnt_o) {
			TWDR = s_tx_d[pos_o++];				// Send new data byte
		} else {
			twcr_new |= _BV(TWSTO);				// Send STOP - no more data available
			s_tx_lock = 0;
			s_twi_tx_done();					// Message sent
		}
		break;

	case 0x30:									// Data byte sent and NACK received
		twcr_new |= _BV(TWSTO);					// Send STOP - due to an error or slave not ready situation
		s_tx_lock = 0;
		s_twi_tx_done();						// Message failure
		break;

	case 0x38:									// Arbitration lost
		twcr_new |= _BV(TWSTA);					// Send START (again)
		break;


	/* Slave Receiver Mode */

	case 0x60:									// SLA+W received and ACK sent
		nop();
		// fall-through.
	case 0x68:
		s_rx_lock = 1;
		mem_set(s_rx_d, 8, 0x00);
		s_rx_d[0] = twd;						// Target address
		pos_i = 1;								// Starting of reception
		twcr_new |= _BV(TWEA);					// Send after next coming data byte ACK
		break;

	case 0x70:									// GCA received and ACK sent
		nop();
		// fall-through.
	case 0x78:
		s_rx_lock = 1;
		s_rx_d[0] = twd;						// GCA
		pos_i = 1;								// Starting of reception
		twcr_new |= _BV(TWEA);					// Send after next coming data byte ACK
		break;

	case 0x80:									// Data after SLA+W received
		nop();
		// fall-through.
	case 0x90:
		if (cnt_i != 0b111) {					// Closed parameter form
			if (pos_i <= 0b111) {
				s_rx_d[pos_i] = twd;
			}
			if (pos_i == 1) {
				//cnt_i = ((twd >> 5) & 0b111) + 1;
				switch (s_rx_d[1]) {
#if 0
					case 0x12:
						cnt_i = 2;
						break;
#endif

					default:
						cnt_i = 3;
				}
			}
			if (pos_i < 0b111) {
				++pos_i;
			}

			if (pos_i <= cnt_i) {
				twcr_new |= _BV(TWEA);			// Send after next coming data byte ACK
			} else {
				//twcr_new &= ~_BV(TWEA);			// Send after next coming data byte NACK
				twcr_new |= _BV(TWEA); // TEST
			}

		} else {								// Open parameter form
			s_rx_d[2] = twd;
			if (!s_twi_rcvd_command_open_form(s_rx_d, ++pos_i)) {
				twcr_new |= _BV(TWEA);			// Send after next coming data byte ACK
			} else {
				twcr_new &= ~_BV(TWEA);			// Send after next coming data byte NACK
				pos_i = 0;
			}
		}
		break;

	case 0x88:									// NACK after last data byte sent
		nop();
		// fall-through.
	case 0x98:
		s_rx_lock = 0;
		if (cnt_i != 0b111) {
			s_twi_rcvd_command_closed_form(s_rx_d, pos_i);	// Call interpreter for closed form of parameters
		} else {
			s_twi_rcvd_command_open_form(s_rx_d, ++pos_i);	// Call interpreter for open form of parameters
		}
		pos_i = 0;
		cnt_i = 0;
		mem_set(s_rx_d, 8, 0x00);
		s_rx_lock = 0;
		twcr_new |= _BV(TWEA);					// TWI goes to unaddressed, be active again
		break;

	case 0xA0:									// STOP or RESTART received while still addressed as slave
		s_rx_lock = 0;
		if (cnt_i != 0b111) {
			s_twi_rcvd_command_closed_form(s_rx_d, pos_i);	// Call interpreter for closed form of parameters
		} else {
			s_twi_rcvd_command_open_form(s_rx_d, ++pos_i);	// Call interpreter for open form of parameters
		}
		twcr_new |= _BV(TWEA);					// TWI goes to unaddressed, be active again
		pos_i = 0;
		break;


	/* Slave Transmitter Mode */

	case 0xA8:									// SLA+R received and ACK has been returned
		nop();
		// fall-through.
	case 0xB0:
		s_rx_lock = 1;
		pos_o = 0;
		cnt_o = s_rx_len;
		TWDR = cnt_o > pos_o ?  s_rx_d[pos_o++] : 0;

		if (cnt_o > pos_o) {
			twcr_new |= _BV(TWEA);				// More data to send ACK
		} else {
			twcr_new &= ~_BV(TWEA);				// No more data to send NACK
		}
		break;

	case 0xB8:									// Data sent and ACK has been returned
		TWDR = cnt_o > pos_o ?  s_rx_d[pos_o++] : 0;
		if (cnt_o > pos_o) {
			twcr_new |= _BV(TWEA);				// More data to send ACK
			} else {
			twcr_new &= ~_BV(TWEA);				// No more data to send NACK
		}
		break;

	case 0xC0:									// Data sent and NACK has been returned
		twcr_new |= _BV(TWEA);					// TWI goes to unaddressed, be active again
		pos_o = 0;
		cnt_o = 0;
		s_rx_lock = 0;
		break;

	case 0xC8:									// Last data sent and ACK has been returned
		twcr_new |= _BV(TWEA);					// TWI goes to unaddressed, be active again
		s_rx_lock = 0;
		/* message transmitted successfully in slave mode */
		break;
		
	default:
		nop();
	}

	return twcr_new;
}
