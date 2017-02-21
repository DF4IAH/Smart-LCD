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

#include "twi.h"


static uint8_t s_tx_next_len = 0;
static uint8_t s_tx_next_d[8];
static uint8_t s_tx_lock = 0;
static uint8_t s_tx_len = 0;
static uint8_t s_tx_d[8];

static uint8_t s_rx_lock = 0;
static uint8_t s_rx_d[8];


static void s_twcr_ack(uint8_t set)
{
	irqflags_t flags = cpu_irq_save();

	if (set) {
		TWCR |=   _BV(TWEA);					// ACK
	} else {
		TWCR &= ~(_BV(TWEA));					// NACK
	}

	cpu_irq_restore(flags);
}

static void s_twcr_all(uint8_t ctrl)
{
	irqflags_t flags = cpu_irq_save();

	TWCR = ctrl;

	cpu_irq_restore(flags);
}

static void s_twdr(uint8_t data_o)
{
	irqflags_t flags = cpu_irq_save();

	TWDR = data_o;

	cpu_irq_restore(flags);
}


static void s_twi_tx_prepare(uint8_t msgCnt, uint8_t msg[])
{
	if (msgCnt && msg) {
		if (!s_tx_lock && !s_tx_next_len) {
			// Prepare master message buffer
			for (int idx = msgCnt; idx >= 0; --idx) {
				s_tx_d[idx] = msg[idx];
			}
			s_tx_len = msgCnt;
			s_twdr(0b11100101);					// Start condition

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
		s_twdr(0b11100101);						// Start condition
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
		switch (cmd) {
			case 0b0000000:						// LCD reset
			// TODO: LCD communication
			break;
			
			case 0b0000001:						// blank screen
			// TODO: LCD communication
			break;
			
			case 0b0000010:						// invert off
			// TODO: LCD communication
			break;
			
			case 0b0000011:						// invert on
			// TODO: LCD communication
			break;
			
			default:
			{
				// do nothing
			}
		}
	}
}


void __vector_24__bottom(uint8_t tws, uint8_t twd)
{
	static uint8_t pos_i	= 0;
	static uint8_t pos_o	= 0;
	static uint8_t cnt_i	= 0;
	static uint8_t cnt_o	= 0;
	
	switch(tws) {

	/* Master Transmitter Mode */
	
	case 0x08:									// Start condition transmitted
		s_tx_lock = 1;
		pos_o = 0;
	case 0x10:									// Repeated start condition transmitted
	case 0x18:									// SLA+W transmitted and ACK received
		s_twdr(s_tx_d[pos_o++]);
		break;
		
	case 0x20:									// SLA+W transmitted and NACK received
		s_twcr_all(0b10010101);					// Send NACK and STOP
		break;
		
	case 0x28:									// Data byte sent and ACK received
		if (pos_o < cnt_o) {
			s_twdr(s_tx_d[pos_o++]);
			s_twcr_all(0b11000101);				// Send new data byte and ACK send enable
		} else {
			s_twcr_all(0b11010101);				// Send STOP and ACK send enable
			s_tx_lock = 0;
			s_twi_tx_done();					// Message sent
		}
		break;
	
	case 0x30:									// Data byte sent and NACK received
		s_twcr_all(0b10010101);					// Send NACK and STOP
		s_tx_lock = 0;
		s_twi_tx_done();						// Message failure
		break;
	
	case 0x38:									// Arbitration lost
		s_twcr_all(0b11100101);					// Send START (again) and ACK send enable
		break;
	
	
	/* Slave Receiver Mode */
	
	case 0x60:									// SLA+W received and ACK sent
	case 0x68:
		s_rx_lock = 1;
		s_rx_d[0] = twd;						// Target address
		pos_i = 1;								// Starting of reception
		break;
	
	case 0x70:									// GCA received and ACK sent
	case 0x78:
		s_rx_lock = 1;
		s_rx_d[0] = twd;						// GCA
		pos_i = 1;								// Starting of reception
		break;
	
	case 0x80:									// Data after SLA+W received
	case 0x90:
		if (cnt_i == 0b111) {					// Open parameter form
			s_rx_d[2] = twd;
			if (!s_twi_rcvd_command_open_form(s_rx_d, ++pos_i)) {
				s_twcr_ack(true);				// ACK
			} else {
				s_twcr_ack(false);				// NACK
				cnt_i = 0;
			}

		} else {								// Closed parameter form
			if (pos_i <= 0b111) {
				s_rx_d[pos_i] = twd;
			}
			if (pos_i == 1) {
				cnt_i = ((twd >> 5) & 0b111) + 1;
			}
			if (pos_i < 0b111) {
				++pos_i;
			}
			s_twcr_ack(pos_i <= cnt_i);			// ACK - NACK
		}
		break;
	
	case 0x88:									// NACK after last data byte sent
	case 0x98:
		if (cnt_i != 0b111) {
			s_twi_rcvd_command_closed_form(s_rx_d, pos_i);	// Call interpreter for closed form of parameters
		} else {
			s_twi_rcvd_command_open_form(s_rx_d, ++pos_i);	// Call interpreter for open form of parameters
		}
		s_rx_lock = 0;
		break;
	
	case 0xA0:
		s_twcr_all(0b11000101);					// Send nothing
		pos_i = 0;
		cnt_i = 0;
		s_rx_lock = 0;
		break;
	
	
	/* Slave Transmitter Mode */
	
	case 0xA8:									// SLA+R received and ACK has been returned
	case 0xB0:
		s_rx_lock = 1;
		pos_o = 0;
		s_twdr(cnt_o > pos_o ?  s_rx_d[pos_o++] : 0);
		s_twcr_ack(cnt_o > pos_o);				// ACK - NACK
		break;
	
	case 0xB8:									// Data sent and ACK has been returned
		s_twdr(cnt_o > pos_o ?  s_rx_d[pos_o++] : 0);
		s_twcr_ack(cnt_o > pos_o);				// ACK - NACK
		break;
	
	case 0xC0:									// Data sent and NACK has been returned
		s_twcr_ack(false);						// NACK
		pos_o = 0;
		cnt_o = 0;
		s_rx_lock = 0;
		break;
	
	case 0xC8:									// Superfluous ACK by master sent after NACK has been returned
		s_twcr_all(0b11000101);					// Send nothing
		s_rx_lock = 0;
		break;
	}
}
