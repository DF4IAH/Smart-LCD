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

#include "gfx_mono/gfx_mono.h"
#include "lcd.h"
#include "main.h"

#include "twi.h"


extern status_t				g_status;
extern uint8_t				g_SmartLCD_mode;
extern gfx_mono_color_t		g_lcd_pixel_type;
extern gfx_coord_t			g_lcd_pencil_x;
extern gfx_coord_t			g_lcd_pencil_y;
extern char					g_strbuf[8];

static uint8_t				s_tx_next_len = 0;
static uint8_t				s_tx_next_d[8];
static uint8_t				s_tx_lock = 0;
static uint8_t				s_tx_len = 0;
static uint8_t				s_tx_d[8];

static uint8_t				s_rx_lock = 0;
static uint8_t				s_rx_d[8];
static uint8_t				s_rx_len = 0;


#if 0
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
#endif

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

static uint8_t s_isr_twi_rcvd_command_open_form(uint8_t data[], uint8_t pos)
{
	uint8_t err = 1;
	// TODO: implementation
	return err;
}

static void s_isr_twi_rcvd_command_closed_form(uint8_t data[], uint8_t cnt)
{
	uint8_t prepareBuf[4] = { 0 };
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

	} else if (data[0] == TWI_SLAVE_ADDR_SMARTLCD) {
		/* unique command section for all modes */
		switch (cmd) {
			case TWI_SMART_LCD_CMD_GET_VER:
				prepareBuf[0] = VERSION;
				s_twi_rx_prepare(1, prepareBuf);
			break;

			case TWI_SMART_LCD_CMD_SET_MODE:
				isr_lcd_set_mode(data[2]);
			break;

			default:
			{
				// do nothing
			}
		}

	} else if ((data[0] == TWI_SLAVE_ADDR_SMARTLCD) && (g_SmartLCD_mode == 0x10)) {
		if (!(g_status.isAnimationStopped)) {
			return;
		}

		switch (cmd) {
			case TWI_SMART_LCD_CMD_CLS:					// Clear screen
				lcd_cls();
			break;

			case TWI_SMART_LCD_CMD_SET_PIXEL_TYPE:		// Set next pixels (OFF / ON / INVERTED)
				g_lcd_pixel_type = data[2];
			break;

			case TWI_SMART_LCD_CMD_SET_POS_X_Y:			// Set pencil position (x, y)
				g_lcd_pencil_x = data[2];
				g_lcd_pencil_y = data[3];
			break;

			case TWI_SMART_LCD_CMD_WRITE:				// Write text of length (length, buffer...)
			{
				int length = data[2];
				if (length <= 6) {
					int i;
					for (i = 0; i < length; ++i) {
						g_strbuf[i] = data[3 + i];
					}
					g_strbuf[length] = 0;
					isr_lcd_write(g_strbuf);
				}
			}
			break;

			case TWI_SMART_LCD_CMD_DRAW_LINE:			// Draw line from current pencil position to next position (x, y)
				gfx_mono_generic_draw_line(g_lcd_pencil_x, g_lcd_pencil_y, data[2], data[3], g_lcd_pixel_type);
			break;

			case TWI_SMART_LCD_CMD_DRAW_RECT:			// Draw rectangular frame with pencil's start position with dimension (width, height)
				gfx_mono_generic_draw_rect(g_lcd_pencil_x, g_lcd_pencil_y, data[2], data[3], g_lcd_pixel_type);
			break;

			case TWI_SMART_LCD_CMD_DRAW_FILLED_RECT:	// Draw filled rectangular frame with pencil's start position with dimension (width, height)
				gfx_mono_generic_draw_filled_rect(g_lcd_pencil_x, g_lcd_pencil_y, data[2], data[3], g_lcd_pixel_type);
			break;

			case TWI_SMART_LCD_CMD_DRAW_CIRC:			// Draw circle or ellipse from the pencil's center point with (radius)
				gfx_mono_generic_draw_circle(g_lcd_pencil_x, g_lcd_pencil_y, data[2], g_lcd_pixel_type, GFX_QUADRANT0 | GFX_QUADRANT1 | GFX_QUADRANT2 | GFX_QUADRANT3);
			break;

			case TWI_SMART_LCD_CMD_DRAW_FILLED_CIRC:	// Draw filled circle or ellipse from the pencil's center point with (radius)
				gfx_mono_generic_draw_filled_circle(g_lcd_pencil_x, g_lcd_pencil_y, data[2], g_lcd_pixel_type, GFX_QUADRANT0 | GFX_QUADRANT1 | GFX_QUADRANT2 | GFX_QUADRANT3);
			break;

			default:
			{
				// do nothing
			}
		}

	} else if ((data[0] == TWI_SLAVE_ADDR_SMARTLCD) && (g_SmartLCD_mode == 0x20)) {
		if (!(g_status.isAnimationStopped)) {
			return;
		}

		switch (cmd) {
			case TWI_SMART_LCD_CMD_SHOW_CLK_STATE:
				isr_lcd_10mhz_ref_osc_show_clkstate_phaseVolt1000_phaseDeg100(data[2], (uint16_t) (data[3] | (data[4] << 8)), (int16_t) (data[5] | (data[6] << 8)));
			break;

			case TWI_SMART_LCD_CMD_SHOW_YEAR_MON_DAY:
				isr_lcd_10mhz_ref_osc_show_date(data[2] | (data[3] << 8), data[4], data[5]);
			break;

			case TWI_SMART_LCD_CMD_SHOW_HR_MIN_SEC:
				isr_lcd_10mhz_ref_osc_show_time(data[2], data[3], data[4]);
			break;

			case TWI_SMART_LCD_CMD_SHOW_PPB:
				isr_lcd_10mhz_ref_osc_show_ppm((int16_t) (data[2] | (data[3] << 8)), data[4] | (data[5] << 8));
			break;

			case TWI_SMART_LCD_CMD_SHOW_TCXO_PWM:
				isr_lcd_10mhz_ref_osc_show_pwm(data[2], data[3]);
			break;

			case TWI_SMART_LCD_CMD_SHOW_TCXO_VC:
				isr_lcd_10mhz_ref_osc_show_pv(data[2], data[3] | (data[4] << 8));
			break;

			case TWI_SMART_LCD_CMD_SHOW_SATS:
				isr_lcd_10mhz_ref_osc_show_sat_use(data[2], data[3], data[4]);
			break;

			case TWI_SMART_LCD_CMD_SHOW_DOP:
				isr_lcd_10mhz_ref_osc_show_sat_dop(data[2] | (data[3] << 8));
			break;

			case TWI_SMART_LCD_CMD_SHOW_POS_STATE:
				isr_lcd_10mhz_ref_osc_show_pos_state(data[2], data[3]);
			break;

			case TWI_SMART_LCD_CMD_SHOW_POS_LAT:
				isr_lcd_10mhz_ref_osc_show_pos_lat(data[2], data[3], data[4], data[5] | (data[6] << 8));
			break;

			case TWI_SMART_LCD_CMD_SHOW_POS_LON:
				isr_lcd_10mhz_ref_osc_show_pos_lon(data[2], data[3], data[4], data[5] | (data[6] << 8));
			break;

			case TWI_SMART_LCD_CMD_SHOW_POS_HEIGHT:
				isr_lcd_10mhz_ref_osc_show_pos_height((data[2] | (data[3] << 8)), data[4]);
			break;

			default:
			{
				// do nothing for unsupported commands
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
	uint8_t twcr_new = twcr_cur & 0b01000101;

	switch(tws) {

	/* Master Transmitter Mode */

	case TWI_TWSR_START:						// Start condition transmitted
		s_tx_lock = 1;
		pos_o = 0;

		cnt_o = 2;								// TEST
		s_tx_d[0] = (0x12 << TWD1) | (0b0 << TWD0);	// TEST
		s_tx_d[1] = 0x34;						// TEST
		s_tx_d[2] = 0x56;						// TEST
		s_tx_d[3] = 0x78;						// TEST
		s_tx_d[4] = 0x9a;						// TEST
		// fall-through.
	case TWI_TWSR_REPEATEDSTART:				// Repeated start condition transmitted
		nop();
		// fall-through.
	case TWI_TWSR_M_SLAW_ADDR_ACK:				// SLA+W transmitted and ACK received
		TWDR = s_tx_d[pos_o++];
	break;

	case TWI_TWSR_M_SLAW_ADDR_NACK:				// SLA+W transmitted and NACK received
		twcr_new |= _BV(TWSTO);					// Send STOP
	break;

	case TWI_TWSR_M_SLAW_DATA_ACK:				// Data byte sent and ACK received
		if (pos_o < cnt_o) {
			TWDR = s_tx_d[pos_o++];				// Send new data byte
		} else {
			s_tx_lock = 0;
			s_twi_tx_done();					// Message sent
			twcr_new |= _BV(TWSTO);				// Send STOP - no more data available
		}
	break;

	case TWI_TWSR_M_SLAW_DATA_NACK:				// Data byte sent and NACK received
		s_tx_lock = 0;
		s_twi_tx_done();						// Message failure
		twcr_new |= _BV(TWSTO);					// Send STOP - due to an error or slave not ready situation
	break;

	case TWI_TWSR_M_SLAW_ARBIT_LOST:			// Arbitration lost
		twcr_new |= _BV(TWSTA);					// Send START (again)
	break;

	case TWI_TWSR_M_SLAR_ADDR_ACK:
		nop();
	break;

	case TWI_TWSR_M_SLAR_ADDR_NACK:
		nop();
	break;

	case TWI_TWSR_M_SLAR_DATA_ACK:
		nop();
	break;

	case TWI_TWSR_M_SLAR_DATA_NACK:
		nop();
	break;


	/* Slave Receiver Mode */

	case TWI_TWSR_S_SLAW_MYADDR_RECEIVED:		// SLA+W received and ACK returned
		nop();
		// fall-through.
	case TWI_TWSR_S_SLAW_MYADDR_ARBIT_LOST:
		s_rx_lock = 1;
		mem_set(s_rx_d, 8, 0x00);
		s_rx_d[0] = twd >> 1;					// [0]=Target address (== MYADDR)
		pos_i = 1;								// Starting of reception
		twcr_new |= _BV(TWEA);					// Send ACK
	break;

	case TWI_TWSR_S_SLAW_OMNIADDR_RECEIVED:		// GCA received and ACK sent
		nop();
		// fall-through.
	case TWI_TWSR_S_SLAW_OMNIADDR_ARBIT_LOST:
		s_rx_lock = 1;
		s_rx_d[0] = twd >> 1;					// GCA
		pos_i = 1;								// Starting of reception
		twcr_new |= _BV(TWEA);					// Send after next coming data byte ACK
	break;

	case TWI_TWSR_S_SLAW_MYADDR_DATA_ACK:		// Data after SLA+W received
		nop();
		// fall-through.
	case TWI_TWSR_S_SLAW_OMNIADDR_DATA_ACK:
		if (cnt_i != 0b1111) {					// Closed parameter form
			if (pos_i <= 0b1110) {
				s_rx_d[pos_i] = twd;			// [1]=cmd, [2..]=parameters
			}
			if (pos_i == 1) {
				/* Load receive counter */
				if (s_rx_d[0] == TWI_SLAVE_ADDR_SMARTLCD) {
					cnt_i = 0;
					cnt_o = 0;

					switch (s_rx_d[1]) {
						case TWI_SMART_LCD_CMD_NOOP:
							cnt_i = 1;
						break;

						case TWI_SMART_LCD_CMD_GET_VER:
							cnt_i = 1;
							cnt_o = 1;
						break;


						case TWI_SMART_LCD_CMD_SET_MODE:
						case TWI_SMART_LCD_CMD_SET_PIXEL_TYPE:
						case TWI_SMART_LCD_CMD_DRAW_CIRC:
						case TWI_SMART_LCD_CMD_DRAW_FILLED_CIRC:
							cnt_i = 2;
						break;

						case TWI_SMART_LCD_CMD_SET_POS_X_Y:
						case TWI_SMART_LCD_CMD_DRAW_LINE:
						case TWI_SMART_LCD_CMD_DRAW_RECT:
						case TWI_SMART_LCD_CMD_DRAW_FILLED_RECT:
						case TWI_SMART_LCD_CMD_SHOW_DOP:
						case TWI_SMART_LCD_CMD_SHOW_POS_STATE:
						case TWI_SMART_LCD_CMD_SHOW_TCXO_PWM:
							cnt_i = 3;
						break;

						case TWI_SMART_LCD_CMD_SHOW_HR_MIN_SEC:
						case TWI_SMART_LCD_CMD_SHOW_POS_HEIGHT:
						case TWI_SMART_LCD_CMD_SHOW_SATS:
						case TWI_SMART_LCD_CMD_SHOW_TCXO_VC:
							cnt_i = 4;
						break;

						case TWI_SMART_LCD_CMD_SHOW_PPB:
						case TWI_SMART_LCD_CMD_SHOW_YEAR_MON_DAY:
							cnt_i = 5;
						break;

						case TWI_SMART_LCD_CMD_SHOW_CLK_STATE:
						case TWI_SMART_LCD_CMD_SHOW_POS_LAT:
						case TWI_SMART_LCD_CMD_SHOW_POS_LON:
							cnt_i = 6;
						break;
					}
				}
			}

			if (pos_i < 0b1110) {
				if (++pos_i <= cnt_i) {
					twcr_new |= _BV(TWEA);		// Send ACK
				} else {
					twcr_new &= ~_BV(TWEA);		// Send NACK
				}
			} else {
				twcr_new &= ~_BV(TWEA);			// Send NACK
			}

		} else {								// Open parameter form
			s_rx_d[2] = twd;
			if (!s_isr_twi_rcvd_command_open_form(s_rx_d, ++pos_i)) {
				twcr_new |= _BV(TWEA);			// Send ACK
			} else {
				pos_i = 0;
				twcr_new &= ~_BV(TWEA);			// Send NACK
			}
		}
	break;

	case TWI_TWSR_S_SLAW_MYADDR_DATA_NACK:		// NACK after last data byte sent
		nop();
		// fall-through.
	case TWI_TWSR_S_SLAW_OMNIADDR_DATA_NACK:
		s_rx_lock = 0;
		if (cnt_i != 0b1111) {
			s_isr_twi_rcvd_command_closed_form(s_rx_d, pos_i);	// Call interpreter for closed form of parameters
		} else {
			s_isr_twi_rcvd_command_open_form(s_rx_d, ++pos_i);	// Call interpreter for open form of parameters
		}
		pos_i = 0;
		cnt_i = 0;
		mem_set(s_rx_d, 8, 0x00);
		s_rx_lock = 0;
		twcr_new |= _BV(TWEA);					// TWI goes to unaddressed, be active again
	break;

	case TWI_TWSR_S_SLAW_STOP_REPEATEDSTART_RECEIVED:	// STOP or RESTART received while still addressed as slave
		s_rx_lock = 0;
		if (cnt_i != 0b1111) {
			s_isr_twi_rcvd_command_closed_form(s_rx_d, pos_i);	// Call interpreter for closed form of parameters
		} else {
			s_isr_twi_rcvd_command_open_form(s_rx_d, ++pos_i);	// Call interpreter for open form of parameters
		}
		pos_i = 0;
		twcr_new |= _BV(TWEA);					// TWI goes to unaddressed, be active again
	break;


	/* Slave Transmitter Mode */

	case TWI_TWSR_S_SLAR_MYADDR_DATA_ACK:		// SLA+R received and ACK has been returned
		nop();
		// fall-through.
	case TWI_TWSR_S_SLAR_MYADDR_ARBIT_LOST:
		s_rx_lock = 1;
		pos_o = 0;
		TWDR = cnt_o > pos_o ?  s_rx_d[pos_o++] : 0;
		if (cnt_o > pos_o) {
			twcr_new |= _BV(TWEA);				// More data to send ACK
		} else {
			twcr_new &= ~_BV(TWEA);				// No more data to send NACK
		}
	break;

	case TWI_TWSR_S_SLAR_OMNIADDR_DATA_ACK:		// Data sent and ACK has been returned
		TWDR = cnt_o > pos_o ?  s_rx_d[pos_o++] : 0;
		if (cnt_o > pos_o) {
			twcr_new |= _BV(TWEA);				// More data to send ACK
		} else {
			twcr_new &= ~_BV(TWEA);				// No more data to send NACK
		}
	break;

	case TWI_TWSR_S_SLAR_OMNIADDR_DATA_NACK:	// Data sent and NACK has been returned
		s_rx_lock = 0;
		twcr_new |= _BV(TWEA);					// TWI goes to unaddressed, be active again
	break;

	case TWI_TWSR_S_SLAR_MYADDR_LASTDATA_ACK:	// Last data sent and ACK has been returned
		/* message transmitted successfully in slave mode */
		s_rx_lock = 0;
		twcr_new |= _BV(TWEA);					// TWI goes to unaddressed, be active again
	break;

	case TWI_TWSR_BUS_ERROR_STARTSTOP:
		nop();
		s_tx_lock = 0;
		s_rx_lock = 0;
		twcr_new |= _BV(TWSTO) | _BV(TWEA);		// TWI goes to unaddressed, be active again
	break;

	case TWI_TWSR_BUS_ERROR_UNKNOWN:
		nop();
		s_tx_lock = 0;
		s_rx_lock = 0;
		twcr_new |= _BV(TWSTO) | _BV(TWEA);		// TWI goes to unaddressed, be active again
	break;

	default:
		nop();
		s_tx_lock = 0;
		s_rx_lock = 0;
		twcr_new |= _BV(TWSTO) | _BV(TWEA);		// TWI goes to unaddressed, be active again
	}

	return twcr_new;
}
