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
extern showData_t			g_showData;
extern gfx_mono_color_t		g_lcd_pixel_type;
extern gfx_coord_t			g_lcd_pencil_x;
extern gfx_coord_t			g_lcd_pencil_y;

/* TWI Master mode */
static uint8_t				s_tx_next_len = 0;
static uint8_t				s_tx_next_d[8];
static uint8_t				s_tx_lock = 0;
static uint8_t				s_tx_len = 0;
static uint8_t				s_tx_d[TWI_SMART_LCD_MASTER_BUF_LEN];

/* TWI Slave mode */
static uint8_t				s_rx_d[TWI_SMART_LCD_SLAVE_BUF_LEN];
static uint8_t				s_rx_ret_d[TWI_SMART_LCD_SLAVE_RET_BUF_LEN];
static uint8_t				s_rx_ret_len = 0;


/* ISR - interrupt disabled functions called within the TWI interrupt handling */

static void s_isr_lcd_set_mode(int8_t mode)
{
	g_SmartLCD_mode = mode;
	if (mode) {
		g_status.doAnimation = false;	// Stop animation demo

		} else {
		// Reset display
		lcd_init();
		lcd_test(0b11110001);			// Start animation again
	}
}


static void s_isr_smartlcd_cmd(uint8_t cmd)
{
	g_showData.cmd = cmd;
}

static void s_isr_smartlcd_cmd_data1(uint8_t cmd, uint8_t data0)
{
	s_isr_smartlcd_cmd(cmd);
	g_showData.data[0] = data0;
}

static void s_isr_smartlcd_cmd_data2(uint8_t cmd, uint8_t data0, uint8_t data1)
{
	s_isr_smartlcd_cmd_data1(cmd, data0);
	g_showData.data[1] = data1;
}

static void s_isr_smartlcd_cmd_data3(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2)
{
	s_isr_smartlcd_cmd_data2(cmd, data0, data1);
	g_showData.data[2] = data2;
}

static void s_isr_smartlcd_cmd_data4(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3)
{
	s_isr_smartlcd_cmd_data3(cmd, data0, data1, data2);
	g_showData.data[3] = data3;
}

static void s_isr_smartlcd_cmd_data5(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4)
{
	s_isr_smartlcd_cmd_data4(cmd, data0, data1, data2, data3);
	g_showData.data[4] = data4;
}

static void s_isr_smartlcd_cmd_data6(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5)
{
	s_isr_smartlcd_cmd_data5(cmd, data0, data1, data2, data3, data4);
	g_showData.data[5] = data5;
}

static void s_isr_smartlcd_cmd_data7(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6)
{
	s_isr_smartlcd_cmd_data6(cmd, data0, data1, data2, data3, data4, data5);
	g_showData.data[6] = data6;
}

static void s_isr_smartlcd_cmd_data8(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7)
{
	s_isr_smartlcd_cmd_data7(cmd, data0, data1, data2, data3, data4, data5, data6);
	g_showData.data[7] = data7;
}

static void s_isr_smartlcd_cmd_data9(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7, uint8_t data8)
{
	s_isr_smartlcd_cmd_data8(cmd, data0, data1, data2, data3, data4, data5, data6, data7);
	g_showData.data[8] = data8;
}

static void s_isr_smartlcd_cmd_data10(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7, uint8_t data8, uint8_t data9)
{
	s_isr_smartlcd_cmd_data9(cmd, data0, data1, data2, data3, data4, data5, data6, data7, data8);
	g_showData.data[9] = data9;
}

static void s_isr_smartlcd_cmd_data11(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7, uint8_t data8, uint8_t data9, uint8_t data10)
{
	s_isr_smartlcd_cmd_data10(cmd, data0, data1, data2, data3, data4, data5, data6, data7, data8, data9);
	g_showData.data[10] = data10;
}

static void s_isr_smartlcd_cmd_data12(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7, uint8_t data8, uint8_t data9, uint8_t data10, uint8_t data11)
{
	s_isr_smartlcd_cmd_data11(cmd, data0, data1, data2, data3, data4, data5, data6, data7, data8, data9, data10);
	g_showData.data[11] = data11;
}

static void s_isr_smartlcd_cmd_data13(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7, uint8_t data8, uint8_t data9, uint8_t data10, uint8_t data11, uint8_t data12)
{
	s_isr_smartlcd_cmd_data12(cmd, data0, data1, data2, data3, data4, data5, data6, data7, data8, data9, data10, data11);
	g_showData.data[12] = data12;
}

static void s_isr_smartlcd_cmd_data14(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7, uint8_t data8, uint8_t data9, uint8_t data10, uint8_t data11, uint8_t data12, uint8_t data13)
{
	s_isr_smartlcd_cmd_data13(cmd, data0, data1, data2, data3, data4, data5, data6, data7, data8, data9, data10, data11, data12);
	g_showData.data[13] = data13;
}


static void s_isr_lcd_10mhz_ref_osc_show_clkstate_phaseVolt1000_phaseDeg100(uint8_t clk_state, uint16_t phaseVolt1000, int16_t phaseDeg100)
{
	// interrupt is already disabled, here
	if ((g_showData.clkState_clk_state     != clk_state    )  ||
	(g_showData.clkState_phaseVolt1000 != phaseVolt1000)  ||
	(g_showData.clkState_phaseDeg100   != phaseDeg100  )) {
		g_showData.newClkState            = true;
		g_showData.clkState_clk_state     = clk_state;
		g_showData.clkState_phaseVolt1000 = phaseVolt1000;
		g_showData.clkState_phaseDeg100   = phaseDeg100;
	}
}

static void s_isr_lcd_10mhz_ref_osc_show_date(uint16_t year, int8_t month, uint8_t day)
{
	// interrupt is already disabled, here
	if (g_showData.date_year != year ||
	g_showData.date_month != month ||
	g_showData.date_day != day) {
		g_showData.newDate = true;
		g_showData.date_year = year;
		g_showData.date_month = month;
		g_showData.date_day = day;
	}
}

static void s_isr_lcd_10mhz_ref_osc_show_time(uint8_t hour, int8_t minute, uint8_t second)
{
	// interrupt is already disabled, here
	if (g_showData.time_hour != hour ||
	g_showData.time_minute != minute ||
	g_showData.time_second != second) {
		g_showData.newTime = true;
		g_showData.time_hour = hour;
		g_showData.time_minute = minute;
		g_showData.time_second = second;
	}
}

static void s_isr_lcd_10mhz_ref_osc_show_ppm(int16_t ppm_int, uint16_t ppm_frac1000)
{
	// interrupt is already disabled, here
	if (g_showData.ppb_int != ppm_int ||
	g_showData.ppb_frac1000 != ppm_frac1000) {
		g_showData.newPpb = true;
		g_showData.ppb_int = ppm_int;
		g_showData.ppb_frac1000 = ppm_frac1000;
	}
}

static void s_isr_lcd_10mhz_ref_osc_show_pwm(uint8_t pwm_int, uint8_t pwm_frac256)
{
	// interrupt is already disabled, here
	if (g_showData.pwm_int != pwm_int ||
	g_showData.pwm_frac256 != pwm_frac256) {
		g_showData.newPwm = true;
		g_showData.pwm_int = pwm_int;
		g_showData.pwm_frac256 = pwm_frac256;
	}
}

static void s_isr_lcd_10mhz_ref_osc_show_pv(uint8_t pv_int, uint16_t pv_frac1000)
{
	// interrupt is already disabled, here
	if (g_showData.pv_int != pv_int ||
	g_showData.pv_frac1000 != pv_frac1000) {
		g_showData.newPv = true;
		g_showData.pv_int = pv_int;
		g_showData.pv_frac1000 = pv_frac1000;
	}
}

static void s_isr_lcd_10mhz_ref_osc_show_sat_use(uint8_t sat_west, uint8_t sat_east, uint8_t sat_used)
{
	// interrupt is already disabled, here
	if (g_showData.satUse_west != sat_west ||
	g_showData.satUse_east != sat_east ||
	g_showData.satUse_used != sat_used) {
		g_showData.newSatUse = true;
		g_showData.satUse_west = sat_west;
		g_showData.satUse_east = sat_east;
		g_showData.satUse_used = sat_used;
	}
}

static void s_isr_lcd_10mhz_ref_osc_show_sat_dop(uint16_t sat_dop100)
{
	// interrupt is already disabled, here
	if (g_showData.satDop_dop100 != sat_dop100) {
		g_showData.newSatDop = true;
		g_showData.satDop_dop100 = sat_dop100;
	}
}

static void s_isr_lcd_10mhz_ref_osc_show_pos_state(uint8_t state_fi, uint8_t state_m2)
{
	// interrupt is already disabled, here
	if (g_showData.posState_fi != state_fi ||
	g_showData.posState_m2 != state_m2) {
		g_showData.newPosState = true;
		g_showData.posState_fi = state_fi;
		g_showData.posState_m2 = state_m2;
	}
}

static void s_isr_lcd_10mhz_ref_osc_show_pos_lat(uint8_t lat_sgn, uint8_t lat_deg, uint8_t lat_min_int, uint16_t lat_min_frac10000)
{
	// interrupt is already disabled, here
	if (g_showData.posLat_sgn != lat_sgn ||
	g_showData.posLat_deg != lat_deg ||
	g_showData.posLat_min_int != lat_min_int ||
	g_showData.posLat_min_int != lat_min_int ||
	g_showData.posLat_min_frac10000 != lat_min_frac10000) {
		g_showData.newPosLat = true;
		g_showData.posLat_sgn = lat_sgn;
		g_showData.posLat_deg = lat_deg;
		g_showData.posLat_min_int = lat_min_int;
		g_showData.posLat_min_frac10000 = lat_min_frac10000;
	}
}

static void s_isr_lcd_10mhz_ref_osc_show_pos_lon(uint8_t lon_sgn, uint8_t lon_deg, uint8_t lon_min_int, uint16_t lon_min_frac10000)
{
	// interrupt is already disabled, here
	if (g_showData.posLon_sgn != lon_sgn ||
	g_showData.posLon_deg != lon_deg ||
	g_showData.posLon_deg != lon_deg ||
	g_showData.posLon_min_int != lon_min_int ||
	g_showData.posLon_min_frac10000 != lon_min_frac10000) {
		g_showData.newPosLon = true;
		g_showData.posLon_sgn = lon_sgn;
		g_showData.posLon_deg = lon_deg;
		g_showData.posLon_min_int = lon_min_int;
		g_showData.posLon_min_frac10000 = lon_min_frac10000;
	}
}

static void s_isr_lcd_10mhz_ref_osc_show_pos_height(int16_t height_int, uint8_t height_frac10)
{
	// interrupt is already disabled, here
	if ((g_showData.pos_height_int != height_int) || (g_showData.pos_height_frac10 != height_frac10)) {
		g_showData.newPosHeight = true;
		g_showData.pos_height_int = height_int;
		g_showData.pos_height_frac10 = height_frac10;
	}
}


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

static uint8_t s_isr_twi_rcvd_command_open_form(uint8_t data[], uint8_t pos)
{
	uint8_t err = 1;
	// TODO: implementation
	return err;
}

static void s_isr_twi_rcvd_command_closed_form(uint8_t data[], uint8_t cnt)
{
	uint8_t isGCA	= !data[0];
	uint8_t cmd		=  data[1];

	if (isGCA) {
		switch (cmd) {
			case 0b0100000:								// IDENTIFY
				// TODO: prepare ADR+R data
			break;

			default:
			{
				// do nothing
			}
		}
	}  // if (isGCA)

	else if ((data[0] == TWI_SLAVE_ADDR_SMARTLCD)) {
		/* unique command section for all modes */
		switch (cmd) {
			case TWI_SMART_LCD_CMD_GET_VER:
			s_rx_ret_d[0] = I2C_VERSION;
			s_rx_ret_len = 1;
			return;

			case TWI_SMART_LCD_CMD_SET_MODE:
			s_isr_lcd_set_mode(data[2]);
			return;

			case TWI_SMART_LCD_CMD_GET_STATE:
			s_rx_ret_d[0] = g_showData.cmd ?  0x81 : 0x80;		// (Valid << 7) | (Busy << 0)
			s_rx_ret_len = 1;
			return;

			default:
			s_rx_ret_len = 0;
		}

		if (g_SmartLCD_mode == C_SMART_LCD_MODE_SMARTLCD) {
			if (!(g_showData.cmd)) {							// Do when no command in process only
				switch (cmd) {
					case TWI_SMART_LCD_CMD_CLS:					// Clear screen
						s_isr_smartlcd_cmd(cmd);
					break;

					case TWI_SMART_LCD_CMD_SET_PIXEL_TYPE:		// Set next pixels (OFF / ON / XOR)
						s_isr_smartlcd_cmd_data1(cmd, data[2]);
					break;

					case TWI_SMART_LCD_CMD_SET_POS_X_Y:			// Set pencil position (x, y)
						s_isr_smartlcd_cmd_data2(cmd, data[2], data[3]);
					break;

					case TWI_SMART_LCD_CMD_WRITE:				// Write text of length (length, buffer...)
					{
						switch (data[2]) {
							case 1:
								s_isr_smartlcd_cmd_data2(cmd, data[2], data[3]);
							break;

							case 2:
								s_isr_smartlcd_cmd_data3(cmd, data[2], data[3], data[4]);
							break;

							case 3:
								s_isr_smartlcd_cmd_data4(cmd, data[2], data[3], data[4], data[5]);
							break;

							case 4:
								s_isr_smartlcd_cmd_data5(cmd, data[2], data[3], data[4], data[5], data[6]);
							break;

							case 5:
								s_isr_smartlcd_cmd_data6(cmd, data[2], data[3], data[4], data[5], data[6], data[7]);
							break;

							case 6:
								s_isr_smartlcd_cmd_data7(cmd, data[2], data[3], data[4], data[5], data[6], data[7], data[8]);
							break;

							case 7:
								s_isr_smartlcd_cmd_data8(cmd, data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9]);
							break;

							case 8:
								s_isr_smartlcd_cmd_data9(cmd, data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10]);
							break;

							case 9:
								s_isr_smartlcd_cmd_data10(cmd, data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11]);
							break;

							case 10:
								s_isr_smartlcd_cmd_data11(cmd, data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12]);
							break;

							case 11:
								s_isr_smartlcd_cmd_data12(cmd, data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13]);
							break;

							case 12:
								s_isr_smartlcd_cmd_data13(cmd, data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14]);
							break;

							case 13:
								s_isr_smartlcd_cmd_data14(cmd, data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);
							break;

							case 0:
							default:
							{
								// nothing to be done
							}
							break;
						}  // switch (data[2])
					}
					break;

					case TWI_SMART_LCD_CMD_DRAW_LINE:			// Draw line from current pencil position to next position (x, y)
						s_isr_smartlcd_cmd_data2(cmd, data[2], data[3]);
					break;

					case TWI_SMART_LCD_CMD_DRAW_RECT:			// Draw rectangular frame with pencil's start position with dimension (width, height)
						s_isr_smartlcd_cmd_data2(cmd, data[2], data[3]);
					break;

					case TWI_SMART_LCD_CMD_DRAW_FILLED_RECT:	// Draw filled rectangular frame with pencil's start position with dimension (width, height)
						s_isr_smartlcd_cmd_data2(cmd, data[2], data[3]);
					break;

					case TWI_SMART_LCD_CMD_DRAW_CIRC:			// Draw circle or ellipse from the pencil's center point with (radius)
						s_isr_smartlcd_cmd_data1(cmd, data[2]);
					break;

					case TWI_SMART_LCD_CMD_DRAW_FILLED_CIRC:	// Draw filled circle or ellipse from the pencil's center point with (radius)
						s_isr_smartlcd_cmd_data1(cmd, data[2]);
					break;

					default:
					{
						// do nothing
					}
				}  // switch (cmd)
			}  // if (!(g_showData.cmd))
		}  // if (g_SmartLCD_mode == C_SMART_LCD_MODE_SMARTLCD)

		else if (g_SmartLCD_mode == C_SMART_LCD_MODE_REFOSC) {
			switch (cmd) {
				case TWI_SMART_LCD_CMD_SHOW_CLK_STATE:
					s_isr_lcd_10mhz_ref_osc_show_clkstate_phaseVolt1000_phaseDeg100(data[2], (uint16_t) (data[3] | (data[4] << 8)), (int16_t) (data[5] | (data[6] << 8)));
				break;

				case TWI_SMART_LCD_CMD_SHOW_YEAR_MON_DAY:
					s_isr_lcd_10mhz_ref_osc_show_date(data[2] | (data[3] << 8), data[4], data[5]);
				break;

				case TWI_SMART_LCD_CMD_SHOW_HR_MIN_SEC:
					s_isr_lcd_10mhz_ref_osc_show_time(data[2], data[3], data[4]);
				break;

				case TWI_SMART_LCD_CMD_SHOW_PPB:
					s_isr_lcd_10mhz_ref_osc_show_ppm((int16_t) (data[2] | (data[3] << 8)), data[4] | (data[5] << 8));
				break;

				case TWI_SMART_LCD_CMD_SHOW_TCXO_PWM:
					s_isr_lcd_10mhz_ref_osc_show_pwm(data[2], data[3]);
				break;

				case TWI_SMART_LCD_CMD_SHOW_TCXO_VC:
					s_isr_lcd_10mhz_ref_osc_show_pv(data[2], data[3] | (data[4] << 8));
				break;

				case TWI_SMART_LCD_CMD_SHOW_SATS:
					s_isr_lcd_10mhz_ref_osc_show_sat_use(data[2], data[3], data[4]);
				break;

				case TWI_SMART_LCD_CMD_SHOW_DOP:
					s_isr_lcd_10mhz_ref_osc_show_sat_dop(data[2] | (data[3] << 8));
				break;

				case TWI_SMART_LCD_CMD_SHOW_POS_STATE:
					s_isr_lcd_10mhz_ref_osc_show_pos_state(data[2], data[3]);
				break;

				case TWI_SMART_LCD_CMD_SHOW_POS_LAT:
					s_isr_lcd_10mhz_ref_osc_show_pos_lat(data[2], data[3], data[4], data[5] | (data[6] << 8));
				break;

				case TWI_SMART_LCD_CMD_SHOW_POS_LON:
					s_isr_lcd_10mhz_ref_osc_show_pos_lon(data[2], data[3], data[4], data[5] | (data[6] << 8));
				break;

				case TWI_SMART_LCD_CMD_SHOW_POS_HEIGHT:
					s_isr_lcd_10mhz_ref_osc_show_pos_height((data[2] | (data[3] << 8)), data[4]);
				break;

				default:
				{
					// do nothing for unsupported commands
				}
			}  // switch (cmd)
		}  // if (g_SmartLCD_mode == C_SMART_LCD_MODE_REFOSC)
	}  // if ((data[0] == TWI_SLAVE_ADDR_SMARTLCD))
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
	case TWI_TWSR_S_SLAW_MYADDR_ARBIT_LOST:
		mem_set(s_rx_d, TWI_SMART_LCD_SLAVE_BUF_LEN, 0x00);
		s_rx_d[0] = twd >> 1;					// [0]=Target address (== MYADDR)
		pos_i = 1;								// Starting of reception
		twcr_new |= _BV(TWEA);					// Send ACK
	break;

	case TWI_TWSR_S_SLAW_OMNIADDR_RECEIVED:		// GCA received and ACK sent
	case TWI_TWSR_S_SLAW_OMNIADDR_ARBIT_LOST:
		s_rx_d[0] = twd >> 1;					// GCA
		pos_i = 1;								// Starting of reception
		twcr_new |= _BV(TWEA);					// Send after next coming data byte ACK
	break;

	case TWI_TWSR_S_SLAW_MYADDR_DATA_ACK:		// Data after SLA+W received
	case TWI_TWSR_S_SLAW_OMNIADDR_DATA_ACK:
		if (cnt_i != 0b1111) {					// Closed parameter form
			if (pos_i < 0b1111) {
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
						case TWI_SMART_LCD_CMD_GET_STATE:
							cnt_i = 1;
							cnt_o = 1;
						break;


						case TWI_SMART_LCD_CMD_SET_MODE:
						case TWI_SMART_LCD_CMD_SET_PIXEL_TYPE:
							cnt_i = 2;
						break;

						case TWI_SMART_LCD_CMD_SET_POS_X_Y:
						case TWI_SMART_LCD_CMD_DRAW_CIRC:
						case TWI_SMART_LCD_CMD_DRAW_FILLED_CIRC:
						case TWI_SMART_LCD_CMD_SHOW_DOP:
						case TWI_SMART_LCD_CMD_SHOW_POS_STATE:
						case TWI_SMART_LCD_CMD_SHOW_TCXO_PWM:
							cnt_i = 3;
						break;

						case TWI_SMART_LCD_CMD_DRAW_LINE:
						case TWI_SMART_LCD_CMD_DRAW_RECT:
						case TWI_SMART_LCD_CMD_DRAW_FILLED_RECT:
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

						case TWI_SMART_LCD_CMD_WRITE:
							cnt_i = TWI_SMART_LCD_SLAVE_BUF_LEN;	// Max length of incoming data
						break;
					}
				}
			}
			else if (pos_i == 2) {
				if (s_rx_d[0] == TWI_SLAVE_ADDR_SMARTLCD) {
					if (s_rx_d[1] == TWI_SMART_LCD_CMD_WRITE) {
						/* Correct length of string to actual size */
						uint8_t str_len = s_rx_d[2];
						cnt_i = ((str_len <= (TWI_SMART_LCD_SLAVE_BUF_LEN - 2)) && (str_len < 0b1111)) ?  (str_len + 2) : 2;
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
	case TWI_TWSR_S_SLAW_OMNIADDR_DATA_NACK:
		if (cnt_i != 0b1111) {
			s_isr_twi_rcvd_command_closed_form(s_rx_d, pos_i);	// Call interpreter for closed form of parameters
		} else {
			s_isr_twi_rcvd_command_open_form(s_rx_d, ++pos_i);	// Call interpreter for open form of parameters
		}
		pos_i = 0;
		cnt_i = 0;
		mem_set(s_rx_d, 8, 0x00);
		twcr_new |= _BV(TWEA);					// TWI goes to unaddressed, be active again
	break;

	case TWI_TWSR_S_SLAW_STOP_REPEATEDSTART_RECEIVED:	// STOP or RESTART received while still addressed as slave
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
	case TWI_TWSR_S_SLAR_OMNIADDR_DATA_ACK:		// Data sent and ACK has been returned
		pos_o = 0;
		cnt_o = s_rx_ret_len;
		s_rx_ret_len = 0;
		TWDR = cnt_o > pos_o ?  s_rx_ret_d[pos_o++] : 0x00;
		if (cnt_o > pos_o) {
			twcr_new |= _BV(TWEA);				// More data to send ACK
		} else {
			twcr_new &= ~_BV(TWEA);				// No more data to send NACK
		}
	break;

	case TWI_TWSR_S_SLAR_OMNIADDR_DATA_NACK:	// Data sent and NACK has been returned
		twcr_new |= _BV(TWEA);					// TWI goes to unaddressed, be active again
	break;

	case TWI_TWSR_S_SLAR_MYADDR_LASTDATA_ACK:	// Last data sent and ACK has been returned
		/* message transmitted successfully in slave mode */
		twcr_new |= _BV(TWEA);					// TWI goes to unaddressed, be active again
	break;

	case TWI_TWSR_S_SLAR_MYADDR_ARBIT_LOST:
	case TWI_TWSR_BUS_ERROR_STARTSTOP:
	case TWI_TWSR_BUS_ERROR_UNKNOWN:
		twcr_new |= _BV(TWSTO) | _BV(TWEA);		// TWI goes to unaddressed, be active again
	break;

	default:
		twcr_new |= _BV(TWSTO) | _BV(TWEA);		// TWI goes to unaddressed, be active again
	}

	return twcr_new;
}
