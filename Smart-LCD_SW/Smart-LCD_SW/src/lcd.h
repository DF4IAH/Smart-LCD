/*
 * lcd.h
 *
 * Created: 22.02.2017 16:54:56
 *  Author: DF4IAH
 */


#ifndef LCD_H_
#define LCD_H_

#include <stdint.h>

#include "gfx_mono/gfx_mono.h"


// MUX: 1 = 128; Power Control: 0b01 = 26nF .. 43nF
#define C_LCD_PWR_CTRL				0b101

// Bias Ratio = 0: 10.7; 1: 11.3, 2:12.0, 3:12.7
#define C_LCD_BIASRATIO				2

// Gain: 0, 1, 2, 3; PM: 32/63 = 50%
#define C_LCD_GAIN_BM				(2 << 6)
#define C_LCD_PM					22
// NOTES: Optimum=15.6V, Max=16.0V
// BR=2&Gain=2&PM=0  --> 14.89V
// BR=2&Gain=2&PM=28 --> 15.50V
// BR=2&Gain=2&PM=32 --> 15.62V
// BR=2&Gain=2&PM=51 --> 15.99V


// MUX rate: 128, Temp Compensation: 0: 0.00, 1: -0.05, 2: -0.10, 3: -0.20% / K
#define C_LCD_MR_TC					((1 << 2) | 0)

// Mapping: no MY, no MX, 0, no MSF
#define C_LCD_MAPPING				0b1000

// Address Control: Page Address increment, no Wrap Around column/page
#define C_LCD_AC					0b000

// Status: BZ flag
#define C_LCD_STATUS_M	_BV(7)

#define LCD_SHOW_LINE_TOP			11
#define LCD_SHOW_LINE_HEIGHT		9
#define LCD_SHOW_LINE_LEFT			3
#define LCD_SHOW_CLMN_WIDTH         6


uint8_t	lcd_bus_read_status(void);
void	lcd_bus_write_cmd(uint8_t cmd);
void	lcd_bus_write_ram(uint8_t data);
uint8_t lcd_bus_read_ram(void);

uint8_t lcd_bounds_x(int x);
uint8_t lcd_bounds_y(int y);
void	lcd_contrast_update(void);
void    lcd_enable(uint8_t on);
void    lcd_page_set(uint8_t page);
void    lcd_col_set(uint8_t col);
void    lcd_cr(void);
void    lcd_home(void);
void    lcd_cls(void);
void    lcd_show_template(void);
uint8_t lcd_show_new_smartlcd_data(void);
uint8_t lcd_show_new_refosc_data(void);
void    lcd_animation_prepare(void);
void    lcd_animation_loop(void);
void    lcd_test(uint8_t pattern_bm);

void	isr_lcd_set_mode(int8_t mode);
void	isr_smartlcd_cmd(uint8_t cmd);
void	isr_smartlcd_cmd_data1(uint8_t cmd, uint8_t data0);
void	isr_smartlcd_cmd_data2(uint8_t cmd, uint8_t data0, uint8_t data1);
void	isr_smartlcd_cmd_data3(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2);
void	isr_smartlcd_cmd_data4(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3);
void	isr_smartlcd_cmd_data5(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4);
void	isr_smartlcd_cmd_data6(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5);
void	isr_smartlcd_cmd_data7(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6);
void	isr_smartlcd_cmd_data8(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7);
void	isr_smartlcd_cmd_data9(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7, uint8_t data8);
void	isr_smartlcd_cmd_data10(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7, uint8_t data8, uint8_t data9);
void	isr_smartlcd_cmd_data11(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7, uint8_t data8, uint8_t data9, uint8_t data10);
void	isr_smartlcd_cmd_data12(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7, uint8_t data8, uint8_t data9, uint8_t data10, uint8_t data11);
void	isr_smartlcd_cmd_data13(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7, uint8_t data8, uint8_t data9, uint8_t data10, uint8_t data11, uint8_t data12);
void	isr_smartlcd_cmd_data14(uint8_t cmd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4, uint8_t data5, uint8_t data6, uint8_t data7, uint8_t data8, uint8_t data9, uint8_t data10, uint8_t data11, uint8_t data12, uint8_t data13);
void	isr_lcd_write(const char *strbuf, gfx_coord_t pos_x, gfx_coord_t pos_y);
void    isr_lcd_10mhz_ref_osc_show_clkstate_phaseVolt1000_phaseDeg100(uint8_t clk_state, uint16_t phaseVolt1000, int16_t phaseDeg100);
void    isr_lcd_10mhz_ref_osc_show_date(uint16_t year, int8_t month, uint8_t day);
void    isr_lcd_10mhz_ref_osc_show_time(uint8_t hour, int8_t minute, uint8_t second);
void    isr_lcd_10mhz_ref_osc_show_ppm(int16_t ppm_int, uint16_t ppm_frac1000);
void    isr_lcd_10mhz_ref_osc_show_pwm(uint8_t pwm_int, uint8_t pwm_frac256);
void    isr_lcd_10mhz_ref_osc_show_pv(uint8_t pv_int, uint16_t pv_frac1000);
void    isr_lcd_10mhz_ref_osc_show_sat_use(uint8_t sat_west, uint8_t sat_east, uint8_t sat_used);
void    isr_lcd_10mhz_ref_osc_show_sat_dop(uint16_t sat_dop100);
void    isr_lcd_10mhz_ref_osc_show_pos_state(uint8_t state_fi, uint8_t state_m2);
void    isr_lcd_10mhz_ref_osc_show_pos_lat(uint8_t lat_sgn, uint8_t lat_deg, uint8_t lat_min_int, uint16_t lat_min_frac10000);
void    isr_lcd_10mhz_ref_osc_show_pos_lon(uint8_t lon_sgn, uint8_t lon_deg, uint8_t lon_min_int, uint16_t lon_min_frac10000);
void    isr_lcd_10mhz_ref_osc_show_pos_height(int16_t height_int, uint8_t height_frac10);

const void* lcd_get_sysfont(void);

uint8_t	lcd_init(void);
void	lcd_shutdown(void);


#endif /* LCD_H_ */
