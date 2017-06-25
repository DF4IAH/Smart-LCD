/*
 * main.h
 *
 * Created: 08.02.2017 21:17:52
 *  Author: DF4IAH
 */


#ifndef MAIN_H_
#define MAIN_H_

#include <gfx_mono/gfx_mono.h>

#include "twi.h"


/* ATmega 328P - fuses ext:0xFD hi:0xB9 lo:0xC2 */


/* Version V1.1 */
#define VERSION														0x11


enum C_SMART_LCD_MODE__ENUM {
	C_SMART_LCD_MODE_UNIQUE											= 0x00,
	C_SMART_LCD_MODE_SMARTLCD										= 0x10,
	C_SMART_LCD_MODE_REFOSC											= 0x20,
};

enum C_EEPROM_ADDR__ENUM {
	C_EEPROM_ADDR_VERSION											= 0x00,
	C_EEPROM_ADDR_LCD_PM											= 0x10,
};

enum C_EEPROM_NVM_SETTING__ENUM {
	C_EEPROM_NVM_SETTING_LCD_CONTRAST								= 0x01,
	C_EEPROM_NVM_SETTING_VERSION									= 0x80,
	C_EEPROM_NVM_SETTING_ALL										= 0xFF
};


typedef struct status_struct {
	uint8_t		doAnimation											: 1;
	uint8_t		isAnimationStopped									: 1;
	uint8_t		reserved											: 6;  // fill to 8 bits
} status_t;

typedef struct showData {
	uint16_t	newClkState											: 1;
	uint16_t	newDate												: 1;
	uint16_t	newTime												: 1;
	uint16_t	newPpb												: 1;
	uint16_t	newPwm												: 1;
	uint16_t	newPv												: 1;
	uint16_t	newSatUse											: 1;
	uint16_t	newSatDop											: 1;
	uint16_t	newPosState											: 1;
	uint16_t	newPosLat											: 1;
	uint16_t	newPosLon											: 1;
	uint16_t	newPosHeight										: 1;
	uint16_t	newRsvrd											: 4;

	uint8_t				cmd;
	uint8_t				data[TWI_SMART_LCD_SLAVE_BUF_LEN];

	gfx_mono_color_t	pixelType;
	gfx_coord_t			pencil_x;
	gfx_coord_t			pencil_y;

	uint16_t	clkState_phaseVolt1000;
	int16_t		clkState_phaseDeg100;
	uint16_t	date_year;
	int16_t		ppb_int;
	uint16_t	ppb_frac1000;
	uint16_t	pv_frac1000;
	uint16_t	satDop_dop100;
	uint16_t	posLat_min_frac10000;
	uint16_t	posLon_min_frac10000;
	int16_t		pos_height_int;

	uint8_t		clkState_clk_state;
	uint8_t		date_month;
	uint8_t		date_day;
	uint8_t		time_hour;
	uint8_t		time_minute;
	uint8_t		time_second;
	uint8_t		pwm_int;
	uint8_t		pwm_frac256;
	uint8_t		pv_int;
	uint8_t		satUse_west;
	uint8_t		satUse_east;
	uint8_t		satUse_used;
	uint8_t		posState_fi;
	uint8_t		posState_m2;
	uint8_t		posLat_sgn;
	uint8_t		posLat_deg;
	uint8_t		posLat_min_int;
	uint8_t		posLon_sgn;
	uint8_t		posLon_deg;
	uint8_t		posLon_min_int;
	uint8_t		pos_height_frac10;

} showData_t;


/* UTILITIES section */
float get_abs_time(void);
void mem_set(uint8_t* buf, uint8_t count, uint8_t val);
void eeprom_nvm_settings_write(uint8_t flags);
void eeprom_nvm_settings_read(uint8_t flags);


/* TASK section */
void task(void);
void enter_sleep(uint8_t sleep_mode);
void halt(void);


/* MAIN section */
int main(void);


#endif /* MAIN_H_ */
