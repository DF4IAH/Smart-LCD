/*
 * main.h
 *
 * Created: 08.02.2017 21:17:52
 *  Author: DF4IAH
 */ 


#ifndef MAIN_H_
#define MAIN_H_


/* Version V1.0 */
#define VERSION														0x10


typedef struct status_struct {
	uint8_t		doAnimation											: 1;
	uint8_t		isAnimationStopped									: 1;
	uint8_t		reserved											: 6;  // fill to 8 bits
} status_t;

typedef struct showData {
	uint16_t	newClkState											: 1;
	uint16_t	newDate												: 1;
	uint16_t	newTime												: 1;
	uint16_t	newPpm												: 1;
	uint16_t	newPwm												: 1;
	uint16_t	newPv												: 1;
	uint16_t	newSatUse											: 1;
	uint16_t	newSatDop											: 1;
	uint16_t	newPosState											: 1;
	uint16_t	newPosLat											: 1;
	uint16_t	newPosLon											: 1;
	uint16_t	newPosHeight										: 1;
	uint16_t	newRsvrd											: 4;

	int16_t		clkState_phase100;
	uint16_t	date_year;
	int16_t		ppm_int;
	uint16_t	ppm_frac1000;
	uint16_t	pv_frac1000;
	uint16_t	satDop_dop100;
	uint16_t	posLat_min_frac10000;
	uint16_t	posLon_min_frac10000;
	int16_t		pos_heigth;

	uint8_t		clkState_clk_state;
	uint8_t		date_month;
	uint8_t		date_day;
	uint8_t		time_hour;
	uint8_t		time_minute;
	uint8_t		time_second;
	uint8_t		pwm_int;
	uint8_t		pwm_frac1000;
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

} showData_t;


/* UTILITIES section */
float get_abs_time(void);
void mem_set(uint8_t* buf, uint8_t count, uint8_t val);


/* TASK section */
void s_task(void);
void halt(void);


/* MAIN section */
int main(void);


#endif /* MAIN_H_ */
