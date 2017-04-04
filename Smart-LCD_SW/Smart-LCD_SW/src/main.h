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


/* UTILITIES section */
float get_abs_time(void);
void mem_set(uint8_t* buf, uint8_t count, uint8_t val);


/* TASK section */
void s_task(void);
void halt(void);


/* MAIN section */
int main(void);


#endif /* MAIN_H_ */
