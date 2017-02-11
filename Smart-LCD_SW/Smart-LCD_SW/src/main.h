/*
 * main.h
 *
 * Created: 08.02.2017 21:17:52
 *  Author: DF4IAH
 */ 


#ifndef MAIN_H_
#define MAIN_H_


/* HELPERS */
static void s_reset_global_vars(void);
static void s_asm_break(void);


/* ISR */
void bad_interrupt(void);


/* INIT section */
static void s_tc_init(void);
static void s_tc_start(void);
static void s_adc_init(void);


/* TASK section */
static void s_task_backlight(uint16_t adc);
static void s_task(void);
void halt(void);


/* MAIN section */
int main(void);


#endif /* MAIN_H_ */