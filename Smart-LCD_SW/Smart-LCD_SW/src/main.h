/*
 * main.h
 *
 * Created: 08.02.2017 21:17:52
 *  Author: DF4IAH
 */ 


#ifndef MAIN_H_
#define MAIN_H_


/* INIT section */

static void s_tc_init(void);
static void s_tc_start(void);
static void s_adc_init(void);


/* RUNNING section */

static void s_task(void);

void halt(void);

int main(void);


#endif /* MAIN_H_ */