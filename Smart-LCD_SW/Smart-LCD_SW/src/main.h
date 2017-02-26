/*
 * main.h
 *
 * Created: 08.02.2017 21:17:52
 *  Author: DF4IAH
 */ 


#ifndef MAIN_H_
#define MAIN_H_


/* HELPERS */
//static void s_reset_global_vars(void);


/* INIT section */
//static void s_tc_init(void);
//static void s_tc_disable(void);
//static void s_tc_start(void);
//static void s_adc_init(void);
//static void s_adc_disable(void);
//static void s_twi_init(void);
//static void s_twi_disable(void);


/* TASK section */
//static void s_task_backlight(float adc_ldr);
//static void s_task_temp(float adc_temp);
//static void s_task(void);
void s_task(void);
//static void s_enter_sleep(uint8_t sleep_mode);
void halt(void);


/* MAIN section */
int main(void);


#endif /* MAIN_H_ */
