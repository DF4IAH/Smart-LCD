/*
 * main.h
 *
 * Created: 08.02.2017 21:17:52
 *  Author: DF4IAH
 */ 


#ifndef MAIN_H_
#define MAIN_H_


enum ADC_STATES {
	ADC_STATE_PRE_LDR = 0,
	ADC_STATE_VLD_LDR,
	ADC_STATE_PRE_TEMP,
	ADC_STATE_VLD_TEMP,

	ADC_STATE__COUNT
	};


/* HELPERS */
static void s_reset_global_vars(void);
static void s_asm_break(void);


/* ISR */
void bad_interrupt(void);
void __vector_21__bottom(uint8_t reason, uint16_t adc_val);

/* INIT section */
static void s_tc_init(void);
static void s_tc_disable(void);
static void s_tc_start(void);
static void s_adc_init(void);
static void s_adc_disable(void);
static void s_twi_init(void);
static void s_twi_disable(void);


/* TASK section */
static void s_task_backlight(float adc_ldr);
static void s_task_temp(float adc_temp);
static void s_task(void);
static void s_enter_sleep(uint8_t sleep_mode);
void halt(void);


/* MAIN section */
int main(void);


#endif /* MAIN_H_ */