/*
 * twi.h
 *
 * Created: 13.02.2017 08:48:44
 *  Author: DF4IAH
 */ 


#ifndef TWI_H_
#define TWI_H_

/* @see 10MHz-Ref-Osc application to compare the address of this "Smart-LCD" */
#define TWI_SLAVE_ADDR		0x22
//#define TWI_SLAVE_ADDR		0x20

/* exactly the address above w/o masking out any bits */
#define TWI_SLAVE_ADDR_BM   0b0000000

#define TWI_SLAVE_ADDR_GCE	0b1


// commands used by the 10MHz-Ref-Osc
#define TWI_SMART_LCD_CMD_NOOP								0x00

#define TWI_SMART_LCD_CMD_GETVER							0x01

#define TWI_SMART_LCD_CMD_SHOW_CLK_STATE					0x80
#define TWI_SMART_LCD_CMD_SHOW_YEAR_MON_DAY					0x81
#define TWI_SMART_LCD_CMD_SHOW_HR_MIN_SEC					0x82
#define TWI_SMART_LCD_CMD_SHOW_PPM							0x83

#define TWI_SMART_LCD_CMD_SHOW_TCXO_PWM						0x84
#define TWI_SMART_LCD_CMD_SHOW_TCXO_VC						0x85

#define TWI_SMART_LCD_CMD_SHOW_SATS							0x88
#define TWI_SMART_LCD_CMD_SHOW_DOP							0x89
#define TWI_SMART_LCD_CMD_SHOW_POS_STATE					0x8A
#define TWI_SMART_LCD_CMD_SHOW_POS_LAT						0x8B
#define TWI_SMART_LCD_CMD_SHOW_POS_LON						0x8C
#define TWI_SMART_LCD_CMD_SHOW_POS_HEIGHT					0x8D


uint8_t __vector_24__bottom(uint8_t tws, uint8_t twd, uint8_t twcr_cur);


#endif /* TWI_H_ */
