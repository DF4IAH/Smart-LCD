/*
 * twi.h
 *
 * Created: 13.02.2017 08:48:44
 *  Author: DF4IAH
 */ 


#ifndef TWI_H_
#define TWI_H_

/* @see 10MHz-Ref-Osc application to compare the address of this "Smart-LCD" */
#define TWI_SLAVE_ADDR_10MHZREFOSC							0x22
#define TWI_SLAVE_ADDR_SMARTLCD								0x30

/* exactly the address above w/o masking out any bits */
#define TWI_SLAVE_ADDR_BM									0b0000000

#define TWI_SLAVE_ADDR_GCE									0b1

/* TWI TWSR states after mask is applied */
#define TWI_TWSR_STATE_MASK									0xF8
#define TWI_TWSR_START										0x08
#define TWI_TWSR_REPEATEDSTART								0x10
#define TWI_TWSR_M_SLAW_ADDR_ACK							0x18
#define TWI_TWSR_M_SLAW_ADDR_NACK							0x20
#define TWI_TWSR_M_SLAW_DATA_ACK							0x28
#define TWI_TWSR_M_SLAW_DATA_NACK							0x30
#define TWI_TWSR_M_SLAW_ARBIT_LOST							0x38
#define TWI_TWSR_M_SLAR_ADDR_ACK							0x40
#define TWI_TWSR_M_SLAR_ADDR_NACK							0x48
#define TWI_TWSR_M_SLAR_DATA_ACK							0x50
#define TWI_TWSR_M_SLAR_DATA_NACK							0x58

#define TWI_TWSR_S_SLAW_MYADDR_RECEIVED						0x60
#define TWI_TWSR_S_SLAW_MYADDR_ARBIT_LOST					0x68
#define TWI_TWSR_S_SLAW_OMNIADDR_RECEIVED					0x70
#define TWI_TWSR_S_SLAW_OMNIADDR_ARBIT_LOST					0x78
#define TWI_TWSR_S_SLAW_MYADDR_DATA_ACK						0x80
#define TWI_TWSR_S_SLAW_MYADDR_DATA_NACK					0x88
#define TWI_TWSR_S_SLAW_OMNIADDR_DATA_ACK					0x90
#define TWI_TWSR_S_SLAW_OMNIADDR_DATA_NACK					0x98
#define TWI_TWSR_S_SLAW_STOP_REPEATEDSTART_RECEIVED			0xA0

#define TWI_TWSR_S_SLAR_MYADDR_DATA_ACK						0xA8
#define TWI_TWSR_S_SLAR_MYADDR_ARBIT_LOST					0xB0
#define TWI_TWSR_S_SLAR_OMNIADDR_DATA_ACK					0xB8
#define TWI_TWSR_S_SLAR_OMNIADDR_DATA_NACK					0xC0
#define TWI_TWSR_S_SLAR_MYADDR_LASTDATA_ACK					0xC8

#define TWI_TWSR_BUS_ERROR_STARTSTOP						0x00
#define TWI_TWSR_BUS_ERROR_UNKNOWN							0xF8



/* Commands used by the 10MHz-Ref-Osc */
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
