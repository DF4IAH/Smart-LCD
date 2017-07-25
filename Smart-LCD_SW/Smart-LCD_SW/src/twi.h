/*
 * twi.h
 *
 * Created: 13.02.2017 08:48:44
 *  Author: DF4IAH
 */


#ifndef TWI_H_
#define TWI_H_

// I2C address of the Smart-LCD device
#define TWI_SLAVE_ADDR_SMARTLCD								0x22

/* Exactly the address above w/o masking out any bits */
#define TWI_SLAVE_ADDR_BM									0b0000000

#define TWI_SLAVE_ADDR_GCE									0b1

// The unique commands of the Smart-LCD device for all modes
#define TWI_SMART_LCD_CMD_NOOP								0x00
#define TWI_SMART_LCD_CMD_GET_VER							0x01
#define TWI_SMART_LCD_CMD_SET_MODE							0x02
#define TWI_SMART_LCD_CMD_GET_STATE							0x03

// Mode 0x10 commands (Smart-LCD draw box)
#define TWI_SMART_LCD_CMD_CLS								0x10
#define TWI_SMART_LCD_CMD_SET_PIXEL_TYPE					0x14
#define TWI_SMART_LCD_CMD_SET_POS_X_Y						0x20
#define TWI_SMART_LCD_CMD_WRITE								0x30
#define TWI_SMART_LCD_CMD_DRAW_LINE							0x32
#define TWI_SMART_LCD_CMD_DRAW_RECT							0x34
#define TWI_SMART_LCD_CMD_DRAW_FILLED_RECT					0x36
#define TWI_SMART_LCD_CMD_DRAW_CIRC							0x38
#define TWI_SMART_LCD_CMD_DRAW_FILLED_CIRC					0x3A
#define TWI_SMART_LCD_CMD_GET_ROTBUT						0x60
#define TWI_SMART_LCD_CMD_GET_LIGHT							0x64
#define TWI_SMART_LCD_CMD_GET_TEMP							0x65
#define TWI_SMART_LCD_CMD_SET_LEDS							0x70
#define TWI_SMART_LCD_CMD_SET_BEEP							0x71
#define TWI_SMART_LCD_CMD_SET_BACKLIGHT						0x74
#define TWI_SMART_LCD_CMD_SET_CONTRAST						0x75

// Mode 0x20 commands (10 MHz-Ref-Osc)
#define TWI_SMART_LCD_CMD_SHOW_CLK_STATE					0x80
#define TWI_SMART_LCD_CMD_SHOW_YEAR_MON_DAY					0x81
#define TWI_SMART_LCD_CMD_SHOW_HR_MIN_SEC					0x82
#define TWI_SMART_LCD_CMD_SHOW_PPB							0x83

#define TWI_SMART_LCD_CMD_SHOW_TCXO_PWM						0x84
#define TWI_SMART_LCD_CMD_SHOW_TCXO_VC						0x85

#define TWI_SMART_LCD_CMD_SHOW_SATS							0x88
#define TWI_SMART_LCD_CMD_SHOW_DOP							0x89
#define TWI_SMART_LCD_CMD_SHOW_POS_STATE					0x8A
#define TWI_SMART_LCD_CMD_SHOW_POS_LAT						0x8B
#define TWI_SMART_LCD_CMD_SHOW_POS_LON						0x8C
#define TWI_SMART_LCD_CMD_SHOW_POS_HEIGHT					0x8D


/* */
#define TWI_SMART_LCD_MASTER_BUF_LEN						 8
#define TWI_SMART_LCD_SLAVE_BUF_LEN							16
#define TWI_SMART_LCD_SLAVE_RET_BUF_LEN						 2


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


uint8_t __vector_24__bottom(uint8_t tws, uint8_t twd, uint8_t twcr_cur);


#endif /* TWI_H_ */
