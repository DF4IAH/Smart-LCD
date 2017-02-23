/*
 * lcd.h
 *
 * Created: 22.02.2017 16:54:56
 *  Author: DF4IAH
 */ 


#ifndef LCD_H_
#define LCD_H_


// Bias Ratio = 11.3
#define C_LCD_BIASRATIO	(1 << 0)

// Gain = 0, PM = 32/63 abt. 50%
#define C_LCD_GAIN_PM	((0 << 6) | (32 << 0))

// MUX rate = 128, Temp Compensation = -0.05% / K
#define C_LCD_MR_TC		((1 << 2) | (1 << 0))

#define C_LCD_PWR_CTRL	(0b101 << 0)

// Mapping: no MX, no MY, no MSF
#define C_LCD_MAPPING	(0 << 0)

// Address Control: Cursor update mode, Page Address increment, no Wrap Around column/page
#define C_LCD_AC		(0b1000 << 0)


#define C_LCD_STATUS_M	0b10010000



uint8_t	lcd_bus_read_status(void);
void	lcd_bus_wait_ready(void);
void	lcd_bus_write_cmd(uint8_t cmd_data);
void	lcd_bus_write_ram(uint8_t cmd_data);
uint8_t lcd_bus_read_ram(void);

uint8_t	lcd_init(void);
void	lcd_shutdown(void);


#endif /* LCD_H_ */
