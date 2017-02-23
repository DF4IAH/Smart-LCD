/**
 * \file
 *
 * \brief LCD module
 *
 */

/**
 * \mainpage LCD
 *
 * \par LCD communication and command module
 *
 * This module communicates with the LCD device via a 8 bit wide databus.
 * The initialization and a command-set is managed by this module. See
 * specification "UC1608_20041104" for details.
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>
#include "gfx_mono/sysfont.h"

#include "lcd.h"


static uint8_t s_lcd_ram_read_nonvalid = 0;

// hold a copy of a font size in the PROG memory section
SYSFONT_DEFINE_GLYPHS;


uint8_t lcd_bus_read_status(void)
{
	uint8_t data;
	irqflags_t flags = cpu_irq_save();

	PORTD = 0x00;													// Disable pull-ups
	DDRD  = 0x00;													// Disable bus-drivers
	ioport_set_pin_level(LCD_CD, false);							// Select command-interface
	ioport_set_pin_level(LCD_RW, true);								// Bus-read
	ioport_set_pin_level(LCD_EN, true);								// Bus-enable
	barrier();

	nop();
	data = PIND;													// Access needs 50ns: therefore take 2 cycles with 33ns each
	ioport_set_pin_level(LCD_EN, false);							// Bus-disable
	
	cpu_irq_restore(flags);
	
	return data;
}

void lcd_bus_wait_ready(void)
{
	irqflags_t flags;
	uint8_t data;
	
	data = lcd_bus_read_status();
	barrier();

	flags = cpu_irq_save();

	while (data & C_LCD_STATUS_M) {
		ioport_set_pin_level(LCD_EN, true);							// Bus-enable
		cpu_irq_restore(flags);
		barrier();

		nop();
		flags = cpu_irq_save();
		data = PIND;
		ioport_set_pin_level(LCD_EN, false);						// Bus-disable
	}

	cpu_irq_restore(flags);
}

void lcd_bus_write_cmd(uint8_t cmd)
{
	irqflags_t flags;

	lcd_bus_wait_ready();											// Wait until device is ready for next bus access
	
	flags = cpu_irq_save();

	PORTD = cmd;													// Data to be written
	DDRD  = 0xff;													// Enable bus-drivers
	ioport_set_pin_level(LCD_CD, false);							// Select command-interface
	ioport_set_pin_level(LCD_RW, false);							// Bus-write
	ioport_set_pin_level(LCD_EN, true);								// Bus-enable
	barrier();

	ioport_set_pin_level(LCD_EN, false);							// Bus-disable

	cpu_irq_restore(flags);
}

void lcd_bus_write_ram(uint8_t data)
{
	irqflags_t flags;

	lcd_bus_wait_ready();											// Wait until device is ready for next bus access
	
	flags = cpu_irq_save();

	PORTD = data;													// Data to be written
	DDRD  = 0xff;													// Enable bus-drivers
	ioport_set_pin_level(LCD_CD, true);								// Select RAM-interface
	ioport_set_pin_level(LCD_RW, false);							// Bus-write
	ioport_set_pin_level(LCD_EN, true);								// Bus-enable
	barrier();

	ioport_set_pin_level(LCD_EN, false);							// Bus-disable

	cpu_irq_restore(flags);
}

uint8_t lcd_bus_read_ram(void)
{
	irqflags_t flags;
	uint8_t data;

	lcd_bus_wait_ready();											// Wait until device is ready for next bus access
	
	flags = cpu_irq_save();
	
	PORTD = 0x00;													// Disable pull-ups
	DDRD  = 0x00;													// Disable bus-drivers
	ioport_set_pin_level(LCD_CD, true);								// Select RAM-interface
	ioport_set_pin_level(LCD_RW, true);								// Bus-read
	ioport_set_pin_level(LCD_EN, true);								// Bus-enable
	barrier();

	nop();
	if (s_lcd_ram_read_nonvalid) {
		data = PIND;												// discard void data from pipeline
		ioport_set_pin_level(LCD_EN, false);						// Bus-disable
		barrier();
		ioport_set_pin_level(LCD_EN, true);							// Bus-enable
		s_lcd_ram_read_nonvalid = false;							// since here read returns valid data
	}
	data = PIND;													// Access needs 50ns: therefore take 2 cycles with 33ns each
	ioport_set_pin_level(LCD_EN, false);							// Bus-disable

	cpu_irq_restore(flags);

	return data;
}


uint8_t lcd_init(void)
{
	uint8_t data;
	
	/* initial settings */
	ioport_set_pin_level(LCD_CS, false);							// Device-unselected
	ioport_set_pin_level(LCD_EN, false);							// Bus-disable
	delay_us(1);													// Stabilize bus
	ioport_set_pin_level(LCD_CS, true);								// Device-selected
	delay_us(1);													// Stabilize bus
	
	/* INIT sequence */
	lcd_bus_write_cmd(0b11100010);									// Reset display
	delay_ms(20);													// Wait for the panel to get ready in case one is attached to the bus
	
	data = lcd_bus_read_status();									// Get current status
	if (!(data & C_LCD_STATUS_M)) {
		/* LCD panel reacts correctly - resume with INIT sequence */
		lcd_bus_write_cmd(0b00101000 | C_LCD_PWR_CTRL);				// Set Power Control
		lcd_bus_write_cmd(0b00100000 | C_LCD_MR_TC);				// Set MR and TC
		lcd_bus_write_cmd(0b11101000 | C_LCD_BIASRATIO);			// Set Bias Ratio
		lcd_bus_write_cmd(0b10000001);								// Set Gain and PM (A)
		lcd_bus_write_cmd(C_LCD_GAIN_PM);							// Set Gain and PM (B)
		lcd_bus_write_cmd(0b11000000 | C_LCD_MAPPING);				// Set Mapping
		lcd_bus_write_cmd(0b10000100 | C_LCD_AC);					// Set RAM Address Control
		lcd_bus_write_cmd(0b10010000);								// Set Fixed Lines (0)
		lcd_bus_write_cmd(0b01000000);								// Set Start Line (0)
		lcd_bus_write_cmd(0b11101110);								// Reset Cursor Mode (destroys CA value)
		lcd_bus_write_cmd(0b10110000);								// Set Page Address (0)
		lcd_bus_write_cmd(0b00000000);								// Set Column Address LSB (0)
		lcd_bus_write_cmd(0b00010000);								// Set Column Address MSB (0)
		s_lcd_ram_read_nonvalid = true;
		lcd_bus_write_cmd(0b11101111);								// Set Cursor Mode
		lcd_bus_write_cmd(0b11101110);								// Reset Cursor Mode (now CR := CA)
		lcd_bus_write_cmd(0b10100100);								// Disable DC[1] (all pixel on)
		lcd_bus_write_cmd(0b10100110);								// Disable DC[0] (all pixel inverse)
		lcd_bus_write_cmd(0b10101111);								// Enable  DC[2] (Display)
		
		// TEST 1
		for (int i = 0; i < 240; ++i) {
			lcd_bus_write_ram(PROGMEM_READ_BYTE(&(sysfont_glyphs[i])));
		}
		
		// TEST 2
		gfx_mono_draw_string("DF4IAH Smart-LCD", 8, 24, &sysfont);
		
		// TEST 3
		gfx_mono_generic_draw_rect(43, 27, 50, 25, GFX_PIXEL_SET);
		
		// TEST 4
		gfx_mono_generic_draw_filled_circle(97, 70, 24, GFX_PIXEL_SET, GFX_WHOLE);
		
		return 0;													// Return OK
		
	} else {
		return 1;													// Return failure
	}
}

void lcd_shutdown(void)
{
	lcd_bus_write_cmd(0b11100010);									// Reset display
	delay_ms(2);													// Wait for the energy to dissipate
}
