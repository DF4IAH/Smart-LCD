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

extern uint8_t				g_u8_DEBUG11,
							g_u8_DEBUG12,
							g_u8_DEBUG13;

extern uint_fast32_t		g_u32_DEBUG21;

extern float				g_f_DEBUG31;



static uint8_t s_lcd_ram_read_nonvalid = 0;

// hold a copy of a font size in the PROG memory section
SYSFONT_DEFINE_GLYPHS;


uint8_t lcd_bus_read_status(void)
{
	uint8_t data;
	irqflags_t flags = cpu_irq_save();
	
	PORTD = 0xff;													// Enable pull-ups
	DDRD  = 0x00;													// Disable bus-drivers
	ioport_set_pin_level(LCD_CD, false);							// Select command-interface
	ioport_set_pin_level(LCD_RW, true);								// Bus-read
	barrier();
	
	ioport_set_pin_level(LCD_EN, true);								// Bus-enable
	nop();
	barrier();
	
	ioport_set_pin_level(LCD_EN, false);							// Bus-disable
	nop();
	barrier();
	
	data = PIND;													// Access needs 50ns: therefore take 2 cycles with 33ns each
	cpu_irq_restore(flags);
	
	return data;
}

void lcd_bus_wait_ready(void)
{
	irqflags_t flags;
	uint8_t data;
	
	data = lcd_bus_read_status();
	flags = cpu_irq_save();
	
	while (data & C_LCD_STATUS_M) {
		ioport_set_pin_level(LCD_EN, true);							// Bus-enable
		nop();
		barrier();
		
		ioport_set_pin_level(LCD_EN, false);						// Bus-disable
		nop();
		barrier();
		
		data = PIND;												// Access needs 50ns: therefore take 2 cycles with 33ns each
		
		cpu_irq_restore(flags);
		nop();														// Allow to get interrupted
		flags = cpu_irq_save();
	}
	
	cpu_irq_restore(flags);
}

void lcd_bus_write_cmd(uint8_t cmd)
{
	irqflags_t flags;
	
	//lcd_bus_wait_ready();											// Wait until device is ready for next bus access
	flags = cpu_irq_save();
	
	PORTD = cmd;													// Data to be written
	DDRD  = 0xff;													// Enable bus-drivers
	ioport_set_pin_level(LCD_CD, false);							// Select command-interface
	ioport_set_pin_level(LCD_RW, false);							// Bus-write
	ioport_set_pin_level(LCD_EN, true);								// Bus-enable
	nop();
	barrier();
	
	ioport_set_pin_level(LCD_EN, false);							// Bus-disable
	
	cpu_irq_restore(flags);
}

void lcd_bus_write_ram(uint8_t data)
{
	irqflags_t flags;
	
	//lcd_bus_wait_ready();											// Wait until device is ready for next bus access
	flags = cpu_irq_save();
	
	PORTD = data;													// Data to be written
	DDRD  = 0xff;													// Enable bus-drivers
	ioport_set_pin_level(LCD_CD, true);								// Select RAM-interface
	ioport_set_pin_level(LCD_RW, false);							// Bus-write
	barrier();
	
	ioport_set_pin_level(LCD_EN, true);								// Bus-enable
	nop();
	barrier();
	
	ioport_set_pin_level(LCD_EN, false);							// Bus-disable
	nop();
	
	cpu_irq_restore(flags);
}

uint8_t lcd_bus_read_ram(void)
{
	irqflags_t flags;
	uint8_t data;
	
	//lcd_bus_wait_ready();											// Wait until device is ready for next bus access
	flags = cpu_irq_save();
	
	PORTD = 0xff;													// Enable pull-ups
	DDRD  = 0x00;													// Disable bus-drivers
	ioport_set_pin_level(LCD_CD, true);								// Select RAM-interface
	ioport_set_pin_level(LCD_RW, true);								// Bus-read
	ioport_set_pin_level(LCD_EN, true);								// Bus-enable
	nop();
	barrier();

	ioport_set_pin_level(LCD_EN, false);							// Bus-disable
	nop();
	barrier();
	
	if (s_lcd_ram_read_nonvalid) {
		data = PIND;												// Discard void data from pipeline
		
		ioport_set_pin_level(LCD_EN, true);							// Bus-enable
		nop();
		barrier();
		
		ioport_set_pin_level(LCD_EN, false);						// Bus-disable
		nop();
		s_lcd_ram_read_nonvalid = false;							// since here read returns valid data
		barrier();
	}
	
	data = PIND;													// Access needs 50ns: therefore take 2 cycles with 33ns each
	cpu_irq_restore(flags);
	
	return data;
}


void lcd_page_set(uint8_t page)
{
	if ((0 <= page && page) < (GFX_MONO_LCD_PAGES)) {
		lcd_bus_write_cmd(0b10110000 | page);					// Set Page Address

		s_lcd_ram_read_nonvalid = 1;
	}
}

void lcd_col_set(uint8_t col)
{
	if ((0 <= col) && (col < GFX_MONO_LCD_WIDTH)) {
		lcd_bus_write_cmd(0b00000000 | ( col       & 0x0f));	// Set Column Address LSB
		lcd_bus_write_cmd(0b00010000 | ((col >> 4) & 0x0f));	// Set Column Address MSB
		
		s_lcd_ram_read_nonvalid = 1;
	}
}

void lcd_cr(void)
{
	lcd_bus_write_cmd(0b00000000);								// Set Column Address LSB (0)
	lcd_bus_write_cmd(0b00010000);								// Set Column Address MSB (0)

	s_lcd_ram_read_nonvalid = 1;
}

void lcd_home(void)
{
	lcd_bus_write_cmd(0b10110000);								// Set Page Address (0)
	lcd_cr();
}

void lcd_cls(void)
{
	/* Blank LCD RAM */
	for (uint8_t page = 0; page < GFX_MONO_LCD_PAGES; ++page) {
		lcd_bus_write_cmd(0b10110000 | page);						// Set Page Address
		lcd_bus_write_cmd(0b00000000);								// Set Column Address LSB (0)
		lcd_bus_write_cmd(0b00010000);								// Set Column Address MSB (0)
		
		for (uint8_t cnt = GFX_MONO_LCD_WIDTH; cnt; --cnt) {		// clear all columns of that page
			lcd_bus_write_ram(0);
		}
	}	
	
	/* Set cursor to home position */
	lcd_home();
}


uint8_t lcd_init(void)
{
	uint8_t data;
	
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
		lcd_bus_write_cmd(0b10001000 | C_LCD_AC);					// Set RAM Address Control

		lcd_bus_write_cmd(0b01000000);								// Set Start Line (0)
		lcd_bus_write_cmd(0b10010000);								// Set Fixed Lines (0)
		
		lcd_bus_write_cmd(0b10110000);								// Set Page Address (0)
		lcd_bus_write_cmd(0b00000000);								// Set Column Address LSB (0)
		lcd_bus_write_cmd(0b00010000);								// Set Column Address MSB (0)
		s_lcd_ram_read_nonvalid = true;

		lcd_bus_write_cmd(0b11101111);								// Set Cursor Mode
		lcd_bus_write_cmd(0b11101110);								// Reset Cursor Mode (now CR := CA)
		lcd_bus_write_cmd(0b10100100);								// Disable DC[1] (all pixel on)
		lcd_bus_write_cmd(0b10100110);								// Disable DC[0] (all pixel inverse)
		lcd_bus_write_cmd(0b10101111);								// Enable  DC[2] (Display)
		
		lcd_cls();													// Clear screen
		
		
		// TEST 1
		for (int i = 0; i < GFX_MONO_LCD_WIDTH; ++i) {
			lcd_bus_write_ram(i);
		}
		
		// TEST 2
		lcd_page_set(2);
		lcd_cr();
		for (int i = 0, pos = 231; i < GFX_MONO_LCD_WIDTH; ++i, ++pos) {
			if (!(i % 7)) {
				lcd_bus_write_ram(0);
			}
			lcd_bus_write_ram(PROGMEM_READ_BYTE(&(sysfont_glyphs[pos])));
		}
		
		// TEST 3
		gfx_mono_draw_pixel(32 + 0, 64 + 0, GFX_PIXEL_SET);
		gfx_mono_draw_pixel(32 + 1, 64 + 1, GFX_PIXEL_SET);
		gfx_mono_draw_pixel(32 + 2, 64 + 2, GFX_PIXEL_SET);
		gfx_mono_draw_pixel(32 + 3, 64 + 3, GFX_PIXEL_SET);
		gfx_mono_draw_pixel(32 + 4, 64 + 4, GFX_PIXEL_SET);
		gfx_mono_draw_pixel(32 + 5, 64 + 5, GFX_PIXEL_SET);
		gfx_mono_draw_pixel(32 + 6, 64 + 6, GFX_PIXEL_SET);
		gfx_mono_draw_pixel(32 + 7, 64 + 7, GFX_PIXEL_SET);
		gfx_mono_draw_pixel(32 + 8, 64 + 8, GFX_PIXEL_SET);
		
		// TEST 4
		gfx_mono_generic_draw_line (0, 0, 127, 127, GFX_PIXEL_SET);
		
		// TEST 5
		gfx_mono_generic_draw_rect(        70, 60, 50, 50, GFX_PIXEL_SET);
		gfx_mono_generic_draw_filled_rect(170, 60, 50, 50, GFX_PIXEL_SET);
		
		// TEST 6
		gfx_mono_generic_draw_circle(       10, 104, 10, GFX_PIXEL_SET, GFX_WHOLE);
		gfx_mono_generic_draw_filled_circle(40, 104, 10, GFX_PIXEL_SET, GFX_WHOLE);

		// TEST 7
		gfx_mono_draw_string("DF4IAH Smart-LCD", 60, 42, &sysfont);
		
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
