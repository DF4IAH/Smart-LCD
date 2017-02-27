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
#include "main.h"

#include "lcd.h"

extern float				g_temp;

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

	PORTD = 0xff;													// Enable pull-ups (when bus-drivers are disabled)
	DDRD  = 0x00;													// Disable bus-drivers
	ioport_set_pin_level(LCD_CD, false);							// Select command-interface
	ioport_set_pin_level(LCD_RW, true);								// Bus-read
	ioport_set_pin_level(LCD_EN, true);								// Bus-enable
	ioport_set_pin_level(LCD_EN, false);							// Bus-disable
	data = PIND;													// Access needs 50ns: therefore take 2 cycles with 33ns each

	cpu_irq_restore(flags);

	return data;
}

void lcd_bus_write_cmd(uint8_t cmd)
{
	irqflags_t flags;

	flags = cpu_irq_save();

	PORTD = cmd;													// Data to be written
	DDRD  = 0xff;													// Enable bus-drivers
	ioport_set_pin_level(LCD_CD, false);							// Select command-interface
	ioport_set_pin_level(LCD_RW, false);							// Bus-write
	ioport_set_pin_level(LCD_EN, true);								// Bus-enable
	ioport_set_pin_level(LCD_EN, false);							// Bus-disable

	cpu_irq_restore(flags);
}

void lcd_bus_write_ram(uint8_t data)
{
	irqflags_t flags;

	flags = cpu_irq_save();

	PORTD = data;													// Data to be written
	DDRD  = 0xff;													// Enable bus-drivers
	ioport_set_pin_level(LCD_CD, true);								// Select RAM-interface
	ioport_set_pin_level(LCD_RW, false);							// Bus-write
	ioport_set_pin_level(LCD_EN, true);								// Bus-enable
	ioport_set_pin_level(LCD_EN, false);							// Bus-disable

	cpu_irq_restore(flags);
}

uint8_t lcd_bus_read_ram(void)
{
	irqflags_t flags;
	uint8_t data;

	flags = cpu_irq_save();

	PORTD = 0xff;													// Enable pull-ups (when bus-drivers are disabled)
	DDRD  = 0x00;													// Disable bus-drivers
	ioport_set_pin_level(LCD_CD, true);								// Select RAM-interface
	ioport_set_pin_level(LCD_RW, true);								// Bus-read
	ioport_set_pin_level(LCD_EN, true);								// Bus-enable
	ioport_set_pin_level(LCD_EN, false);							// Bus-disable

	if (s_lcd_ram_read_nonvalid) {
		data = PIND;												// Discard void data from pipeline
		ioport_set_pin_level(LCD_EN, true);							// Bus-enable
		ioport_set_pin_level(LCD_EN, false);						// Bus-disable
	}
	data = PIND;													// Access needs 50ns: therefore take 2 cycles with 33ns each

	s_lcd_ram_read_nonvalid = false;								// since here read returns valid data
	cpu_irq_restore(flags);

	return data;
}


uint8_t lcd_bounds_x(int x)
{
	if (x < 0) {
		return 0;
	} else if (x >= GFX_MONO_LCD_WIDTH) {
		return (uint8_t) GFX_MONO_LCD_WIDTH - 1;
	} else {
		return (uint8_t) x;
	}
}

uint8_t lcd_bounds_y(int y)
{
	if (y < 0) {
		return 0;
		} else if (y >= GFX_MONO_LCD_HEIGHT) {
		return (uint8_t) GFX_MONO_LCD_HEIGHT - 1;
		} else {
		return (uint8_t) y;
	}
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


static void s_lcd_test_lines(void)
{
	const int oy = 10;
	const int h = 18;
	const int w = GFX_MONO_LCD_WIDTH;

	static int loop = 0;
	static uint8_t sw = 0;

	if (loop++ < h) {
		uint8_t y11 = oy + loop;
		uint8_t y12 = oy + h - loop - 1;
		gfx_mono_generic_draw_line (0, y11, w - 1, y12, sw % 3);

	} else if (loop < (h + w)) {
		uint8_t x21 = (loop - h);
		uint8_t x22 = w - (loop - h) - 1;
		gfx_mono_generic_draw_line (x21, oy, x22, oy + h - 1, sw % 3);

	} else {
		loop = 0;
		if (++sw >= 3) {
			sw = 0;
		}
	}
}

static void s_lcd_test_temp(void)
{
	char	buf[7];
	float	t;

	s_task();

	irqflags_t flags = cpu_irq_save();
	t = g_temp;
	cpu_irq_restore(flags);

	if (t < 0.f) {
		t = 0.f;
	}

	buf[0] = '0' + (uint8_t)(((int)(t /  10.f)) % 10);
	buf[1] = '0' + (uint8_t)(((int) t         ) % 10);
	buf[2] = ',';
	buf[3] = '0' + (uint8_t)(((int)(t *  10.f)) % 10);
	buf[4] = '0' + (uint8_t)(((int)(t * 100.f)) % 10);
	buf[5] = 'C';
	buf[6] = 0;

	gfx_mono_draw_string(buf, 120, 65, &sysfont);
}

static void s_lcd_animation(void)
{
	const int cnt = 4;
	const int blank_len = 1 + 15 + cnt * (3 + 8) + 1;  // train length + front + rear spacer
	int		origin = -blank_len;
	int		i = 0;
	int8_t  dx = 1;
	uint8_t	train_left[blank_len];
	uint8_t	train_right[blank_len];

	/* prepare trains */	
	{
		int idx;

		for (idx = 0; idx < blank_len; ++idx) {
			if (!idx) {
				train_left[idx] = 0;

			} else if (idx < 3) {
				train_left[  idx] = 0b11111000;
				train_left[++idx] = 0b11011000;
			} else if (idx <  5) {
				train_left[  idx] = 0b11011111;
			} else if (idx < 11) {
				train_left[  idx] = 0b11011000;
			} else if (idx < 16) {
				train_left[  idx] = 0b11111000;

			} else if (idx == blank_len - 1) {
				train_left[  idx] = 0;

			} else if (!((idx - 16) % 11)) {
				train_left[  idx] = 0b01000000;
				train_left[++idx] = 0b01000000;
				train_left[++idx] = 0b01000000;
				train_left[++idx] = 0b11111000;
				train_left[++idx] = 0b11111000;
				train_left[++idx] = 0b11001000;
				train_left[++idx] = 0b11111000;
				train_left[++idx] = 0b11111000;
				train_left[++idx] = 0b11001000;
				train_left[++idx] = 0b11111000;
				train_left[++idx] = 0b11111000;
			} else {
				train_left[idx] = 0;
			}
		}

		for (idx = 0; idx < blank_len; ++idx) {
			train_right[blank_len - idx - 1] = train_left[idx];		// x-mirror
		}
	}

	gfx_mono_generic_draw_filled_rect(0, (GFX_MONO_LCD_PAGES - 1) * GFX_MONO_LCD_PIXELS_PER_BYTE, GFX_MONO_LCD_WIDTH, GFX_MONO_LCD_PIXELS_PER_BYTE, GFX_PIXEL_CLR);

	do {
		if (!(i++ % 8)) {
			origin += dx;

			if (origin <= (-10 - blank_len)) {
				dx = 1;
			} else if (origin >= (GFX_MONO_LCD_WIDTH + 10)) {
				dx = -1;
			}

			if (dx < 0) {
				// Draw train left
				if (origin >= 0 && origin < GFX_MONO_LCD_WIDTH) {
					gfx_mono_lcd_uc1608_put_page(train_left, GFX_MONO_LCD_PAGES - 1, origin, blank_len);				// full width
				} else if (-blank_len < origin && origin < 0) {
					gfx_mono_lcd_uc1608_put_page(train_left - origin, GFX_MONO_LCD_PAGES - 1, 0, blank_len + origin);	// left: reduced width
				}

			} else {
				// Draw train right
				if (origin >= 0 && origin < GFX_MONO_LCD_WIDTH) {
					gfx_mono_lcd_uc1608_put_page(train_right, GFX_MONO_LCD_PAGES - 1, origin, blank_len);				// full width
				} else if (-blank_len < origin && origin < 0) {
					gfx_mono_lcd_uc1608_put_page(train_right - origin, GFX_MONO_LCD_PAGES - 1, 0, blank_len + origin);	// left: reduced width
				}
			}
		}

		if (!(i % 100)) {
			s_lcd_test_temp();
		}

		s_lcd_test_lines();
	} while(true);
}

static void s_lcd_test(void)
{
#if 1
	// TEST 1
	for (int i = 0; i < GFX_MONO_LCD_WIDTH; ++i) {
		lcd_bus_write_ram(i);
	}
#endif

#if 0
	// TEST 2
	lcd_page_set(2);
	lcd_cr();
	for (int i = 0, pos = 231; i < GFX_MONO_LCD_WIDTH; ++i, ++pos) {
		if (!(i % 7)) {
			lcd_bus_write_ram(0);
		}
		lcd_bus_write_ram(PROGMEM_READ_BYTE(&(sysfont_glyphs[pos])));
	}
#endif

#if 0
	// TEST 3
	gfx_mono_draw_pixel(4 + 0, 40 + 0, GFX_PIXEL_SET);
	gfx_mono_draw_pixel(4 + 1, 40 + 1, GFX_PIXEL_SET);
	gfx_mono_draw_pixel(4 + 2, 40 + 2, GFX_PIXEL_SET);
	gfx_mono_draw_pixel(4 + 3, 40 + 3, GFX_PIXEL_SET);
	gfx_mono_draw_pixel(4 + 2, 40 + 4, GFX_PIXEL_SET);
	gfx_mono_draw_pixel(4 + 1, 40 + 5, GFX_PIXEL_SET);
	gfx_mono_draw_pixel(4 + 2, 40 + 6, GFX_PIXEL_SET);
	gfx_mono_draw_pixel(4 + 3, 40 + 7, GFX_PIXEL_SET);
	gfx_mono_draw_pixel(4 + 4, 40 + 8, GFX_PIXEL_SET);
#endif

#if 0
	// TEST 4
	gfx_mono_generic_draw_line (0, 16, 239, 31, GFX_PIXEL_SET);
#endif

#if 1
	// TEST 5
	gfx_mono_generic_draw_rect(        70, 48, 40, 40, GFX_PIXEL_SET);
	gfx_mono_generic_draw_filled_rect(170, 48, 40, 40, GFX_PIXEL_SET);
#endif

#if 1
	// TEST 6
	gfx_mono_generic_draw_circle(       10, 80, 10, GFX_PIXEL_SET, GFX_WHOLE);
	gfx_mono_generic_draw_filled_circle(40, 80, 10, GFX_PIXEL_SET, GFX_WHOLE);
#endif

#if 1
	// TEST 7
	gfx_mono_draw_string("DF4IAH Smart-LCD", 70, 32, &sysfont);
#endif

#if 1
	// TEST 8
	s_lcd_animation();
#endif
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

		s_lcd_test();		
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
