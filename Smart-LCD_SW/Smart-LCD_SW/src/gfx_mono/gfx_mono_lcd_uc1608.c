/**
 * \file
 *
 * \brief Monochrome graphic library for the UC1608 LCD controller
 *
 * Copyright (c) 2011-2017 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include "lcd.h"

#include "gfx_mono_lcd_uc1608.h"


/**
 * \ingroup gfx_mono_lcd_uc1608
 * @{
 */


/**
 * \brief Put a page from RAM to the LCD controller memory
 *
 * \param data Pointer to data to be written
 * \param page Page address
 * \param column Offset into page (x coordinate)
 * \param width Number of bytes to be written.
 */
void gfx_mono_lcd_uc1608_put_page(gfx_mono_color_t *data, gfx_coord_t page, gfx_coord_t page_offset, gfx_coord_t width)
{
	if (data &&
		(page					<  GFX_MONO_LCD_PAGES) && 
	    (page_offset			<  GFX_MONO_LCD_WIDTH)) {

		gfx_mono_color_t *data_pt = data;

		lcd_page_set(page);
		lcd_col_set(page_offset);

		if (page_offset + width > GFX_MONO_LCD_WIDTH) {
			width = GFX_MONO_LCD_WIDTH - page_offset;
		}

		for (uint8_t cnt = width; cnt; --cnt) {
			lcd_bus_write_ram(*(data_pt++));								// Write byte slice to LCD panel
		}
	}
}

/**
 * \brief Read a page from the LCD controller memory into the RAM
 *
 * \param data   Pointer where to store the read data
 * \param page   Page address
 * \param column Offset into page (x coordinate)
 * \param width  Number of bytes to be read.
 */
void gfx_mono_lcd_uc1608_get_page(gfx_mono_color_t *data, gfx_coord_t page, gfx_coord_t page_offset, gfx_coord_t width)
{
	if (data &&
	(page					<  GFX_MONO_LCD_PAGES) &&
	(page_offset			<  GFX_MONO_LCD_WIDTH)) {

		gfx_mono_color_t *data_pt = data;

		lcd_page_set(page);
		lcd_col_set(page_offset);

		if (page_offset + width > GFX_MONO_LCD_WIDTH) {
			width = GFX_MONO_LCD_WIDTH - page_offset;
		}

		for (uint8_t cnt = width; cnt; --cnt) {
			*(data_pt++) = lcd_bus_read_ram();								// Read byte slice from LCD panel
		}
	}
}

/**
 * \brief Draw pixel to LCD controller memory
 *
 * \param x         X coordinate of the pixel
 * \param y         Y coordinate of the pixel
 * \param color     Pixel operation.
 */
void gfx_mono_lcd_uc1608_draw_pixel(gfx_coord_t x, gfx_coord_t y, gfx_mono_color_t color)
{
	if ((x < GFX_MONO_LCD_WIDTH) && (y < GFX_MONO_LCD_HEIGHT)) {
		gfx_coord_t			page		= y / GFX_MONO_LCD_PIXELS_PER_BYTE;
		gfx_mono_color_t	pixel_mask	= 1 << (y % GFX_MONO_LCD_PIXELS_PER_BYTE);

		gfx_mono_lcd_uc1608_mask_byte(page, x, pixel_mask, color);
	}
}

/**
 * \brief Get the pixel value at x,y in the LCD controller memory
 *
 * \param x      X coordinate of pixel
 * \param y      Y coordinate of pixel
 * \return Non zero value if pixel is set.
 */
uint8_t gfx_mono_lcd_uc1608_get_pixel(gfx_coord_t x, gfx_coord_t y)
{
	uint8_t isSet = GFX_PIXEL_CLR;
	
	if ((x < GFX_MONO_LCD_WIDTH) && (y < GFX_MONO_LCD_HEIGHT)) {
		gfx_coord_t			page		= y / GFX_MONO_LCD_PIXELS_PER_BYTE;
		gfx_mono_color_t	pixel_mask	= 1 << (y % GFX_MONO_LCD_PIXELS_PER_BYTE);
		uint8_t				byte = gfx_mono_lcd_uc1608_get_byte(page, x);

		isSet = (byte & pixel_mask) ?  GFX_PIXEL_SET : GFX_PIXEL_CLR;
	}
	return isSet;
}

/**
 * \brief Put a byte to the LCD controller memory
 *
 * \param page   Page address
 * \param column Page offset (x coordinate)
 * \param data   Data to be written.
 */
void gfx_mono_lcd_uc1608_put_byte(gfx_coord_t page, gfx_coord_t column, uint8_t data)
{
	if ((page < GFX_MONO_LCD_PAGES) && (column < GFX_MONO_LCD_WIDTH)) {
		lcd_page_set(page);
		lcd_col_set(column);
		lcd_bus_write_ram(data);										// Write byte slice to RAM
	}
}

/**
 * \brief Get a byte from the LCD controller memory
 *
 * \param page   Page address
 * \param column Page offset (x coordinate)
 * \return       data from LCD controller.
 */
uint8_t gfx_mono_lcd_uc1608_get_byte(gfx_coord_t page, gfx_coord_t column)
{
	uint8_t data = 0;

	if ((page < GFX_MONO_LCD_PAGES) && (column < GFX_MONO_LCD_WIDTH)) {
		lcd_page_set(page);
		lcd_col_set(column);
		data = lcd_bus_read_ram();										// Read byte slice from RAM
	}
	return data;
}

/**
 * \brief Read/Modify/Write a byte in the LCD controller memory
 *
 * This function will read the byte from the LCD controller and
 * do a mask operation on the byte according to the pixel operation selected
 * by the color argument and the pixel mask provided.
 *
 * \param page       Page address
 * \param column     Page offset (x coordinate)
 * \param pixel_mask Mask for pixel operation
 * \param color      Pixel operation
 */
void gfx_mono_lcd_uc1608_mask_byte(gfx_coord_t page, gfx_coord_t column, gfx_mono_color_t pixel_mask, gfx_mono_color_t color)
{
	uint8_t data = 0;

	if ((page < GFX_MONO_LCD_PAGES) && (column < GFX_MONO_LCD_WIDTH)) {
		data = gfx_mono_lcd_uc1608_get_byte(page, column);

		switch (color) {
			case GFX_PIXEL_CLR:
				data &= ~pixel_mask;
				break;

			case GFX_PIXEL_SET:
				data |= pixel_mask;
				break;

			case GFX_PIXEL_XOR:
				data ^= pixel_mask;
				break;
		}

		lcd_col_set(column);
		lcd_bus_write_ram(data);										// Write byte slice to RAM
	}
}

/** @} */
