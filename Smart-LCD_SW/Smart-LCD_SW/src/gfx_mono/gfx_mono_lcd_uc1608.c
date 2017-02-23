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
#include "gfx_mono_lcd_uc1608.h"

/**
 * \ingroup gfx_mono_lcd_uc1608
 * @{
 */


/**
 * \brief gfx_mono_lcd_uc1608_put_page
 */
void gfx_mono_lcd_uc1608_put_page(gfx_mono_color_t *data, gfx_coord_t page, gfx_coord_t page_offset, gfx_coord_t width)
{
	
}

/**
 * \brief gfx_mono_lcd_uc1608_get_page
 */
void gfx_mono_lcd_uc1608_get_page(gfx_mono_color_t *data, gfx_coord_t page, gfx_coord_t page_offset, gfx_coord_t width)
{
	
}

/**
 * \brief gfx_mono_lcd_uc1608_draw_pixel
 */
void gfx_mono_lcd_uc1608_draw_pixel(gfx_coord_t x, gfx_coord_t y, gfx_mono_color_t color)
{
	
}

/**
 * \brief gfx_mono_lcd_uc1608_get_pixel
 */
uint8_t gfx_mono_lcd_uc1608_get_pixel(gfx_coord_t x, gfx_coord_t y)
{
	return 0;
}

/**
 * \brief gfx_mono_lcd_uc1608_put_byte
 */
void gfx_mono_lcd_uc1608_put_byte(gfx_coord_t page, gfx_coord_t column, uint8_t data)
{
	
}

/**
 * \brief gfx_mono_lcd_uc1608_get_byte
 */
uint8_t gfx_mono_lcd_uc1608_get_byte(gfx_coord_t page, gfx_coord_t column)
{
	return 0;
}

/**
 * \brief gfx_mono_lcd_uc1608_mask_byte
 */
void gfx_mono_lcd_uc1608_mask_byte(gfx_coord_t page, gfx_coord_t column, gfx_mono_color_t pixel_mask, gfx_mono_color_t color)
{
	
}

/** @} */
