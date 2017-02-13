/**
 * \file
 *
 * \brief TWI / I2C communication module
 *
 */

/**
 * \mainpage TWI / I2C
 *
 * \par TWI / I2C communication module
 *
 * The TWI is compatible to the I2C bus and allows communication on a two-wire
 * bi-directional bus.
 *
 * This module allows to variants: the application is a Slave on the bus for
 * working out any commands given to it. On the other hand this application needs
 * to address a digital pot to steer the LCD contrast on the display. For this
 * to work the application is the Master for the Pot device at the bus.
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

#include "twi.h"

void __vector_24__bottom(uint8_t tws, uint8_t twd)
{
	switch(tws) {
	
	//	default:
	}
}
