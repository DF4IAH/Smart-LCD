/*
 * twi.h
 *
 * Created: 13.02.2017 08:48:44
 *  Author: DF4IAH
 */ 


#ifndef TWI_H_
#define TWI_H_


#define TWI_SLAVE_ADDR		0x22
#define TWI_SLAVE_ADDR_BM   0b0000000

#define TWI_SLAVE_ADDR_GCE	0b1


uint8_t __vector_24__bottom(uint8_t tws, uint8_t twd, uint8_t twcr_cur);


#endif /* TWI_H_ */
