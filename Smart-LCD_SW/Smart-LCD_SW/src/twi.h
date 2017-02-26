/*
 * twi.h
 *
 * Created: 13.02.2017 08:48:44
 *  Author: DF4IAH
 */ 


#ifndef TWI_H_
#define TWI_H_


#define TWI_SLAVE_ADDR		0b0111100
#define TWI_SLAVE_ADDR_BM   0b1111111

#define TWI_SLAVE_ADDR_GCE	0b1


void __vector_24__bottom(uint8_t tws, uint8_t twd);


#endif /* TWI_H_ */
