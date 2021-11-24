/*
 * pata_I2C328PB.c
 *
 * Created: 2021-09-14 오전 1:02:53
 *  Author: yoopata
 */ 

#include "I2C_328PB.h"
#include <avr/io.h>


/**************************************************************/
/*                          I2C                               */
/**************************************************************/
void i2c_init()
{
	TWSR0 = 0;                         // no prescaler
	TWBR0 = ((F_CPU/200000)-16)/2;  // must be > 10 for stable operation
	
}
uint8_t i2c_start(uint8_t slave_address)
{
	// send START condition
	TWCR0 = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR0 & (1<<TWINT)));

	// send device address
	TWDR0 = slave_address;
	TWCR0 = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR0 & (1<<TWINT)));
	
	return 0;

}
uint8_t i2c_write(uint8_t data)
{
	// send data to the previously addressed device
	TWDR0 = data;
	TWCR0 = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR0 & (1<<TWINT)));

	return 0;
	
}
void i2c_stop()
{
	// send stop condition
	TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
	// wait until stop condition is executed and bus released
	while(TWCR0 & (1<<TWSTO));
	
}