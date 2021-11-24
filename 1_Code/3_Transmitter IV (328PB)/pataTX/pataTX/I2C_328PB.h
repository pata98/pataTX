/*
 * pata_I2C328PB.h
 *
 * Created: 2021-09-14 오전 1:01:45
 *  Author: yoopata
 */ 


#ifndef I2C_328PB_H_
#define I2C_328PB_H_

#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#include <stdint.h>

void i2c_init();
uint8_t i2c_start(uint8_t slave_address);
uint8_t i2c_write(uint8_t data);
void i2c_stop();



#endif /* PATA_I2C328PB_H_ */