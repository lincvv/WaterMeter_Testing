/*
 * I2C2.h
 *
 *  Created on: 15 ���. 2018 �.
 *      Author: vlinc
 */

#ifndef MY_I2C_H_
#define MY_I2C_H_

//#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"

#define DS3231_WRITE_ADDR 0xD0
#define DS3231_READ_ADDR 0xD1



uint8_t hal_write_i2c(uint8_t slave_addr, uint8_t addr, uint8_t data);
uint8_t hal_read_i2c(uint8_t slave_addr, uint8_t addr, uint8_t size, uint8_t *data);

void write_i2c(uint8_t slave_addr, uint8_t addr, uint8_t size, uint8_t *data);
void read_i2c(uint8_t slave_addr, uint8_t addr, uint8_t size, uint8_t *data);


#endif /* MY_I2C_H_ */




