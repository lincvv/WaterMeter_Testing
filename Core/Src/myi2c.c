/*
 * I2C2.c
 *
 *  Created on: 15 ���. 2018 �.
 *      Author: vlinc
 */
#include "i2c.h"
#include "printf.h"
#include "myi2c.h"

extern I2C_HandleTypeDef hi2c2;
//extern I2C_HandleTypeDef I2C2;

#define hi2c &hi2c2

void read_i2c(uint8_t slave_addr, uint8_t addr, uint8_t size, uint8_t *data)
{
    // I2C Send
    I2C1_Start();
    I2C1_SendAddress(slave_addr, I2C_TRANSMITTER);
    I2C1_SendData(addr);

    // I2C Receive
    uint8_t count = 0;
    I2C1_Start();
    I2C1_SendAddress(slave_addr, I2C_RECEIVER);

    for (count = 0; count < size - 1; ++count ) {
        data[count] = I2C1_ReceiveData(I2C_ACK);
    }

    I2C1_Stop();
    data[count] = I2C1_ReceiveData(I2C_NACK);
}

void write_i2c(uint8_t slave_addr, uint8_t addr, uint8_t size, uint8_t *data)
{
    // I2C Send
    I2C1_Start();
    I2C1_SendAddress(slave_addr, I2C_TRANSMITTER);
    I2C1_SendData(addr);

    for (int8_t i = 0; i < size; ++i) {
        I2C1_SendData(data[i]);
    }

    I2C1_Stop();
}

uint8_t hal_read_i2c(uint8_t slave_addr, uint8_t addr, uint8_t size, uint8_t *data)
{
	HAL_StatusTypeDef statusWrite, statusRead;

	uint8_t read_i2c_data[size];

	read_i2c_data[0]=addr;

    slave_addr <<= 1;
    slave_addr &= ~(1 << 0);

//	statusWrite = HAL_I2C_Master_Transmit_IT(hi2c, (slave_addr - 1), (uint8_t*)&read_i2c_data, 1);
    statusWrite = HAL_I2C_Master_Transmit(&hi2c2, slave_addr , (uint8_t*)&read_i2c_data, 1, 10);

	if (statusWrite == HAL_ERROR) {
		printf("ERROR_Write");

	}
	else if(statusWrite == HAL_OK) {
		printf("OK_Write \n");
	}
    HAL_Delay(150);

    slave_addr |= (1 << 0);

	statusRead = HAL_I2C_Master_Receive(hi2c, slave_addr, read_i2c_data, size, 10);

	if (statusRead != HAL_ERROR) {
        for (uint8_t i = 0; i < size; i++){

            data[i] = read_i2c_data[i];
        }

	} else {
		printf("ERROR_Read");
	}

	return statusRead;

}

uint8_t hal_write_i2c(uint8_t slave_addr, uint8_t addr, uint8_t data)
{
	HAL_StatusTypeDef statusError;
	uint8_t write_i2c_data[2];
	write_i2c_data[0]=addr;
	write_i2c_data[1]=data;

    slave_addr <<= 1;
    slave_addr &= ~(1 << 0);

	statusError = HAL_I2C_Master_Transmit_IT(hi2c, slave_addr, write_i2c_data, 2);
	if(statusError == HAL_ERROR){
		printf("Error! \n");
	}
	else if(statusError == HAL_OK){
		printf("OK_Write \n");
	}

	return statusError;
}
