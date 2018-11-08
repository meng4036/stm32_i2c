#ifndef _DRV_I2C_H
#define _DRV_I2C_H

#include <stdbool.h>
#include <stdint.h>


void i2c_unstick(void);

int8_t i2c_writebyte(uint8_t slave_addr, uint8_t ch, uint8_t offset);

int8_t i2c_write(uint8_t slave_addr, uint8_t offset, uint8_t *buffer, uint8_t len);
int8_t i2c_read(uint8_t slave_addr, uint8_t offset, uint8_t *buffer, uint8_t len);


#endif;