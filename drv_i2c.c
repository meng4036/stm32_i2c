#include "board.h"
#include <stdbool.h>
#include "timing.h"
#include "drv_i2c.h"
#include "stm32f10x.h"
#include "drv_systick.h"

/* --EV7_2 */
#define  I2C_EVENT_MASTER_BYTE_BTF                    ((uint32_t)0x00030004)  /* BUSY, MSL and BTF flags */

void i2c_unstick(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    uint8_t i;

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;

    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7);

    for (i = 0; i < 8; i++)
    {
        while (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6))               // Wait for any clock stretching to finish
            delay_ms(1);

        GPIO_ResetBits(GPIOB, GPIO_Pin_6);                              //Set bus low
        delay_ms(1);

        GPIO_SetBits(GPIOB, GPIO_Pin_6);                                //Set bus high
        delay_ms(1);
    }

    // Generate a start then stop condition

    GPIO_ResetBits(GPIOB, GPIO_Pin_7);                                  //Set bus data low
    delay_ms(1);
    GPIO_ResetBits(GPIOB, GPIO_Pin_6);                                  //Set bus scl low
    delay_ms(1);
    GPIO_SetBits(GPIOB, GPIO_Pin_6);                                    //Set bus scl high
    delay_ms(1);
    GPIO_SetBits(GPIOB, GPIO_Pin_7);                                    //Set bus sda high
    delay_ms(1);
}

///////////////////////////////////////////////////////////////////////////////
int8_t i2c_write(uint8_t slave_addr, uint8_t offset, uint8_t *buffer, uint8_t len)
{
	volatile uint8_t a;
	uint8_t i = 0;
	
	/* Send START condition */
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send MPU6050 address for write */
	I2C_Send7bitAddress(I2C1, slave_addr, I2C_Direction_Transmitter);

	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	
	/* Send the MPU6050's internal address to write to */
	I2C_SendData(I2C1, offset);

	/* Test on EV8_2 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	for (i=0; i<len; i++) {
		/* Send the byte to be written */
		I2C_SendData(I2C1, buffer[i]);

		/* Test on EV8_2 and clear it */
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	}

	/* Send STOP condition */
	I2C_GenerateSTOP(I2C1, ENABLE);
	
	return 0;
}

int8_t i2c_writebyte(uint8_t slave_addr, uint8_t ch, uint8_t offset)
{
	uint8_t tmp = ch;
	
	return i2c_write(slave_addr, offset, &tmp, 1);
}


///////////////////////////////////////////////////////////////////////////////

/* 拉低SCL */
static __inline void I2C_PullDownSCL(void)
{
	GPIOB->BRR = GPIO_Pin_6;
	GPIOB->CRL &= ~0x8000000;
}

/* 释放SCL */
static __inline void I2C_ReleaseSCL(void)
{
	GPIOB->CRL |= 0x8000000;
}

int8_t i2c_read(uint8_t slave_addr, uint8_t offset, uint8_t *buffer, uint8_t len)
{
	volatile uint8_t a;
	uint8_t i = 0;
	
	/* While the bus is busy */
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

	/* Send START condition */
	I2C_GenerateSTART(I2C1, ENABLE);

	/* Test on EV5 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	/* Send MPU6050 address for write */
	I2C_Send7bitAddress(I2C1, slave_addr, I2C_Direction_Transmitter); 

	/* Test on EV6 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	/* Clear EV6 by setting again the PE bit */
	I2C_Cmd(I2C1, ENABLE);

	/* Send the MPU6050's internal address to write to */
	I2C_SendData(I2C1, offset);

	/* Test on EV8_2 and clear it */
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_Cmd(I2C1, ENABLE);

	/* Send STRAT condition a second time */
	I2C_GenerateSTART(I2C1, ENABLE);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	
	/* Send MPU6050 address for read */
	I2C_Send7bitAddress(I2C1, slave_addr, I2C_Direction_Receiver);

	if (len == 1) {
		
		/* waiting EV6 */
		while (!I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR));
		
		I2C_PullDownSCL();
		
		/* Disable Acknowledgement */
		I2C_AcknowledgeConfig(I2C1, DISABLE);
			
		/* Clear ADDR */
		__DMB();
		a = I2C1->SR1;
		a = I2C1->SR2;
		__DMB();
		
		/* Send STOP Condition */
		I2C_GenerateSTOP(I2C1, ENABLE);
		
		I2C_ReleaseSCL();
		
		while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
		buffer[i++] = I2C_ReceiveData(I2C1);
		
		return 0;
	} 
	else if (len == 2) {
		/* waiting EV6 */
		while (!I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR));

		/* set POS and ACK */
		I2C_AcknowledgeConfig(I2C1, ENABLE);
		I2C1->CR1 |= 0x0800;

		/* Pull down SCL */
		I2C_PullDownSCL();

		__DMB();
		a = I2C1->SR1;
		a = I2C1->SR2;
		__DMB();
		I2C_AcknowledgeConfig(I2C1, DISABLE);
		/* Release SCL */
		I2C_ReleaseSCL();			
		
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_BTF));
		
		/* Pull down SCL */
		I2C_PullDownSCL();
		
		I2C1->CR1 &= ~0x0800;
		I2C_AcknowledgeConfig(I2C1, ENABLE);
		I2C_GenerateSTOP(I2C1, ENABLE);
		buffer[i++] = I2C_ReceiveData(I2C1);
		
		I2C_ReleaseSCL();
		
		buffer[i++] = I2C_ReceiveData(I2C1);
		len -=2;
		
		return 0;
	}
	else if (len > 2) {
		/* Test on EV6 and clear it */
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
		
		while (len > 3) {
			while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_BTF));
			buffer[i++] = I2C_ReceiveData(I2C1);
			len --;
		}
		
		/* three bytes */
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_BTF));
		/* Pull down SCL */
		I2C_PullDownSCL();
		I2C_AcknowledgeConfig(I2C1, DISABLE);
		/* Release SCL */
		I2C_ReleaseSCL();

		buffer[i++] = I2C_ReceiveData(I2C1);
		len --;	
		
		/* two bytes */
		while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_BTF));
		
		/* Pull down SCL */
		I2C_PullDownSCL();
		
		I2C_AcknowledgeConfig(I2C1, ENABLE);
		I2C_GenerateSTOP(I2C1, ENABLE);
		buffer[i++] = I2C_ReceiveData(I2C1);
		
		I2C_ReleaseSCL();
		
		buffer[i++] = I2C_ReceiveData(I2C1);
		len -=2;
	}
	
	return 0;
}


int8_t stm32_i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t len, uint8_t *data)
{
	return i2c_write(slave_addr, reg_addr, data, len);
}

int8_t stm32_i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t len, uint8_t *data)
{	
	/* Okay */
	return 0;
}

