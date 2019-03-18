#include "microros_spi_hal.h"
#ifdef freertos

SPI_HandleTypeDef hspi1;

void freertos_spi_write(uint8_t regaddr, int8_t regval,uint8_t len ){
	//Revisar!
	HAL_SPI_Transmit(&hspi1,&regaddr,1,10);//Transmit the register dir that you want to read
	HAL_SPI_Transmit(&hspi1,&regval,len,10);
}

uint32_t freertos_spi_read(uint8_t regaddr,uint8_t len ){
	uint32_t regval;

	HAL_SPI_Transmit(&hspi1,&regaddr,1,10);//Transmit the register dir that you want to read
	HAL_SPI_Receive(&hspi1,&regval,len,10);//Then receive from the sensor the data
	if(len == 2 ){
		uint16_t msb, lsb;

		msb = (regval & 0xFF);
		lsb = (regval & 0xFF00) >> 8;

		regval = (msb << 8) | lsb;
	}
	else if(len==3){
		uint16_t v1, v2,v3;

		v1=(regval & 0xFF);
		v2=(regval & 0xFF00) >>8;
		v3=(regval & 0xFFFF00) >> 16;


		regval = (v1 << 16) | (v2 << 8) | v3;
	}

	return regval;
}

#endif 
