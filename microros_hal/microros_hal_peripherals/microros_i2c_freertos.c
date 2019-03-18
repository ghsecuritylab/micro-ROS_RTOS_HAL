#include "microros_i2c_hal.h"


#ifdef freertos
#include "stm32f4xx_hal.h"

I2C_HandleTypeDef hi2c1;

void freertos_i2c_write(uint8_t data, int8_t addr){
  //TODO
}

uint32_t freertos_i2c_read(uint8_t device_addr,uint8_t regaddr,uint8_t len ){



  uint32_t regval;
  int ret;

   ret=HAL_I2C_Master_Transmit (&hi2c1, device_addr<<1,&regaddr, 1, 100);
   if(ret>0){
      //error
  	  printf("Error getreg32 transmit: %i \r\n",ret);
    }
   ret=HAL_I2C_Master_Receive (&hi2c1,device_addr<<1,&regval,len,100);
    if(ret>0){
     printf("Error getreg32 receive: %i \r\n",ret);
    }

   if(len==2){

  	uint16_t msb, lsb;
  	msb = (regval & 0xFF);
  	lsb = (regval & 0xFF00) >> 8;

  	regval = (msb << 8) | lsb;
   }
   else if(len==3){
  	uint16_t byte1, byte2, byte3, byte4;//byte1 is the msb and byte4 is the lsb
  	byte1=(regval & 0xFF);
  	byte2=(regval & 0xFF00) >>8;
  	byte3=(regval & 0xFFFF00) >> 16;
  	byte4=(regval & 0xFFFFFF00) >> 24;

  	regval=(byte1 << 24) | (byte2 << 16) | (byte3 << 8) | byte4;
  }
    	return regval;
}

#endif
