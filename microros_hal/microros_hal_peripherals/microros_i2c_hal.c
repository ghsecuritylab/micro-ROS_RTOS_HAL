
#include "microros_i2c_hal.h"

void uros_i2c_write(uint8_t device_addr,uint8_t regaddr, int8_t regval,uint8_t len ){
  #ifdef nuttx
    nuttx_i2c_write( device_addr,regaddr,regval,len );
  #elif freertos
    freertos_i2c_write( device_addr,regaddr,regval,len);
  #endif
}

uint32_t uros_i2c_read(uint8_t device_addr,uint8_t regaddr,uint8_t len ){
  #ifdef nuttx
    nuttx_i2c_read(device_addr,regaddr,len );
  #elif freertos
    freertos_i2c_read( device_addr, regaddr, len );
  #endif
}

uint32_t uros_i2c_hal_aux(void *aux){
  #ifdef nuttx
    nuttx_i2c_hal_aux(aux);
  #elif freertos
    //nothing to add
  #endif
}
