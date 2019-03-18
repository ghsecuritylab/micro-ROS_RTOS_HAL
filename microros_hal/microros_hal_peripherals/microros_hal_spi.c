
#include "microros_spi_hal.h"

void uros_spi_write(uint8_t regaddr, int8_t regval,uint8_t len ){
  #ifdef nuttx
	nuttx_spi_write( regaddr,  regval, len );
  #elif freertos
    freertos_spi_write( regaddr,  regval, len );
  #endif
}

uint32_t uros_spi_read(uint8_t regaddr,uint8_t len ){
  #ifdef nuttx
	nuttx_spi_read(regaddr, len );
  #elif freertos
    freertos_spi_read( regaddr, len );
  #endif
}

uint32_t uros_spi_hal_aux(void *aux){
  #ifdef nuttx
	nuttx_spi_hal_aux(aux);
  #elif freertos
    //nothing to add
  #endif
}
