#define freertos 1
//#define nuttx 0

//Specific libraries and initialization of each RTOS
#ifdef freertos

  #include "stm32f4xx_hal.h"
  #include "cmsis_os.h"
  I2C_HandleTypeDef hi2c1;

#elif nuttx

  #include <nuttx/i2c/i2c_master.h>


#endif

// Common libraries
#include "stdlib.h"
#include "stdint.h"
