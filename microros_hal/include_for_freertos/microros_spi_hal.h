#define freertos 1
//#define nuttx

//Specific libraries and initialization of each RTOS
#ifdef freertos

  #include "stm32f4xx_hal.h"
  #include "cmsis_os.h"


#elif nuttx

  #include <nuttx/spi/spi.h>


#endif

// Common libraries
#include "stdlib.h"
#include "stdint.h"
