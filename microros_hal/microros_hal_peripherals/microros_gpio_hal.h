
#define nuttx 1
//#define freertos 0

#ifdef nuttx
//NuttX libraries

#elif freertos

#include "stm32f4xx_hal.h"

#endif

//Common libraries
#include "stdint.h"
#include "stdlib.h"
