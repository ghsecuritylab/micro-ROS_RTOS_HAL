#include "stdlib.h"
#include <stdio.h>
#include <stdarg.h>
#include "stm32f4xx_hal.h"



/*************Variables and handles***************/
//Check which UART port we're using
#ifdef UART1_set

#elif UART2_set
  UART_HandleTypeDef huart2;
#elif UART3_set
  UART_HandleTypeDef huart3;
#elif UART4_set
  UART_HandleTypeDef huart4;
#endif

UART_HandleTypeDef huart3;

/*****************Printf implementation***********/
//With this funcitons we can use printf instead of
//the HAL stm32 implementation

int __io_putchar(int ch)
{
 uint8_t c[1];
 c[0] = ch & 0x00FF;
 HAL_UART_Transmit(&huart3, &c[0], 1, 10);
 return ch;
}

int _write(int file,char *ptr, int len)
{
 int DataIdx;
 for(DataIdx= 0; DataIdx< len; DataIdx++)
 {
 __io_putchar(*ptr++);
 }
return len;
}


