#include <nuttx/config.h>

#include <stdbool.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/wdog.h>
#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "chip.h"
#include "stm32.h"
#include "olimex-stm32-e407.h"

#include "microros_gpio_nuttx.h"




//This define is to set the number of pin that we're going to use in this board
#define pin_number 14


static const uint32_t gpiout_num[pin_number] =
{
  GPIOUT_0,GPIOUT_1,GPIOUT_3,GPIOUT_4,GPIOUT_5,GPIOUT_6,GPIOUT_7,GPIOUT_8,GPIOUT_9,GPIOUT_10,GPIOUT_11,
  GPIOUT_12,GPIOUT_13,
};

static const uint32_t gpin_num[pin_number] =
{
  GPIN_0,GPIN_1,GPIN_3,GPIN_4,GPIN_5,GPIN_6,GPIN_7,GPIN_8,GPIN_9,GPIN_10,GPIN_11,
  GPIN_12,GPIN_13,
};

void nuttx_gpio_config(uint8_t pin, uint16_t mode){
  if(mode == 0xf1){
    printf("output\n" );
    stm32_configgpio(gpiout_num[pin]);
  }
  else if(mode == 0xf2) stm32_configgpio(gpin_num[pin]);
  //TODO: Interruption configuration
}

void nuttx_digital_write(uint8_t pin, uint8_t value){
  stm32_gpiowrite(gpiout_num[pin], value);
  printf("pin: %i, value: %i \r\n",pin,value);
}

//TODO nuttx_digital_read
//TODO nuttx_interrupt
