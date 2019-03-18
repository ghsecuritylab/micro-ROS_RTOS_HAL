#include "microros_gpio_hal.h"

void uros_gpio_config(uint8_t pin, uint16_t mode){
   #ifdef nuttx
    nuttx_gpio_config(pin,mode);
   #elif freertos
    freertos_gpio_config(pin,mode);
  #endif
}

void uros_gpio_write(uint8_t pin, uint8_t value){
  #ifdef nuttx
    nuttx_digital_write(pin,value);
  #elif freertos
    freertos_digital_write(pin,value);
  #endif
}
