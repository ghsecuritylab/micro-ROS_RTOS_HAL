## How to install microROS I2C HAL library

### NuttX

- Copy folder **microros_hal_peripherals** to ```nuttx\libc```
- In the file ```nuttx\libc\Makefile``` introduce the next line: ```include microros_hal_peripherals\Make.defs```
- Copy the folders **configs** and **include** from **gpio nuttx** to ```nuttx\```
- It will ask if you want to overwrite, set yes.

**Note**
This is still WIP.
- GO to each microros_hal_xx.h and set the next config:
```c++
//#define freertos 1
#define nuttx 1
```




### freeRTOS

- Copy the folder **microros_hal_peripherals** to ```freertos\src```
- Copy the content of folder **include_for_freertos** to ```freertos\Inc```


**Note**
This is still WIP.
- GO to each microros_hal_xx.h and set the next config:
```c++
//#define freertos 1
#define nuttx 1
```
