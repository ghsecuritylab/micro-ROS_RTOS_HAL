### Primitive Table

#### I2C

|      | NuttX     | freeRTOS(STM32CubeMX HAL)    | Comment|
| :------------- | :------------- | :------------- | :------------- |
| I2C Write        | I2C_TRANSFER(FAR struct i2c_master_s *dev, struct i2c_msg_s *msg, uint8_t opt)       |HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)|-|
| I2C Write(Higher Abstraction)      | i2c_write(FAR struct i2c_master_s *dev,FAR const struct i2c_config_s *config,FAR const uint8_t *buffer, int buflen) |-       |- |
| I2C Read    |I2C_TRANSFER(FAR struct i2c_master_s *dev, struct i2c_msg_s *msg, uint8_t opt)   |HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)|-|
| I2C Read(Higher Abstraction)  |i2c_read(FAR struct i2c_master_s *dev,FAR const struct i2c_config_s *config,FAR uint8_t *buffer, int buflen)|-       | -|

The function I2C_TRANSFER is the same to write and read, because you set what do you want to do in the structure "msg". You can set if you want to read or write and how many data you want to send or receive.
This is an example of write 1 byte:
``` c++
struct i2c_msg_s msg;

msg.addr = priv->addr;
msg.flags = 0;
msg.buffer = &buffer;
msg.length = 1;
```
The difference between I2C_TRANSFER and i2c_write/i2c_read is that the last one is an abstraction of the I2C_Transfer.

#### SPI

|      | NuttX     | freeRTOS(STM32CubeMX HAL)   | Comment|
| :------------- | :------------- | :------------- | :------------- |
| SPI Send 1 Byte     | SPI_SEND(dev->spi, regval)    | HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)     |-  |
| SPI Send block of data      | SPI_SNDBLOCK(dev->spi, regval, length)       |Is the previous function only changing the value of size.   |-      |
| SPI Chip Select       | SPI_SELECT(priv->spi, SPIDEV_ACCELEROMETER(0), false)     |HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)|In freeRTOS is set to high or low a gpio pin. NuttX do the same but it have a complex structure      |
| SPI receive 1 Byte     | SPI_SEND(dev->spi, 0)    |HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)  |-   |
| SPI receive block     | -   |Is the previous function only changing the value of size.      |In NuttX to receive a block you need to call several times SPI_SEND|
| SPI Lock/Unlock the bus    | SPI_LOCK(spi, false);    |-      |I don't find in freeRTOS a similar tool. But it's basically a mutex.|


#### UART

The UART under NuttX use possix.

|      | NuttX     | freeRTOS(STM32CubeMX HAL)    | Comment|
| :------------- | :------------- | :------------- | :------------- |
| Open UART       | open("/dev/ttyS1",O_RDWR)       | HAL_UART_Init(UART_HandleTypeDef *huart)       |-|
| Read UART       | read(fd,&buffer,sizeof(buffer))      | HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)     |-|
| Write UART       | write(fd,buffer,sizeof(char)*i)  | HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)|-|

#### Ethernet

This are the basic networking functions.

|      | NuttX     | freeRTOS(LWIP)    | Comment|
| :------------- | :------------- | :------------- | :------------- |
| Socket       | int socket(int domain, int type, int protocol)       |netconn_new(NETCONN_TCP)       |I think it's not exctly the same, because the freeRTOS implement more things inside, but it could be equivalent.      |
| Set socket       | int setsockopt(int sockfd, int level, int optname,const void *optval, socklen_t optlen)     |-      |Probably in freeRTOS both are implemented |
| Bind       | int bind(int sockfd, const struct sockaddr *addr,socklen_t addrlen)      |  netconn_bind(struct netconn *conn, const ip_addr_t *addr, u16_t port)       |-|
| Listen to a port       |  listen(int sockfd, int backlog)     |netconn_listen(struct netconn *conn)      |-|
| Accept       | int accept(int sockfd, struct sockaddr *addr, socklen_t *addrlen)|netconn_accept ( struct netconn * aNetConn, struct netconn ** aNewConn);|-|
| Received      | ssize_t recvfrom(int sockfd, void *buf, size_t len, int flags,struct sockaddr *src_addr, socklen_t *addrlen);|netconn_recv(struct netconn *conn, struct netbuf **new_buf)       |-|
| Send     |ssize_t sendto(int sockfd, const void *buf, size_t len, int flags,const struct sockaddr *dest_addr, socklen_t addrlen)|netconn_write ( struct netconn * aNetConn, const void * aData,size_t aSize, u8_t aApiFlags )|-|

NuttX give you more options to send and receive data:
```c++
- ssize_t send(int sockfd, const void *buf, size_t len, int flags);
- write(int sockfd, const void *buf, size_t len);
- ssize_t recv(int sockfd, void *buf, size_t len, int flags);
- read(int sockfd, const void *buf, size_t len);
```

#### Power Modes

|      | NuttX     | freeRTOS    | Comment|
| :------------- | :------------- | :------------- | :------------- |
| STM32 Stop      | int stm32_pmstop(bool lpds)   |HAL_PWR_EnterSTOPMode(uint32_t Regulator, uint8_t STOPEntry)       |Stop Entry variable is to set if we want mode WFI or WFE and in NuttX set by ifconfig      |
| STM32 Sleep       | void stm32_pmsleep(bool sleeponexit)       |void HAL_PWR_EnterSLEEPMode(uint32_t Regulator, uint8_t SLEEPEntry)       |The same as STM32Stop       |
| STM32 StandBy       | int stm32_pmstandby(void)      |void HAL_PWR_EnterSTANDBYMode(void)       |- |

#### Scheduling

The scheduling in both RTOS is implemented in the kernel, so you can't modify it easily, but they give some options to configure it.

**freeRTOS:**

The options that give you this RTOS are very little. You're only be able to modify the priority of the task when you create it.

The options available are:
- osPriorityIdle
- osPriorityLow
- osPriorityBelowNormal
- osPriorityNormal
- osPriorityAboveNormal
- osPriorityHigh
- osPriorityRealtime

In this link they explain with an example how it works.
https://www.freertos.org/implementation/a00008.html

The explanation in the webpage it's not extended, but basically is this:

![image](https://user-images.githubusercontent.com/13625726/45149605-42fa2500-b1ca-11e8-8c94-ec1f9292bd98.png)

We have 2 threads created by the user (vControlTask and vKeyHandlerTask) and the last one (Idle Task) is created by RTOS to control the user thread (Note: I use indistinctly thread or task, because in this RTOs are basically the same).

The Idle Task is running if other task is not running and decide which task should have access depending the events and priority of the thread.



And this is an example of creation of the thread:
```c++
osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
```


**NuttX**

In this case, we have the opposite case, too much info without a summary. These links have very usefull info, but specially the second one with is a presentation of the NuttX realtime programming:

http://nuttx.org/doku.php?id=wiki:nxinternal:nxtasking#the_scheduler
http://nuttx.org/Documentation/Realtime-151012.pdf

This RTOS has the next features:

- Task = A thread within an environment (like a Linux process)
- Thread = “Normal” sequence of instruction execution
- Each thread has its own stack
- Each thread has an execution priority managed by the OS
- Each thread is a member of a “task group”
- Share resources (like a Linux process)
- Can wait for events or resource availability
- Threads communicate via Interprocess Communications (IPC):
- POSIX Message Queues, Signals, Counting semaphores, etc.
- Standard / Linux compatible
- NuttX supports use of standard IPCs from interrupt handlers

So, to have like a practical example:

If you create an App you have a Task, this task is basically a thread with their own resources. Then inside the app you can create threads with the standard C/C++ pthreads library like this sub-example:
```c++
#include <nuttx/config.h>
#include <stdio.h>
#include <pthread.h>

 void *hebra1(void *vargp)
 {
   while(1){
     usleep(2000000);
     printf("Thread\n");
   }
     return NULL;
 }

 void *hebra2(void *vargp)
 {
     //sleep(1);
     while(1){
       usleep(1000000);
       printf("Thread 2\n");
     }

     return NULL;
 }

#if defined(BUILD_MODULE)
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
  pthread_t thread_id,thread_id2;

  pthread_create(&thread_id, NULL, hebra1, NULL);
  pthread_create(&thread_id2, NULL, hebra2, NULL);
  pthread_join(thread_id, NULL);
  pthread_join(thread_id2, NULL);

  return 0;
}
```

** Nuttx implements the next scheduler options: **

- Scheduler FIFO:
  - For managed latency.
  - Supports Rates Monotonic Scheduling (RMS).
- Scheduler RR(Round Robin):
  - Not real time.
  - Time-Slicing.
  - Balanced throughput
- Scheduler Sporadic:
  - Dynamic prioritization to achieve processing budget.
  - For background task with guaranteed bandwidth.
