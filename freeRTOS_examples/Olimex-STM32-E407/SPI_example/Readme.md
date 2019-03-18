### Instructions
This is an example of how implement  SPI with **freeRTOS**  and the board **Olimex-STM32-E407**. To show this example we're going to use the sensor BMP280 and we will check the chip ID

This example uses the UART3 of the board and the SPI1 (PA4(CS),PA5(SPI_SCK),PA6(SPI_MISO),PB5(SPI_MOSI))
#### How to use:
Connect the sensor to the board with the next connections:
PA4 -> BMP280_CS
PA5 -> BMP280_SCK
PA6 -> BMP280_MISO
PB5 -> BMP280_MOSI
+3.3V -> HIH6130_VDD
GND   ->  HIH6130_GND

Upload the code with TrueStudio and you'll see at the console the chipID every second.
