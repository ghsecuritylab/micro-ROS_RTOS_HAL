### Instructions
This is an example of how implement  I2C with **freeRTOS**  and the board **Olimex-STM32-E407**. To show this example we're going to use the sensor HIH6130 and will show every second an update of the temperature

This example uses the UART3 of the board and the I2C1 (PB6(SCL),PB7(SDA))
#### How to use:
Connect the sensor to the board with the next connections:
PB6 -> HIH6130_SCL
PB7 -> HIH6130_SDA
+3.3V -> HIH6130_VDD
GND   ->  HIH6130_GND

Upload the code with TrueStudio and you'll see at the console the temperature and humidity every second.
