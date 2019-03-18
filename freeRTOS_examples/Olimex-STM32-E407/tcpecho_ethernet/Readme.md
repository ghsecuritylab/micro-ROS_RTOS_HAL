### Instructions
This is an example of how implement a TCP Echo server with **freeRTOS** , **LWIP** and the board **Olimex-STM32-E407**.

This example set a dinamic IP with DHCP. Then a thread is checking if there is any new message and in the case of a new message, it returns to the client.

#### How to use:
Upload the code to the board with TrueStudio, then open a console(Console is in the UART3) and wait a few seconds to see which IP was assigned to the board. Then open a TCP Client in the console with the returned IP and the port 7.
You could use for example ``netcast``.
If everything was fine, you should see in the client the same message as you send.
