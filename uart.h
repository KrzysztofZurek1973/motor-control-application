#ifndef _UART_H_
#define _UART_H_

#define BAUDRATE B115200
#define UART_PORT "/dev/ttyUSB0"

int open_uart_port(char *dev, int baudrate, int vtime, int bytes);

#endif
