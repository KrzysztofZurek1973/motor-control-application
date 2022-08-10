#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
#include <strings.h>
/**
 *
 * Open UART port
 *
 */
int open_uart_port(char *dev, int baudrate, int vtime, int bytes){
	struct termios newtio;
	int fd = 0;

	//open and configure serial port
	fd = open(dev, O_RDWR | O_NOCTTY);
	if (fd < 0){
		printf("UART port open failed\n");
		//finish_app = true;
		return -1;
	}
	//serial port configuration, always wait for 24 bytes
	bzero(&newtio, sizeof(newtio));
	newtio.c_cflag = baudrate | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	newtio.c_cc[VTIME] = vtime; //time-out in 0.1 sec
	newtio.c_cc[VMIN] = bytes;
	//tcflush(uart_fd, TCIOFLUSH);
	tcsetattr(fd, TCSANOW, &newtio);

	return fd;
}
