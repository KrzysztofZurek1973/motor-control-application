#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>
#include <strings.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

#include "main.h"
//#include "morswin.h"
//#include "pulpit_zdalny.h"
#include "crc16.h"
#include "uart.h"

int mca_diodes_test(int addr){
	uint8_t buff[32];
	uint16_t crc_calc;
	int res = 0;
	int uart_fd;
	
	printf("Diodes test - not implemented yet\n");
	printf("CTRL address: %i\n", addr);
	if (addr > MODBUS_MAX_ADDR){
		printf("Failed: CTRL address error\n");
		exit(EXIT_SUCCESS);
	}
	buff[0] = 0xFF;
	buff[1] = 0x53;
	crc_calc = crc_modbus(buff, 2);
	buff[2] = crc_calc;
	buff[3] = crc_calc >> 8;
	uart_fd = open_uart_port(UART_PORT, BAUDRATE, 20, 4);
	if (uart_fd > 0){
		tcflush(uart_fd, TCIOFLUSH);
		write(uart_fd, buff, 4);
		close(uart_fd);
	}
	else{
		printf("Failed; UART error\n");
		res = -1;
	}
	return res;
}


int mca_start_bootloader(uint8_t addr){
	uint8_t buff[32];
	uint16_t crc_calc;
	int res = 0;
	int uart_fd;
	
	printf("Start bootloader\n");
	printf("CTRL address: %i\n", addr);
	if (addr > MODBUS_MAX_ADDR){
		printf("Failed: CTRL address error\n");
		exit(EXIT_SUCCESS);
	}
	buff[0] = addr;
	buff[1] = 0x7E;
	crc_calc = crc_modbus(buff, 2);
	buff[2] = crc_calc;
	buff[3] = crc_calc >> 8;
	uart_fd = open_uart_port(UART_PORT, BAUDRATE, 2, 0);
	if (uart_fd > 0){
		tcflush(uart_fd, TCIOFLUSH);
		write(uart_fd, buff, 4);
		close(uart_fd);
	}
	else{
		printf("Failed, UART error\n");
		res = -1;
	}
	return res;
}



int mca_write_serial_number(uint8_t addr, uint16_t sn){
	uint8_t buff[32];
	uint16_t crc_recv, crc_calc;
	int res;
	int uart_fd;
	uint8_t msg_buff[16], recv_bytes = 0;
	
	printf("write new serial number\n");
	printf("CTRL address: %i\n", addr);
	if (addr > MODBUS_MAX_ADDR){
		printf("Failed: CTRL address error\n");
		exit(EXIT_SUCCESS);
	}
	buff[0] = addr;
	buff[1] = 0xAA;
	buff[2] = sn;
	buff[3] = sn >> 8;
	crc_calc = crc_modbus(buff, 4);
	buff[4] = crc_calc;
	buff[5] = crc_calc >> 8;
	uart_fd = open_uart_port(UART_PORT, BAUDRATE, 5, 0);
	if (uart_fd > 0){
		tcflush(uart_fd, TCIOFLUSH);
		write(uart_fd, buff, 6);
		tcflush(uart_fd, TCIOFLUSH);
		//wait for answer
		memset(buff, 0, 32);
		while (recv_bytes < 6){
			res = read(uart_fd, buff, 16);
			if (res == 0){
				close(uart_fd);
				printf("Failed: CTRL not responding\n");
				return -1;
			}
			else if (res < 0){
				close(uart_fd);
				printf("Failed: UART read error\n");
				return -1;
			}
			else {
				//copy received bytes into message buffer
				memcpy(msg_buff + recv_bytes, buff, res);
				recv_bytes += res;
				if (recv_bytes == 6){
					crc_calc = crc_modbus(msg_buff, 4);
					crc_recv = (uint16_t)msg_buff[4] + (uint16_t)(msg_buff[5] * 256);
					if (crc_calc == crc_recv){
						uint16_t nsn = (uint16_t)msg_buff[2] + (uint16_t)(msg_buff[3]*256);
						if (nsn == sn){
							printf("New serial number: %i\n", nsn);
						}
						else{
							printf("Failed: serial number NOT set\n");
							res = -1;
						}
					}
					else{
						printf("Write SN CRC error, set current SN = 0\n");
						res = -1;
					}
				}
			}
		}
		close(uart_fd);
	}
	else{
		printf("Failed, UART error\n");
		res = -1;
	}
	return res;
}


int mca_read_serial_number(uint8_t addr){
	uint8_t buff[32];
	uint16_t crc_recv, crc_calc;
	int res;
	int uart_fd;
	uint8_t msg_buff[8], recv_bytes = 0;
	
	printf("Serial number reading\n");
	printf("CTRL address: %i\n", addr);
	if (addr > MODBUS_MAX_ADDR){
		printf("Failed: CTRL address error\n");
		exit(EXIT_SUCCESS);
	}
	buff[0] = addr;
	buff[1] = 0xA1;
	
	crc_calc = crc_modbus(buff, 2);
	buff[2] = crc_calc;
	buff[3] = crc_calc >> 8;
	uart_fd = open_uart_port(UART_PORT, BAUDRATE, 5, 0);
	if (uart_fd > 0){
		tcflush(uart_fd, TCIOFLUSH);
		write(uart_fd, buff, 4);
		//wait for answer
		tcflush(uart_fd, TCIOFLUSH);
		memset(buff, 0, 32);
		while (recv_bytes < 6){
			res = read(uart_fd, buff, 6);
			if (res == 0){
				close(uart_fd);
				printf("Failed: CTRL not responding\n");
				return -1;
			}
			else if (res < 0){
				close(uart_fd);
				printf("Failed: UART read error\n");
				return -1;
			}
			else{
				//copy received bytes into message buffer
				memcpy(msg_buff + recv_bytes, buff, res);
				recv_bytes += res;
				if (recv_bytes == 6){
					crc_calc = crc_modbus(msg_buff, 4);
					crc_recv = (uint16_t)msg_buff[4] + (uint16_t)(msg_buff[5] * 256);
					if (crc_calc == crc_recv){
						uint16_t sn = (uint16_t)msg_buff[2] + (uint16_t)(msg_buff[3] * 256 );
						printf("Current serial number: %i\n", sn);
						res = 0;
					}
					else{
						printf("Read SN CRC error\n");
						res = -1;
					}
				}
			}
		
		}
		close(uart_fd);
	}
	else{
		printf("Failed, UART error\n");
	}
	return res;
}


int mca_write_modbus_addr(uint8_t addr){
	uint8_t buff[32];
	uint16_t crc_recv, crc_calc;
	int res;
	int uart_fd;
	uint8_t msg_buff[8], recv_bytes = 0;
	
	printf("write new addr\n");
	printf("CTRL address: %i\n", addr);
	if (addr > MODBUS_MAX_ADDR){
		printf("Failed: CTRL address error\n");
		exit(EXIT_SUCCESS);
	}
	//send new address to Motor Controller
	//first read current address
	buff[0] = 0xFF; //set new addres only for devices with addres "0"
	buff[1] = 0x41;
	buff[2] = addr;
	crc_calc = crc_modbus(buff, 3);
	buff[3] = crc_calc;
	buff[4] = crc_calc >> 8;
	uart_fd = open_uart_port(UART_PORT, BAUDRATE, 5, 0);
	if (uart_fd > 0){
		tcflush(uart_fd, TCIOFLUSH);
		write(uart_fd, buff, 5);
		//wait for the current address
		tcflush(uart_fd, TCIOFLUSH);
		memset(buff, 0, 32);
		while (recv_bytes < 5){
			res = read(uart_fd, buff, 16);
			if (res == 0){
				close(uart_fd);
				printf("Failed: CTRL not responding\n");
				return -1;
			}
			else if (res < 0){
				close(uart_fd);
				printf("Failed: UART read error\n");
				return -1;
			}
			else{
				memcpy(msg_buff + recv_bytes, buff, res);
				recv_bytes += res;
				if (recv_bytes == 5){
					crc_calc = crc_modbus(msg_buff, 3);
					crc_recv = (uint16_t)msg_buff[3] + (uint16_t)(msg_buff[4]*256 );
					if (crc_calc == crc_recv){
						//new address only for zero address node
						buff[0] = 0;
						buff[1] = 0x41;
						buff[2] = addr;
						crc_calc = crc_modbus(buff, 3);
						buff[3] = crc_calc;
						buff[4] = crc_calc >> 8;
						tcflush(uart_fd, TCIOFLUSH);
						write(uart_fd, buff, 5);
						printf("New address sent: %i\n", addr);
					}
					else{
						printf("Failed: new addres error, %i\n", res);
						res = -1;
					}
				}
				else if (res == 0){
					printf("Failed: CTRL not responding\n");
					res = -1;
				}
			}
		}
		close(uart_fd);
	}
	else{
		printf("Failed: UART error\n");
		res = -1;
	}
	return res;
}


int mca_read_modbus_addr(void){
	uint8_t buff[32];
	uint16_t crc_recv, crc_calc;
	int res;
	int uart_fd;
	uint8_t msg_buff[8], recv_bytes = 0;
	
	printf("Modbus address reading\n");
	buff[0] = 0xFF; //broadcast address
	buff[1] = 0x44;

	crc_calc = crc_modbus(buff, 2);
	buff[2] = crc_calc;
	buff[3] = crc_calc >> 8;
	uart_fd = open_uart_port(UART_PORT, BAUDRATE, 5, 0);
	if (uart_fd > 0){
		tcflush(uart_fd, TCIOFLUSH);
		write(uart_fd, buff, 4);
		//wait for address
		tcflush(uart_fd, TCIOFLUSH);
		
		while (recv_bytes < 5){
			res = read(uart_fd, buff, 16);
			if (res == 0){
				close(uart_fd);
				printf("Failed: CTRL not responding\n");
				return -1;
			}
			else if (res < 0){
				close(uart_fd);
				printf("Failed: UART read error\n");
				return -1;
			}
			else{
				//copy received bytes into message buffer
				memcpy(msg_buff + recv_bytes, buff, res);
				recv_bytes += res;
				if (recv_bytes == 5){
					crc_calc = crc_modbus(msg_buff, 3);
					crc_recv = (uint16_t)msg_buff[3] + (uint16_t)(msg_buff[4] * 256);
					if (crc_calc == crc_recv){
						printf("current address: %i\n", msg_buff[0]);
						res = 0;
					}
					else{
						printf("Failed: CRC error\n");
						res = -1;
					}
				}
			}
		}
		close(uart_fd);
	}
	else{
		printf("Failed: UART open error\n");
		res = -1;
	}
	return res;
}

/**
 *
 *
 *
 */
int mca_set_boot_bit(uint8_t addr, int boot_bit){
	uint8_t buff[32];
	uint16_t crc_recv, crc_calc;
	int res;
	int uart_fd;
	uint8_t msg_buff[16], recv_bytes = 0;

	printf("Setting nBoot1 bit in \"Option Bytes\"\n");
	printf("CTRL address: %i\n", addr);
	if (addr > MODBUS_MAX_ADDR){
		printf("Failed: CTRL address error\n");
		exit(EXIT_SUCCESS);
	}
	buff[0] = addr;
	buff[1] = 0x22;
	if (boot_bit == 0){
		buff[2] = 0;
	}
	else{
		buff[2] = 1;
	}
	
	crc_calc = crc_modbus(buff, 3);
	buff[3] = crc_calc;
	buff[4] = crc_calc >> 8;
	uart_fd = open_uart_port(UART_PORT, BAUDRATE, 2, 0);
	if (uart_fd > 0){
		tcflush(uart_fd, TCIOFLUSH);
		write(uart_fd, buff, 5);
		//wait for answer
		memset(buff, 0, 32);
		while (recv_bytes < 5){
			res = read(uart_fd, buff, 16);
			if (res == 0){
				close(uart_fd);
				//printf("Failed: CTRL not responding\n");
				printf("NOT implemented, if you need it very much ");
				printf("write e-mail to: krzzurek@pg.edu.pl\n");
				return -1;
			}
			else if (res < 0){
				close(uart_fd);
				printf("Failed: UART read error\n");
				return -1;
			}
			else {
				//copy received bytes into message buffer
				memcpy(msg_buff + recv_bytes, buff, res);
				recv_bytes += res;
				if (recv_bytes == 5){
					crc_calc = crc_modbus(msg_buff, 3);
					crc_recv = (uint16_t)msg_buff[3] + (uint16_t)(msg_buff[4]*256);
					if (crc_calc == crc_recv){
						printf("Success, bit set\n");
					}
				}
			}
			close(uart_fd);
		}
	}
	else{
		printf("Failed, UART error\n");
	}
	return 0;
}


/**
 *
 * Activate (2) or desactivate (1) setting speed by morswin protocol
 * This functionality is for tests only to anable control by Motor Pilot app.
 *
 */
int mca_mor_prot_activate(uint8_t addr, int input){
	uint8_t buff[32];
	uint16_t crc_recv, crc_calc;
	int res;
	int uart_fd;
	uint8_t msg_buff[16], recv_bytes = 0;
	
	input--;
	if (input == 0){
		printf("Deactivate morswin protocol\n");
	}
	else{
		printf("Activate morswin protocol\n");
	}
	
	printf("CTRL address: %i\n", addr);
	if (addr > MODBUS_MAX_ADDR){
		printf("Failed: CTRL address error\n");
		exit(EXIT_SUCCESS);
	}
	buff[0] = addr;
	buff[1] = 0xCC;
	buff[2] = input;
	crc_calc = crc_modbus(buff, 3);
	buff[3] = crc_calc;
	buff[4] = crc_calc >> 8;
	uart_fd = open_uart_port(UART_PORT, BAUDRATE, 5, 0);
	if (uart_fd > 0){
		tcflush(uart_fd, TCIOFLUSH);
		write(uart_fd, buff, 5);
		//wait for answer
		memset(buff, 0, 32);
		while (recv_bytes < 6){
			res = read(uart_fd, buff, 16);
			if (res == 0){
				close(uart_fd);
				printf("Failed: CTRL not responding\n");
				return -1;
			}
			else if (res < 0){
				close(uart_fd);
				printf("Failed: UART read error\n");
				return -1;
			}
			else {
				//copy received bytes into message buffer
				memcpy(msg_buff + recv_bytes, buff, res);
				recv_bytes += res;
				if (recv_bytes == 5){
					if (msg_buff[1] == 0xCC){
						crc_calc = crc_modbus(msg_buff, 3);
						crc_recv = (uint16_t)msg_buff[3] + (uint16_t)(msg_buff[4] * 256);
						if (crc_calc == crc_recv){
							printf("Success\n");
						}
						else{
							printf("Failed, CRC error\n");
						}
					}
					else{
						printf("CTRL not responded fot this command, 0x%02X\n", msg_buff[1]);
					}
				}
				else if (res == 0){
					printf("CTRL not responding\n");
				}
			}
		}
		close(uart_fd);
	}
	else{
		printf("Failed, UART error\n");
	}
	
	return res;
}
