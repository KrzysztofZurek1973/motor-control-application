/**
 * Software for testing communication with motor controler for Morswin
 * underwater vehicle
 *
 * Gdansk University of Technology
 * Deep Sea Techniques Laboratory
 * Krzysztof Zurek, 2022
 * krzzurek@gmail.com
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>
#include <strings.h>
#include <stdint.h>

#include "morswin.h"
#include "pulpit_zdalny.h"
#include "crc16.h"

#define uint8_t unsigned char
#define int8_t char
#define uint32_t unsigned int
#define int32_t int
#define uint16_t unsigned short
#define int16_t short

#define TIMER_PERIOD_MS 1
#define MAIN_TIMER_PERIOD_S 5
#define V_MAX 3000
#define V_MIN 500
#define DISPLAY_PERIOD_S 2
#define RC_PERIOD_MS 10
#define MODBUS_MAX_ADDR 8

char help_str[] =	"Motor Controller Application\n"\
					"version 2022-05-10\n\n	"\
					"options:\n"\
					"\t-r - read\n"\
					"\t-w - write\n"\
					"\t-a - address\n"\
					"\t-s - serial number\n"\
					"\t-d - diodes test\n"\
					"\t-v - speed\n"\
					"\t-b - start bootloader\n"\
					"\t-o - activate nBOOT1 bit (1), deactivate (0)\n"\
					"\t-h - this help\n"\
					"examples:\n"\
					"\t-a 1          communication only with CTRL 	address 1\n"\
					"\t-r -a 0       read Modbus address\n"\
					"\t-r -a 1 -s 0  read serial number\n"\
					"\t-w -a 0       reset Modbus address (set as 0)\n"\
					"\t-w -a 2       set Modbus address as 2, possible only if current address is 0\n"\
					"\t-w -a 1 -s 1200   write serial number as 1200\n"\
					"\t-w -a 1 -v 1000   set speed as 1000 (for tests)\n"\
					"\t-a 1 -b       start bootloader on CTRL with address 1\n"\
					"\t-a 1 -o 1     set nBOOT1 bit on CTRL with address 1";

//global variables
int uart_fd = -1;
uint8_t modbus_addr = MODBUS_MAX_ADDR + 1;
TModoWKBP ModoWKBP;
TModoPG ModoPG;
int32_t p_poziom, p_pion = 0;
bool finish_app = false;
pthread_t p_keyboard_thread, p_mc_thread, rc_thread;
uint32_t main_loop_cnt = 0;
uint8_t ipnna = 0;
uint8_t ped_mar_odp[4] = {0, 0, 0, 0};
uint8_t ped_man_odp[4] = {0, 0, 0, 0};
int16_t pednik_mar[4];
int16_t pednik_man[4];
int16_t current_speed = 0;
uint32_t recv_bytes = 0;
uint32_t motor_cnt = 0;

//for test only
pthread_t p_display_param_thread;
TodSilnik poziom_glob[4]={0};
void *display_param_fun(void *arg);
uint32_t disp_cnt = 0;
//end of test

//functions
void *keyboard_thread_fun(void *arg);
void *pedniki_func(void *arg);
void *remote_console_fun(void *arg);

int NadajAdresSilnika(int32_t uchwyt_portu,
					uint8_t adres_sterownika,
					uint8_t timeout);
int32_t Silnik(int32_t uchwyt_portu,
			uint8_t adres_sterownika,
			int16_t predkosc,
			uint8_t timeout,
			TodSilnik *buf_wyn);
int CzekajNaZnaki(int uchwyt_portu,
					unsigned char adres_sterownika,
					unsigned short il_zn_do_odebrania,
					unsigned short timeout,
					unsigned char buf_in[100]);

//functions
void CRC16(uint8_t ramka[], uint8_t m, uint8_t *crc_h, uint8_t *crc_l);
int open_uart_port(char *dev, int baudrate, int vtime, int bytes);

/**
 *
 * Main loop
 *
 */
int main(int argc, char *argv[]) {
    struct timespec t_req, t_rem;
	int option;
	char *cvalue = NULL;
	uint8_t buff[32]; //uart buffer
	int res = 0;
	uint16_t crc_recv, crc_calc;
	bool read_mode = false, write_mode = false;
	bool set_new_speed = false, set_serial = false, diode_test = false;
	bool set_new_addr = false;
	int new_speed = 0, new_serial_number = 0;
	bool setting_mode = false;
	bool bootloader_run = false;
	bool set_boot_bit = false;
	int nboot1_val;

	modbus_addr = 100; //default address

	if (argc > 1){
		while ((option = getopt(argc, argv, "rwv:a:s:hdbo:")) != -1){
			switch (option){
			case 'b':
				bootloader_run = true;
				setting_mode = true;
				break;

			case 'o':
				set_boot_bit = true;
				setting_mode = true;
				cvalue = optarg;
				if (cvalue != NULL){
					nboot1_val = atoi(cvalue);
				}
				break;

			case 'a':
				//controller modbus address
				cvalue = optarg;
				if (cvalue != NULL){
					set_new_addr = true;
					modbus_addr = atoi(cvalue);
				}
				//printf("address: %i\n", modbus_addr);
				break;

			case 'h':
				//display help text
				printf("%s\n", help_str);
				return 0;

			case 'v':
				//set new speed
				cvalue = optarg;
				new_speed = atoi(cvalue);
				set_new_speed = true;
				//printf("new speed: %i\n", new_speed);
				setting_mode = true;
				break;

			case 'r':
				//reset modbus address
				read_mode = true;
				setting_mode = true;
				//printf("read mode\n");
				break;

			case 'w':
				write_mode = true;
				setting_mode = true;
				//printf("write mode\n");
				break;

			case 'd':
				setting_mode = true;
				diode_test = true;
				break;

			case 's':
				//printf("serial number");
				set_serial = true;
				cvalue = optarg;
				if (cvalue != NULL){
					new_serial_number = atoi(cvalue);
					//printf(" %i\n", new_serial_number);
				}
				else{
					//printf("\n");
				}
				break;

			case '?':
			default:
				printf("Unknown option\n");
				exit(EXIT_FAILURE);
			}
		}
	}

	//reade/write mode
	if (setting_mode == true){
		if (read_mode == true){
			//printf("read mode\n");
			if ((set_new_addr == true) && (set_serial == false)){
				printf("Modbus address reading\n");
				buff[0] = 0xFF; //broadcast address
				buff[1] = 0x44;
				//CRC16(buff, 2, &buff[3], &buff[2]);
				crc_calc = crc_modbus(buff, 2);
				buff[2] = crc_calc;
				buff[3] = crc_calc >> 8;
				uart_fd = open_uart_port("/dev/ttyUSB0", B57600, 20, 5);
				tcflush(uart_fd, TCIOFLUSH);
				write(uart_fd, buff, 4);
				//wait for address
				//nanosleep(&t_req, &t_rem);
				res = read(uart_fd, buff, 5);
				close(uart_fd);

				if (res == 5){
					printf("current addres: %i\n", buff[0]);
				}
				else{
					printf("Address reading ERROR: response not correct, %i, 0x%02x\n", res, buff[0]);
				}
			}
			else if (set_serial == true){
				printf("Serial number reading\n");
				buff[0] = modbus_addr;
				buff[1] = 0xA1;
				crc_calc = crc_modbus(buff, 2);
				buff[2] = crc_calc;
				buff[3] = crc_calc >> 8;
				uart_fd = open_uart_port("/dev/ttyUSB0", B57600, 20, 6);
				tcflush(uart_fd, TCIOFLUSH);
				write(uart_fd, buff, 4);
				//wait for answer
				res = read(uart_fd, buff, 8);
				if (res == 6){
					crc_calc = crc_modbus(buff, 4);
					crc_recv = (uint16_t)buff[4] + (uint16_t)(buff[5]*256);
					if (crc_calc == crc_recv){
						uint16_t sn = (uint16_t)buff[2] + (uint16_t)(buff[3]*256 );
						printf("Current serial number: %i\n", sn);
					}
					else{
						printf("Read SN CRC error\n");
					}
				}
				else{
					printf("Read SN error: answer length NOT ok\n");
				}
				close(uart_fd);
			}
			else{
				printf("read option not correct\n");
			}
		}
		else if (write_mode == true){
			//printf("write mode\n");
			if ((set_new_addr == true) && (set_new_speed == false) &&
				(set_serial == false)){
				printf("write new addr\n");
				//send new address to Motor Controller
				//first read current address
				buff[0] = 0xFF; //set new addres only for devices with addres "0"
				buff[1] = 0x41;
				buff[2] = modbus_addr;
				crc_calc = crc_modbus(buff, 3);
				buff[3] = crc_calc;
				buff[4] = crc_calc >> 8;
				uart_fd = open_uart_port("/dev/ttyUSB0", B57600, 20, 5);
				tcflush(uart_fd, TCIOFLUSH);
				write(uart_fd, buff, 5);
				//wait for the current address
				tcflush(uart_fd, TCIOFLUSH);
				res = read(uart_fd, buff, 24);
				if (res == 5){
					crc_calc = crc_modbus(buff, 3);
					crc_recv = (uint16_t)buff[3] + (uint16_t)(buff[4]*256 );
					if (crc_calc == crc_recv){
						//new address only for zero address node
						buff[0] = 0;
						buff[1] = 0x41;
						buff[2] = modbus_addr;
						crc_calc = crc_modbus(buff, 3);
						buff[3] = crc_calc;
						buff[4] = crc_calc >> 8;
						tcflush(uart_fd, TCIOFLUSH);
						write(uart_fd, buff, 5);
						printf("New address sent: %i\n", modbus_addr);
					}
					else{
						printf("New addres error, %i\n", res);
					}
				}
				else{
					printf("Set address failed: wrong answer, %i\n", res);
				}
				close(uart_fd);
			}
			else if (set_serial == true){
				printf("write new serial number\n");
				buff[0] = modbus_addr;
				buff[1] = 0xAA;
				buff[2] = new_serial_number;
				buff[3] = new_serial_number >> 8;
				crc_calc = crc_modbus(buff, 4);
				buff[4] = crc_calc;
				buff[5] = crc_calc >> 8;
				uart_fd = open_uart_port("/dev/ttyUSB0", B57600, 20, 6);
				tcflush(uart_fd, TCIOFLUSH);
				write(uart_fd, buff, 6);
				//wait for answer
				res = read(uart_fd, buff, 6);
				if (res == 6){
					crc_calc = crc_modbus(buff, 4);
					crc_recv = (uint16_t)buff[4] + (uint16_t)(buff[5]*256);
					if (crc_calc == crc_recv){
						uint16_t nsn = (uint16_t)buff[2] + (uint16_t)(buff[3]*256);
						if (nsn == new_serial_number){
							printf("New serial number: %i\n", nsn);
						}
						else{
							printf("Serial number NOT set\n");
						}
					}
					else{
						printf("Read SN CRC error\n");
					}
				}
				else{
					printf("Read SN error: answer length NOT ok, %i\n", res);
				}
				close(uart_fd);
			}
			else if (set_new_speed == true){
				printf("Set new speed\n");
				if (modbus_addr > 8){
					buff[0] = 0xFF;
				}
				else{
					buff[0] = modbus_addr;
				}
				buff[1] = 0x01;
				buff[2] = 0;
				buff[3] = new_speed;
				buff[4] = new_speed >> 8;
				crc_calc = crc_modbus(buff, 5);
				buff[5] = crc_calc;
				buff[6] = crc_calc >> 8;
				uart_fd = open_uart_port("/dev/ttyUSB0", B57600, 20, 19);
				tcflush(uart_fd, TCIOFLUSH);
				t_req.tv_sec = 0;
				t_req.tv_nsec = 100 * 1000000;
				while (1){
					tcflush(uart_fd, TCIOFLUSH);
					write(uart_fd, buff, 7);
					nanosleep(&t_req, &t_rem);
				}
				close(uart_fd);
			}
			else{
				printf("unknown write option\n");
			}
		}
		else{
			if (diode_test == true){
				buff[0] = 0xFF;
				buff[1] = 0x53;
				crc_calc = crc_modbus(buff, 2);
				buff[2] = crc_calc;
				buff[3] = crc_calc >> 8;
				uart_fd = open_uart_port("/dev/ttyUSB0", B57600, 20, 4);
				tcflush(uart_fd, TCIOFLUSH);
				write(uart_fd, buff, 4);
				close(uart_fd);
				printf("Diodes test\n");
			}
			else if (bootloader_run == true){
				//start bootloader
				buff[0] = modbus_addr;
				buff[1] = 0x7E;
				crc_calc = crc_modbus(buff, 2);
				buff[2] = crc_calc;
				buff[3] = crc_calc >> 8;
				uart_fd = open_uart_port("/dev/ttyUSB0", B57600, 20, 4);
				tcflush(uart_fd, TCIOFLUSH);
				write(uart_fd, buff, 4);
				close(uart_fd);
				printf("Start bootloader\n");
			}
			else if (set_boot_bit == true){
				//set nBoot1 in option bytes
				buff[0] = modbus_addr;
				buff[1] = 0x22;
				if (nboot1_val == 0){
					buff[2] = 0;
				}
				else{
					buff[2] = 1;
				}
				crc_calc = crc_modbus(buff, 3);
				buff[3] = crc_calc;
				buff[4] = crc_calc >> 8;
				uart_fd = open_uart_port("/dev/ttyUSB0", B57600, 20, 5);
				tcflush(uart_fd, TCIOFLUSH);
				write(uart_fd, buff, 5);
				//close(uart_fd);
				printf("Setting nBoot1 bit in \"Option Bytes\"\n");
				//wait for answer
				res = read(uart_fd, buff, 5);
				if (res == 5){
					crc_calc = crc_modbus(buff, 2);
					crc_recv = (uint16_t)buff[3] + (uint16_t)(buff[4]*256);
					if (crc_calc == crc_recv){
						printf("Success, bit set\n");
					}
				}
				else{
					printf("Answer error, %i\n", res);
				}
				close(uart_fd);
			}
		}
		exit(EXIT_SUCCESS);
	}

	init_pulpit_zdalny("/dev/ttyS0", B9600);

	//create motor controller writer
	int pmc = pthread_create(&p_mc_thread,
							NULL,
							pedniki_func,
							NULL);
	if (pmc != 0){
		printf("Motor controller writer thread NOT created\n");
		return -1;
	}
	//create keyboard reader
	int pkey = pthread_create(&p_keyboard_thread,
							NULL,
							keyboard_thread_fun,
							NULL);
	if (pkey != 0){
		printf("Keyboard reader thread NOT created\n");
		return -1;
	}

	//create display writer thread
	int pdisp = pthread_create(&p_display_param_thread,
							NULL,
							display_param_fun,
							NULL);
	if (pdisp != 0){
		printf("Parameter display thread NOT created\n");
		return -1;
	}

	//create remote console reader
	int pdrc = pthread_create(&rc_thread,
							NULL,
							remote_console_fun,
							NULL);
	if (pdrc != 0){
		printf("RC thread NOT created\n");
		return -1;
	}

	t_req.tv_sec = MAIN_TIMER_PERIOD_S;
	t_req.tv_nsec = 0;

	while (finish_app == false){
		nanosleep(&t_req, &t_rem);
		//printf("%i\n", main_loop_cnt++);
	}
	printf("EXIT\n");
}


/**
 *
 * Sending current speed to motor controllers
 *
 */
void *pedniki_func(void *arg){
	struct timespec t_req, t_rem;
	//sigset_t blocked;
	//siginfo_t si;
	//int res;
	short pednik_poz[4]={0,0,0,0};
	short pednik_pio[4]={0,0,0,0};
	TodSilnik poziom_lok[4]={0};
	TodSilnik pion_lok[4]={0};
	//unsigned char i;
	int lor[4]= {0,0,0,0};			//liczba odebranych ramek
	//int uart_fd = -1;
	uart_fd = open_uart_port("/dev/ttyUSB0", B57600, 0, 0);
	if (uart_fd < 0){
		return NULL;
	}
	//p_pion = uart_fd;
	p_poziom = uart_fd;

	printf("Pedniki thread is running\n");

	arg = arg;
	while(1) {
		t_req.tv_sec = 0;
		t_req.tv_nsec = TIMER_PERIOD_MS * 1000000;
		nanosleep(&t_req, &t_rem);
		motor_cnt++;

		//pthread_mutex_lock(&muteks);
		memcpy((void*)pednik_poz, (const void*)pednik_mar, 4 * sizeof(short));
		memcpy((void*)pednik_pio, (const void*)pednik_man, 4 * sizeof(short));
		memcpy((void*)&ModoPG.marszowy, (const void*)poziom_lok, 4 * sizeof(TodSilnik));
		memcpy((void*)&ModoPG.manewrowy, (const void*)pion_lok, 4 * sizeof(TodSilnik));
		//pthread_mutex_unlock(&muteks);

		if ((ModoWKBP.Niespr[1] & 0x01) && (ipnna < 3)){
			NadajAdresSilnika(p_poziom,1,7);
			ipnna++;
		}
		if ((ModoWKBP.Niespr[1] & 0x02) && ipnna < 3){
			NadajAdresSilnika(p_poziom,2,7);
			ipnna++;
		}
		if ((ModoWKBP.Niespr[1] & 0x04) && ipnna < 3){
			NadajAdresSilnika(p_poziom,3,7);
			ipnna++;
		}
		if ((ModoWKBP.Niespr[1] & 0x08) && ipnna < 3){
			NadajAdresSilnika(p_poziom,4,7);
			ipnna++;
		}
		//if ((motor_cnt % 100) == 0){
		//	printf("%i, V poz: %i\n", motor_cnt, pednik_poz[0]);
		//}
		if (modbus_addr <= MODBUS_MAX_ADDR){
			if (Silnik(p_poziom,
						modbus_addr,
						pednik_poz[modbus_addr - 1],
						7,
						(TodSilnik*)&poziom_lok[modbus_addr - 1]) == 19){
				lor[modbus_addr - 1]++;
			}
			t_req.tv_sec = 0;
			t_req.tv_nsec = 100 * 1000000;
			nanosleep(&t_req, &t_rem);
		}
		else if (modbus_addr == 0xFF){
			/*if (Silnik(p_poziom,
						0xFF,
						pednik_poz[0],
						7,
						(TodSilnik*)&poziom_lok[0]) == 19){
				lor[0]++;
			}*/
			t_req.tv_sec = 0;
			t_req.tv_nsec = 1000 * 1000000;
			nanosleep(&t_req, &t_rem);
		}
		else if (modbus_addr > MODBUS_MAX_ADDR){
			if (Silnik(p_poziom,1,pednik_poz[0],7,(TodSilnik*)&poziom_lok[0])==19) lor[0]++;
			if (Silnik(p_poziom,2,pednik_poz[1],7,(TodSilnik*)&poziom_lok[1])==19) lor[1]++;
			if (Silnik(p_poziom,3,-pednik_poz[2],7,(TodSilnik*)&poziom_lok[2])==19) lor[2]++;
			if (Silnik(p_poziom,4,-pednik_poz[3],7,(TodSilnik*)&poziom_lok[3])==19) lor[3]++;
		}

		memcpy(poziom_glob, poziom_lok, sizeof(TodSilnik) * 4);

		/*
		Silnik(p_pion,1,pednik_pio[0],7,(TodSilnik*)&pion_lok[0]);
		Silnik(p_pion,2,pednik_pio[1],7,(TodSilnik*)&pion_lok[1]);
		Silnik(p_pion,3,pednik_pio[2],7,(TodSilnik*)&pion_lok[2]);
		Silnik(p_pion,4,pednik_pio[3],7,(TodSilnik*)&pion_lok[3]);
		*/
	}
}


/**
 *
 * Send new speed to motor controller
 *
 */
int32_t Silnik(int32_t uchwyt_portu,
			uint8_t adres_sterownika,
			int16_t predkosc,
			uint8_t timeout,
			TodSilnik *buf_wyn){
	//int const maska = TIOCM_RTS;	//do zmiany sygnalu RTS
	//int lsr;
	unsigned char crc_hi, crc_lo;
	unsigned char buf_out[10];
	unsigned char od_siln[150];
	uint32_t il_zn = 0;
	struct timespec t_req, t_rem;
	//TodSilnik *test_motor;
	//TodSilnik sui;

	if (uchwyt_portu == 0){
		return 0;
	}

	buf_out[0] = adres_sterownika;
	buf_out[1] = 0x01;
	buf_out[2] = 1; //0x0 wylaczane regulatory pednikow wszystkich
	buf_out[3] = predkosc;	//predkosc LO
	buf_out[4] = predkosc >> 8;	//predkosc HI
	CRC16(buf_out, 5, &buf_out[6], &buf_out[5]);
	write(uchwyt_portu, buf_out, 7);

	//do ioctl(uchwyt_portu, TIOCSERGETLSR, &lsr); //czekanie na wyslanie znaku
	//while (!(lsr & TIOCSER_TEMT));
	//ioctl(uchwyt_portu,TIOCMBIC,&maska);	//wlaczenie nasluchu z RS-485

	//printf("Data sent to motor\n");
	il_zn = CzekajNaZnaki(uchwyt_portu, adres_sterownika, 19, timeout, od_siln);
	//printf("Received: %i\n", il_zn);

	if (il_zn >= 19) {		//na pełną odpowiedź sterownik potrzebuje min. 6 ms
		CRC16(od_siln, 17, &crc_hi, &crc_lo);
		if (crc_hi == od_siln[18] && crc_lo == od_siln[17]){
			memcpy(buf_wyn, od_siln, sizeof(TodSilnik));
			//sui = *(TodSilnik*)od_siln
			//zresetowanie flag bledow
			//test_motor = (TodSilnik *)buf_wyn;
			/*
			printf("%i, V: %i, ERR: 0x%X\n",
					motor_cnt,
					test_motor -> cisnienie,
					test_motor -> bledy);
			*/
			if (uchwyt_portu == p_poziom){
				ModoWKBP.Niespr[1]&=~(1<<(adres_sterownika-1)); //wyzerowanie flagi braku komunikacji
				ped_mar_odp[adres_sterownika - 1] = 0;
			}
			if (uchwyt_portu == p_pion){
				ModoWKBP.Niespr[1]&=~(1<<(adres_sterownika+3)); //wyzerowanie flagi braku komunikacji
				ped_man_odp[adres_sterownika-1]=0;
			}
		}
		else {
			if (uchwyt_portu == p_poziom){
				ped_mar_odp[adres_sterownika-1]++;
			}
			if (uchwyt_portu == p_pion){
				ped_man_odp[adres_sterownika - 1]++;
			}
		}
	}
	else {
		if (uchwyt_portu == p_poziom){
			ped_mar_odp[adres_sterownika - 1]++;
		}
		if (uchwyt_portu == p_pion){
			ped_man_odp[adres_sterownika - 1]++;
		}
	}

	if (ped_mar_odp[adres_sterownika - 1] > 10){
		ModoWKBP.Niespr[1] |= 0x01 << (adres_sterownika - 1);
	}
	if (ped_man_odp[adres_sterownika - 1] > 10){
		ModoWKBP.Niespr[1] |= 0x01 << (adres_sterownika + 3);
	}

	ModoPG.bledy[1] = ModoWKBP.Niespr[1];	//przepisanie flag braków komunikacji 
	ModoPG.bledy[2] = ModoWKBP.Niespr[2];	//przepisanie flag usterek

	//RS485_DELAY_AFTER_TRANSFER;
	t_req.tv_sec = 0;
	t_req.tv_nsec = 1000000;
	nanosleep(&t_req, &t_rem);

	return il_zn;
}


/**
 *
 * Set new address for motor controller
 *
 */
int NadajAdresSilnika(int32_t uchwyt_portu,
					uint8_t adres_sterownika,
					uint8_t timeout){

	//int const maska = TIOCM_RTS;	//do zmiany sygnalu RTS
	int lsr;
	unsigned char crc_hi, crc_lo;
	unsigned char buf_out[10];
	unsigned char od_siln[150];
	short il_zn = 0;
	//TodSilnik sui;
	struct timespec t_req, t_rem;

	buf_out[0] = 0x00;	//dotychczasowy adres sterownika (zawsze 0)
	buf_out[1] = 0x10;	//kod funkcji
	buf_out[2] = adres_sterownika;	//nowy adres sterownika
	CRC16(buf_out, 3, &buf_out[4], &buf_out[3]);
	write(uchwyt_portu, buf_out, 5);

	do ioctl(uchwyt_portu,TIOCSERGETLSR,&lsr); //czekanie na wyslanie znaku
	while (!(lsr&TIOCSER_TEMT));
	//ioctl(uchwyt_portu,TIOCMBIC,&maska);	//wlaczenie nasluchu z RS-485

	il_zn = CzekajNaZnaki(uchwyt_portu, adres_sterownika, 4, timeout, od_siln);

	if (il_zn >= 4) {		//na pełną odpowiedź sterownik potrzebuje min. 6 ms
		CRC16(od_siln, 2, &crc_hi, &crc_lo);
		if (crc_hi == od_siln[3] && crc_lo == od_siln[2]) {
			printf("New Addres OK: %d\n", adres_sterownika);
		} else printf("New Addres FAILED: %d\n",adres_sterownika);
	}
	else{
		printf("New addres procedure error: %d\n",adres_sterownika);
	}

	//RS485_DELAY_AFTER_TRANSFER;
	t_req.tv_sec = 0;
	t_req.tv_nsec = 1000000;
	nanosleep(&t_req, &t_rem);

	return 1;
}


/**
 *
 * Read response data from motor controller
 *
 */
int CzekajNaZnaki(int uchwyt_portu,
					unsigned char adres_sterownika,
					unsigned short il_zn_do_odebrania,
					unsigned short timeout,
					unsigned char buf_in[100]){
	int il_odeb_znak = 0;		//ilość odebranych znaków
	unsigned char bufor[100], i;
	int res;
	struct timeval t1, t2;
    unsigned int usTime = 0;	//globalny licznik mikrosekund
	struct timespec t_req, t_rem;

	//start timer
	gettimeofday(&t1, NULL);

	t_req.tv_sec = 0;
	t_req.tv_nsec = 1000000; //1 ms
	nanosleep(&t_req, &t_rem);

	while ((il_odeb_znak < il_zn_do_odebrania) && (usTime < (timeout * 1000))) {
		//usleep(1000); //wait for data from motor controller
		nanosleep(&t_req, &t_rem);

		res = read(uchwyt_portu, bufor, il_zn_do_odebrania);

		if (res > 0){
			recv_bytes += res;
			//printf("%i, %i, Received %i\n", motor_cnt, recv_bytes, res);
			for (i = 0; i < res; i++) {
			  	if (bufor[i] == adres_sterownika && il_odeb_znak == 0) {
					buf_in[0] = bufor[i];
					il_odeb_znak = 1;
				}
				else if (il_odeb_znak > 0){
					buf_in[il_odeb_znak++] = bufor[i];
				}
			}
		}

		gettimeofday(&t2, NULL);
		usTime = (t2.tv_sec - t1.tv_sec)*1000000;      // sec to us
		usTime += (t2.tv_usec - t1.tv_usec);   // us
		//printf("usTime: %i\n", usTime);
	}
	//printf("%i, Bytes: %i, timeout: %i\n", main_loop_cnt, il_odeb_znak, timeout);
	return il_odeb_znak;
}


/**
 *
 * Remote console reader
 *
 */
void *remote_console_fun(void *arg){
	struct timespec t_req, t_rem;
	TPulpitZdalny p_zdal;
	int16_t v;

	arg = arg;

	t_req.tv_sec = 0;
	t_req.tv_nsec = RC_PERIOD_MS * 1000000;
	while(1){
		nanosleep(&t_req, &t_rem);
		get_pulpit_zdalny(&p_zdal);
		if (p_zdal.joy_prawy_y > 3){
			v = 28 * p_zdal.joy_prawy_y + 417;
		}
		else if (p_zdal.joy_prawy_y < -3){
			v = 28 * p_zdal.joy_prawy_y - 417;
		}
		else{
			v = 0;
		}

		if (v > V_MAX){
			v = V_MAX;
		}
		else if (v < -V_MAX){
			v = -V_MAX;
		}

		pednik_mar[0] = v;
		//printf("x: %i, y: %i, v: %i\n",
		//		p_zdal.joy_prawy_x, p_zdal.joy_prawy_y, v);
	}

	return NULL;
}


/**
 *
 * reading keys from keyboard
 * arrow up - increase speed
 * arrow down - decrease speed
 * q - exit
 *
 */
void *keyboard_thread_fun(void *arg){
	int c = 0x30, step = 0;
	static struct termios oldt, newt;

	arg = arg;

	//get console settings
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);

	printf("Keyboard reader is running\n");

    while (1) {
        //puts("enter a line");
		while((c = getchar()) != 'q'){
			//putchar(c);
			//printf("%X ", c);
			if (c == 0x1b){
				step = 1;
			}
			else if ((step == 1) && (c == 0x5b)){
				step = 2;
			}
			else if ((step == 2) && (c == 0x41)){
				//arrow UP pressed
				//printf("UP\n");
				step = 0;
				if (pednik_mar[0] == 0){
					pednik_mar[0] = V_MIN;
					pednik_mar[1] = V_MIN;
					pednik_mar[2] = V_MIN;
					pednik_mar[3] = V_MIN;
				}
				else if (pednik_mar[0] == -V_MIN){
					pednik_mar[0] = 0;
					pednik_mar[1] = 0;
					pednik_mar[2] = 0;
					pednik_mar[3] = 0;
				}
				else {
					pednik_mar[0] += 100;
					pednik_mar[1] += 100;
					pednik_mar[2] += 100;
					pednik_mar[3] += 100;
				}
				//check max velocity
				if (pednik_mar[0] > V_MAX){
					pednik_mar[0] = V_MAX;
					pednik_mar[1] = V_MAX;
					pednik_mar[2] = V_MAX;
					pednik_mar[3] = V_MAX;
				}
				//printf("Current speed: %i\n", pednik_mar[0]);
			}
			else if ((step == 2) && (c == 0x42)){
				//arrow DOWN pressed
				//printf("DOWN\n");
				step = 0;
				if (pednik_mar[0] == 0){
					pednik_mar[0] = -V_MIN;
					pednik_mar[1] = -V_MIN;
					pednik_mar[2] = -V_MIN;
					pednik_mar[3] = -V_MIN;
				}
				else if (pednik_mar[0] == V_MIN){
					pednik_mar[0] = 0;
					pednik_mar[1] = 0;
					pednik_mar[2] = 0;
					pednik_mar[3] = 0;
				}
				else {
					pednik_mar[0] -= 100;
					pednik_mar[1] -= 100;
					pednik_mar[2] -= 100;
					pednik_mar[3] -= 100;
				}
				//check max velocity
				if (pednik_mar[0] < -V_MAX){
					pednik_mar[0] = -V_MAX;
					pednik_mar[1] = -V_MAX;
					pednik_mar[2] = -V_MAX;
					pednik_mar[3] = -V_MAX;
				}
				//printf("Current speed: %i\n", pednik_mar[0]);
			}
			else {
				step = 0;
			}
		}
		tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
		finish_app = true;
		printf("finish keyboard thread\n");
		break;
    }
    return 0;
}


/**
 *
 * periodicaly display parameters received from
 * motor controler
 *
 */
void *display_param_fun(void *arg){
	struct timespec t_req, t_rem;
	int i = 0;

	if (modbus_addr > 4){
		i = 0;
	}
	else if (modbus_addr == 0){
		i = 0;
	}
	else{
		i = modbus_addr - 1;
	}

	arg = arg;
	t_req.tv_sec = DISPLAY_PERIOD_S;
	t_req.tv_nsec = 0;
	while(1) {
		nanosleep(&t_req, &t_rem);
		printf("%5i| V: %4i; U: %4.2f; I: %4i; Tm: %d; Ts: %d; p: %d; err: 0x%02X\n",
				disp_cnt++,
				poziom_glob[i].pred,
				(float)((float)poziom_glob[i].nap)/1000,
				poziom_glob[i].prad,
				poziom_glob[i].temp_modul,
				poziom_glob[i].temp_siln,
				poziom_glob[i].cisnienie,
				poziom_glob[i].bledy);
	}

}


void CRC16(uint8_t ramka[], uint8_t m, uint8_t *crc_h, uint8_t *crc_l){
	uint16_t CRC = 0xFFFF;
	uint8_t x=0,  //aktualny numer bajtu wiadomosci
			p;	//zmienna pomocnicza dla petli

	while (m > 0) {
		CRC ^= ramka[x];
		for (p = 0; p < 8; p++) {
			if (CRC & 1) {
				CRC >>= 1;
				CRC ^= 0xA001;
			}
			else CRC >>= 1;
		}
		m--;
		x++;
	}
	*crc_h = (CRC >> 8) & 0xFF;
	*crc_l = CRC & 0xFF;
}


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
		finish_app = true;
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
