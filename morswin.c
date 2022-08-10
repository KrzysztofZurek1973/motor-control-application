#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>
#include <strings.h>
#include <stdint.h>
#include <time.h>

#include "morswin.h"
#include "crc_mor.h"
#include "uart.h"
#include "main.h"

#define TIMER_PERIOD_MS 1

extern uint8_t ped_man_odp[4];
extern int16_t pednik_mar[4];
extern uint8_t modbus_addr;
extern TodSilnik poziom_glob[4];

int16_t pednik_man[4];
int32_t p_poziom, p_pion = 0;
uint32_t motor_cnt = 0;
TModoPG ModoPG;
TModoWKBP ModoWKBP;
uint8_t ped_mar_odp[4] = {0, 0, 0, 0};
uint32_t recv_bytes = 0;

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

/**
 *
 * Sending current speed to motor controllers
 *
 */
void *pedniki_func(void *arg){
	struct timespec t_req, t_rem;
	int uart_fd = -1;
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
	uart_fd = open_uart_port(UART_PORT, BAUDRATE, 0, 0);
	if (uart_fd < 0){
		printf("Communication with motor not possible\nUART open failed\n");
		exit(1);
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
