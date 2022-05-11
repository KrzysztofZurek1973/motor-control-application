#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <strings.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>

#include "pulpit_zdalny.h"

#define P_ZDAL_TIMER_PERIOD_MS 10
#define P_ZDAL_TIMEOUT_MS 400

int pulpit_zdalny_handle = -1;
bool pulpit_zdalny_ready = true;
unsigned long int pulpit_zdalny_odp;
bool p_zdal_alive = false;

static pthread_mutex_t pulpit_zdalny_mux = PTHREAD_MUTEX_INITIALIZER;

TPulpitZdalny p_zdal_data;
pthread_t p_zdal_thread, p_zdal_timer_thread;

void *pulpit_zdalny_recv_fun(void *arg);
void *pulpit_zdalny_timer_fun(void *arg);

//uint16_t calc_crc(uint8_t *b, uint8_t len);
//unsigned int _crc_ccitt_update(unsigned int crc, unsigned char data);

/**
 *
 * Initialization
 */
int init_pulpit_zdalny(char *dev, int baudrate){
	struct termios newtio;

	memset(&p_zdal_data, 0, sizeof(p_zdal_data));
	pulpit_zdalny_handle = open(dev, O_RDWR | O_NOCTTY);
	if (pulpit_zdalny_handle < 0) {
		printf("Pulpit zdalny failed\n");
		return -1;
	}
	//serial port configuration, always wait for 36 bytes
	bzero(&newtio, sizeof(newtio));
	newtio.c_cflag = baudrate | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;
	newtio.c_cc[VTIME] = 0;	//200ms
	newtio.c_cc[VMIN] = 36; //always wait for min 36 characters
	tcflush(pulpit_zdalny_handle, TCIOFLUSH);
	tcsetattr(pulpit_zdalny_handle, TCSANOW, &newtio);

	//create receiver thread
	int pzs = pthread_create(&p_zdal_thread,
							NULL,
							pulpit_zdalny_recv_fun,
							NULL);
	if (pzs != 0){
		printf("Pul. zdalny thread NOT created\n");
		return -1;
	}

	//create timer thread
	int pzt = pthread_create(&p_zdal_timer_thread,
							NULL,
							pulpit_zdalny_timer_fun,
							NULL);
	if (pzt != 0){
		printf("Pul. zdalny TIMER thread NOT created\n");
		return -1;
	}

	return 1;
}


void *pulpit_zdalny_timer_fun(void *arg){
	struct timespec t_req, t_rem;
	bool send_request = false;

	arg = arg; //for compiler
	t_req.tv_sec = 0;
	t_req.tv_nsec = P_ZDAL_TIMER_PERIOD_MS * 1000000;
	if (pulpit_zdalny_handle >= 0){
		while(1){
			nanosleep(&t_req, &t_rem);
			send_request = false;
			pulpit_zdalny_odp += P_ZDAL_TIMER_PERIOD_MS;
			if (pulpit_zdalny_ready == true){
				pthread_mutex_lock(&pulpit_zdalny_mux);
				pulpit_zdalny_ready = false;
				pthread_mutex_unlock(&pulpit_zdalny_mux);

				send_request = true;
			}
			else{
				//if during timeout period there was no answer try again
				pthread_mutex_lock(&pulpit_zdalny_mux);
				if (pulpit_zdalny_odp >= P_ZDAL_TIMEOUT_MS){
					//pthread_mutex_lock(&pulpit_zdalny_mux);
					pulpit_zdalny_odp = 0;
					p_zdal_alive = false;
					memset(&p_zdal_data, 0, sizeof(p_zdal_data));
					//pthread_mutex_unlock(&pulpit_zdalny_mux);

					send_request = true;
				}
				pthread_mutex_unlock(&pulpit_zdalny_mux);
			}
			if (send_request == true){
				tcflush(pulpit_zdalny_handle, TCIOFLUSH);
				write(pulpit_zdalny_handle, "x31z", 4);
			}
		}
	}
	else{
		printf("Pulpit zdalny not running\n");
	}

	return NULL;
}


/**
 *
 * Return status of live
 *
 */
bool pulpit_zdalny_is_alive(void){
	bool res;

	pthread_mutex_lock(&pulpit_zdalny_mux);
	res = p_zdal_alive;
	pthread_mutex_unlock(&pulpit_zdalny_mux);

	return res;
}


/**
 *
 */
void get_pulpit_zdalny(TPulpitZdalny *out){

	pthread_mutex_lock(&pulpit_zdalny_mux);
	*out = p_zdal_data;
	pthread_mutex_unlock(&pulpit_zdalny_mux);
}

/**
 ***************************************************
 *
 * odbior danych z pulpitu zdalnego IREL
 *
 **************************************************/
void *pulpit_zdalny_recv_fun(void *arg){
	//unsigned short crc_recv, crc_calc;
	TPulpitZdalny p_zdal;
	uint8_t buff[50];
	uint8_t liczba;
	char s[20];
	int res;
	//uint16_t crc_calc, crc_recv;

	arg = arg;
	printf("Pulpit zdalny receiver started OK\n");

	for(;;){
		memset(buff, 0, 50);
		res = read(pulpit_zdalny_handle, buff, 50);

		if (res == 36){
			buff[36] = 0;
			if (buff[0] == 'x' && buff[1] == '3' && buff[2] == '1' &&
				buff[14] == 0x30 && buff[35]=='z'){
				if (buff[13] == 0x38){
					//jeśli jest załączona część zdalna (jajo)
					//joystick Lewy oś X
					s[0] = buff[12];
					s[1] = buff[9];
					s[2] = 0;
					sscanf(s, "%x", (unsigned int *)&liczba);
					p_zdal.joy_lewy_x = liczba & 0x3F;
					if (p_zdal.joy_lewy_x >= 32) {
						p_zdal.joy_lewy_x = (p_zdal.joy_lewy_x - 32) * 3;
					}
					else if (p_zdal.joy_lewy_x < 32) {
						p_zdal.joy_lewy_x = (32 - p_zdal.joy_lewy_x) * -3;
					}

					//joystick Lewy oś Y
					//sprintf(s, "%c%c", buff[11], buff[12]);	//zamiana na ASCII
					s[0] = buff[11];
					s[1] = buff[12];
					s[2] = 0;
					sscanf(s, "%x", (unsigned int *)&liczba);
					p_zdal.joy_lewy_y = liczba >> 2;
					if (p_zdal.joy_lewy_y >= 32) {
						p_zdal.joy_lewy_y = (p_zdal.joy_lewy_y - 32) * 3;
					}
					else if (p_zdal.joy_lewy_y < 32) {
						p_zdal.joy_lewy_y = (32 - p_zdal.joy_lewy_y) * -3;
					}

					//joystick Prawy oś X
					//sprintf(s, "%c%c", buff[7], buff[8]);	//zamiana na ASCII
					s[0] = buff[7];
					s[1] = buff[8];
					s[2] = 0;
					sscanf(s, "%x", (unsigned int *)&liczba);
					p_zdal.joy_prawy_x = liczba & 0x3F;
					if (p_zdal.joy_prawy_x >= 32) {
						p_zdal.joy_prawy_x = (p_zdal.joy_prawy_x - 32) *3;
					}
					else if (p_zdal.joy_prawy_x<32) {
						p_zdal.joy_prawy_x = (32 - p_zdal.joy_prawy_x) * -3;
					}

					//joystick Prawy oś Y
					//sprintf(s, "%c%c", buff[10], buff[7]);		//zamiana na ASCII
					s[0] = buff[10];
					s[1] = buff[7];
					s[2] = 0;
					sscanf(s, "%x", (unsigned int *)&liczba);
					p_zdal.joy_prawy_y = liczba >> 2;
					if (p_zdal.joy_prawy_y >= 32) {
						p_zdal.joy_prawy_y = (p_zdal.joy_prawy_y - 32) * -3;
					}
					else if (p_zdal.joy_prawy_y < 32) {
						p_zdal.joy_prawy_y = (32 - p_zdal.joy_prawy_y) * 3;
					}

					//przyciski i przełączniki
					if (buff[15] & 0x04){
						p_zdal.Cofnij = 1;
					}
					else {
						p_zdal.Cofnij = 0;
					}
					if (buff[18] & 0x01){
						p_zdal.Wysun = 1;
					}
					else{
						p_zdal.Wysun = 0;
					}
					if (buff[18]&0x08) p_zdal.Otworz=1; else p_zdal.Otworz=0;
					if (buff[18]&0x04) p_zdal.Zamknij=1; else p_zdal.Zamknij=0;
					if (buff[19]&0x01) p_zdal.ToczekB=1; else p_zdal.ToczekB=0;
					if (buff[20]&0x02) p_zdal.ZwalnianieLad=1; else p_zdal.ZwalnianieLad=0;
					if (buff[20]&0x01) p_zdal.LadujToczekA=1; else p_zdal.LadujToczekA=0;
					if (buff[17]&0x08) p_zdal.ZwolnijToczekA=1; else p_zdal.ZwolnijToczekA=0;
					if (buff[20]&0x04) p_zdal.WindaDol=1; else p_zdal.WindaDol=0;
					if (buff[20]&0x08){
						p_zdal.WindaGora=1;
					}
					else {
						p_zdal.WindaGora=0;
					}
					//bieg pionowy
					if (buff[17]&0x04) {
						p_zdal.BiegPoziom = 1;
					}
					else {
						p_zdal.BiegPoziom = 0;
					}
					if (buff[17]&0x02) p_zdal.BiegPion=1; else p_zdal.BiegPion=0;
					if (buff[17]&0x01) p_zdal.BiegObr=1; else p_zdal.BiegObr=0;

					//update global variables
					pthread_mutex_lock(&pulpit_zdalny_mux);
					p_zdal_data = p_zdal;
					pulpit_zdalny_odp = 0;
					pulpit_zdalny_ready = true;
					p_zdal_alive = true;
					pthread_mutex_unlock(&pulpit_zdalny_mux);
				}
				else if (buff[13] == 0x34){
					//czesc radiowa pulpitu jest wylaczony
					pthread_mutex_lock(&pulpit_zdalny_mux);
					p_zdal_alive = false;
					pulpit_zdalny_odp = 0;
					pulpit_zdalny_ready = true;
					memset(&p_zdal_data, 0, sizeof(p_zdal_data));
					pthread_mutex_unlock(&pulpit_zdalny_mux);
				}
			}
		} //if(res == 36)
	}//for
}

/*
//CRC calculation
//code received from IREL (2021-12-20)
uint16_t calc_crc(uint8_t *b, uint8_t len){
	uint16_t crc;
	crc = 0xffff;
	while(len--) {
		crc = _crc_ccitt_update(crc, *b++);
	}
	return crc;
}


uint32_t _crc_ccitt_update (uint32_t crc, uint8_t data){
	data ^= (uint8_t)(crc & 0xff);
	data ^= data << 4;

	return ((((uint32_t)data << 8) | (crc >> 8)) ^ (uint32_t)(data >> 4)
                ^ ((uint32_t)data << 3));
}
*/
