//motors.h

#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <stdlib.h>		//tu jest def exit();
//#include <linux/rtc.h> 
#include <fcntl.h>
#include <termios.h> 
//#include <signal.h>
//#include <arpa/inet.h>
//#include <netdb.h>
//#include <netinet/in.h>
//#include <sys/socket.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
//#include <sys/select.h>
#include <unistd.h>
#include <math.h>

typedef struct {
	uint8_t address;
	uint8_t function;	//0x01 set speed
	int16_t speed;	//RPM
	uint16_t power_supp_volt;	//V
	uint16_t current;	//mA
	uint8_t error;
	uint8_t not_used;	//padding
	uint16_t crc;
} aux_motor_resp_t;

typedef struct{
	uint8_t address;
	uint8_t function;
	uint8_t max_current;	//0.1A
	uint8_t not_used;		//padding
	int16_t speed;
	uint16_t crc;
} aux_motor_req_t;

				
void *motors_speed_func(void *arg);

bool mor_thread_is_running();
//void timer_18ms(void);
//void *automatyka_func(void *cookie);
//void *glowny_func(void *cookie);
//void *pedniki_func(void *cookie);
bool new_data_received(void);
int motors_set_new_speed(int16_t new_speed[5]);
void *motor_receiver_fun();
