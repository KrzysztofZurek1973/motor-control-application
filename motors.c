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
#include <semaphore.h>

#include "motors.h"
#include "crc16.h"
#include "uart.h"
#include "main.h"

#define TIMER_PERIOD_MS 50
#define RECV_BUFF_LEN 32

pthread_mutex_t motor_mux = PTHREAD_MUTEX_INITIALIZER;

extern uint8_t modbus_addr;
int16_t motor_speed[5];

int16_t pednik_man[4];
int32_t p_poziom, p_pion = 0;
uint16_t motor_cnt = 0;
uint32_t recv_bytes = 0;
bool new_data = false;
bool mor_thread_state = true; //true - thread is running
bool rs485_bus_free = true;
int uart_fd = -1;

bool mor_thread_is_running(){
    return mor_thread_state;
}

bool new_data_received(){
	return new_data;
}

/**
 *
 * Set new speed value
 *
 */
int motors_set_new_speed(int16_t *new_speed){
	motor_speed[0] = new_speed[0];
	motor_speed[1] = new_speed[1];
	motor_speed[2] = new_speed[2];
	motor_speed[3] = new_speed[3];
	motor_speed[4] = new_speed[4];
	
	printf("new speed: %i\n", motor_speed[0]);
	
	return 0;
}

/**
 *
 * GPS receiver thread function
 *
 */
void *motor_receiver_fun(){
	int n = 0, cnt = 0;
	char recv_buff[RECV_BUFF_LEN];
	aux_motor_resp_t recv_data;
	
	while(1){
		if (uart_fd > 0){
			n = read(uart_fd, recv_buff, RECV_BUFF_LEN);

			if (n > 0){
				recv_data = *(aux_motor_resp_t *)&recv_buff[0];
				//rs485_bus_free = true;
			
				printf("%i, speed: %i, vol: %i, current: %i, CRC: %04X\n",
						cnt++,
						recv_data.speed,
						recv_data.power_supp_volt,
						recv_data.current,
						recv_data.crc);
			
				new_data = true;
			}
			else{
				printf("%i, recv: TIMEOUT\n", cnt++);
				//rs485_bus_free = true;
				new_data = false;
			}
			pthread_mutex_lock(&motor_mux);
			rs485_bus_free = true;
			pthread_mutex_unlock(&motor_mux);
		}
		else{
			sleep(1);
		}
	}
}


/**
 *
 * Sending current speed to motor controllers
 * Tukan test version
 *
 */
void *motors_speed_func(void *arg){
	struct timespec t_req, t_rem;
	aux_motor_req_t speed_req;
	uint8_t cnt = 0;
	int j = 0;
	
	char *port = (char *)arg;
	uart_fd = open_uart_port(port, BAUDRATE, 2, 12);
	if (uart_fd < 0){
		printf("Serial port open ERROR\n");
		mor_thread_state = false;
		exit(EXIT_FAILURE);
	}
	else{
		printf("CTRL at %s opened\n", port);
	}
	printf("Motors thread is running\n");
	
	t_req.tv_sec = 0;
	t_req.tv_nsec = TIMER_PERIOD_MS * 1000000;
	
	while(1){
		motor_cnt++;
		//if (rs485_bus_free == true){
			speed_req.address = j;
			speed_req.function = 1;
			speed_req.max_current = 70;
			speed_req.not_used = cnt++;
			speed_req.speed = motor_speed[j];
			speed_req.crc = crc_modbus((uint8_t *)&speed_req, 6);
			//send data to motor CTRL
			write(uart_fd, &speed_req, 8);
		
			pthread_mutex_lock(&motor_mux);
			rs485_bus_free = false;
			pthread_mutex_unlock(&motor_mux);
		
			j++; //motor selector
			if (j > 4){
				j = 0;
			}
		//}
		nanosleep(&t_req, &t_rem);
	}
}


