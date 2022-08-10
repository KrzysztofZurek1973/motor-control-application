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
#include <time.h>

#include "main.h"
#include "morswin.h"
#include "pulpit_zdalny.h"
#include "crc16.h"
#include "uart.h"
#include "mca_comm_fun.h"

#define MAIN_TIMER_PERIOD_S 5
#define V_MAX 3000		//maximum motort speed [rpm]
#define V_MIN 500		//minimum motort speed [rpm]
#define KEY_STEP 100	//keyboard increment/decrement speed step [rpm]
#define DISPLAY_PERIOD_S 2
#define RC_PERIOD_MS 10

char help_str[] =	"Motor Controller Application\n"\
					"version 2022-08-08\n---------------------\n"\
					"\trun: ./mca\n"\
					"options:\n"\
					"\t-r - read\n"\
					"\t-w - write\n"\
					"\t-a - address\n"\
					"\t-s - serial number\n"\
					"\t-d - diodes test (not used)\n"\
					"\t-v - speed\n"\
					"\t-b - start bootloader\n"\
					"\t-o - activate nBOOT1 bit (1), deactivate (0)\n"\
					"\t-m - 1: morswin control active (default)\n"\
					"\t     0: motor not controled by morswin protocol\n"\
					"\t-h - this help\n"\
					"examples:\n"\
					"\t-a 1          communication only with CTRL address 1\n"\
					"\t-r -a 0       read Modbus address\n"\
					"\t-r -a 1 -s 0  read serial number\n"\
					"\t-w -a 0       reset Modbus address (set as 0)\n"\
					"\t-w -a 2       set Modbus address as 2\n"\
					"\t\t      possible only if current address is 0\n"\
					"\t-w -a 1 -s 9  set serial number to 9\n"\
					"\t-a 1 -v 1000  set speed to 1000 RPM\n"\
					"\t-a 1 -b       start bootloader on CTRL with address 1\n"\
					"\t-a 1 -o 1     set nBOOT1 bit on CTRL with address 1\n"\
					"\t-a 1 -m 0     deactivate morswin speed control";

//global variables
int uart_fd = -1;
uint8_t modbus_addr = MODBUS_MAX_ADDR + 1;


//int32_t p_poziom, p_pion = 0;
bool finish_app;
pthread_t p_keyboard_thread, p_mc_thread, rc_thread;
uint32_t main_loop_cnt = 0;
uint8_t ipnna = 0;
uint8_t ped_man_odp[4] = {0, 0, 0, 0};
int16_t pednik_mar[4];
int16_t current_speed = 0;
bool keyboard_activated = false;
uint32_t test_cnt = 0;
uint8_t mors_protocol_deactivate = 0;

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

//functions

/**
 *
 * Main loop
 *
 */
int main(int argc, char *argv[]) {
    struct timespec t_req, t_rem;
	int option;
	char *cvalue = NULL;
	bool read_mode = false, write_mode = false;
	bool set_new_speed = false, set_serial = false, diode_test = false;
	bool set_new_addr = false;
	int new_speed = 0, new_serial_number = 0;
	bool setting_mode = false;
	bool bootloader_run = false;
	bool set_boot_bit = false;
	int nboot1_val;

	modbus_addr = 1; //default address

	if (argc > 1){
		while ((option = getopt(argc, argv, "rwv:a:s:hdbo:m:")) != -1){
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
				
			case 'm':
				cvalue = optarg;
				if (cvalue != NULL){
					mors_protocol_deactivate = atoi(cvalue);
					if (mors_protocol_deactivate >= 1){
						mors_protocol_deactivate = 2;
					}
					else{
						mors_protocol_deactivate = 1;
					}
					setting_mode = true;
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
			//if ((set_new_addr == true) && (set_serial == false)){
			if ((modbus_addr <= MODBUS_MAX_ADDR) && (set_serial == false)){
				//read modbus address
				mca_read_modbus_addr();
			}
			else if (set_serial == true){
				mca_read_serial_number(modbus_addr);
			}
			else{
				printf("read option not correct\n");
			}
		}
		else if (write_mode == true){
			//printf("write mode\n");
			if ((set_new_addr == true) && (set_serial == false)){
				mca_write_modbus_addr(modbus_addr);
			}
			else if (set_serial == true){
				mca_write_serial_number(modbus_addr, new_serial_number);
			}
			else{
				printf("unknown write option\n");
			}
		}
		else{
			if (diode_test == true){
				mca_diodes_test(modbus_addr);
			}
			else if (bootloader_run == true){
				//start bootloader
				mca_start_bootloader(modbus_addr);
			}
			else if (set_boot_bit == true){
				//set nBoot1 in option bytes
				mca_set_boot_bit(modbus_addr, nboot1_val);
			}
			else if (mors_protocol_deactivate > 0){
				//activate/deactivate control by morswin protocol
				//deactivation causes only no speed setting in controller
				//function MC_ProgramSpeedRampMotor1_F is not called
				mca_mor_prot_activate(modbus_addr, mors_protocol_deactivate);
			}
		}
		printf("Finished\n");
		exit(EXIT_SUCCESS);
	}
	
	if (set_new_speed == true){
		//start with fixed speed level
		printf("Set speed: %i rpm\n", new_speed);
		printf("CTRL address: %i\n", modbus_addr);
				
		pednik_mar[0] = new_speed;
		pednik_mar[1] = new_speed;
		pednik_mar[2] = new_speed;
		pednik_mar[3] = new_speed;
		//avoid reading radio console
		keyboard_activated = true;
	}

	finish_app = false;
	if (modbus_addr > MODBUS_MAX_ADDR){
		printf("Failed: CTRL address error\n");
		exit(EXIT_SUCCESS);
	}
	//create motor controller writer
	int pmc = pthread_create(&p_mc_thread,
							NULL,
							pedniki_func,
							NULL);
	if (pmc != 0){
		printf("Motor controller writer thread NOT created\n");
		finish_app = true;
	}

	init_pulpit_zdalny("/dev/ttyS0", B9600);

	//create keyboard reader
	int pkey = pthread_create(&p_keyboard_thread,
							NULL,
							keyboard_thread_fun,
							NULL);
	if (pkey != 0){
		printf("Keyboard reader thread NOT created\n");
		finish_app = true;
	}

	//create display writer thread
	if (finish_app == false){
		int pdisp = pthread_create(&p_display_param_thread,
								NULL,
								display_param_fun,
								NULL);
		if (pdisp != 0){
			printf("Parameter display thread NOT created\n");
			finish_app = true;
		}
	}

	//create remote console reader
	if (finish_app == false){
		int pdrc = pthread_create(&rc_thread,
								NULL,
								remote_console_fun,
								NULL);
		if (pdrc != 0){
			printf("RC thread NOT created\n");
			finish_app = true;
		}
	}

	t_req.tv_sec = MAIN_TIMER_PERIOD_S;
	t_req.tv_nsec = 0;

	while (finish_app == false){
		nanosleep(&t_req, &t_rem);
		//printf("%i\n", main_loop_cnt++);
	}
	printf("Finished\n");
	exit(EXIT_SUCCESS);
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
		if (keyboard_activated == false){
			//printf("%i, read p_zdal\n", test_cnt++);
			if (pulpit_zdalny_is_alive() == true){
				get_pulpit_zdalny(&p_zdal);
				if (p_zdal.joy_prawy_y > 3){
					//v = 28 * p_zdal.joy_prawy_y + 417;
					v = 24 * p_zdal.joy_prawy_y + 260;
				}
				else if (p_zdal.joy_prawy_y < -3){
					//v = 28 * p_zdal.joy_prawy_y - 417;
					v = 24 * p_zdal.joy_prawy_y - 260;
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
			}
			else{
				v = 0;
			}
			pednik_mar[0] = v;
			pednik_mar[1] = v;
			pednik_mar[2] = v;
			pednik_mar[3] = v;
		}
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
	short v_temp = 0;

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
				step = 0;
				if (pednik_mar[0] == 0){
					v_temp = V_MIN;
					keyboard_activated = true;
				}
				else {
					v_temp = pednik_mar[0] + 100;
				}
				//check max velocity
				if (pednik_mar[0] >= V_MAX){
					v_temp = V_MAX;
				}
				else if (abs(v_temp) < V_MIN){
					v_temp = 0;
					keyboard_activated = false;
				}
				//printf("Current speed: %i\n", pednik_mar[0]);
				pednik_mar[0] = v_temp;
				pednik_mar[1] = v_temp;
				pednik_mar[2] = v_temp;
				pednik_mar[3] = v_temp;
				//printf("%i, UP, %i\n", test_cnt++, v_temp);
			}
			else if ((step == 2) && (c == 0x42)){
				//arrow DOWN pressed
				step = 0;
				if (pednik_mar[0] == 0){
					v_temp = -V_MIN;
					keyboard_activated = true;
				}
				else {
					v_temp = pednik_mar[0] - 100;
				}
				//check max velocity
				if (pednik_mar[0] <= -V_MAX){
					v_temp = -V_MAX;
				}
				else if (abs(v_temp) < V_MIN){
					v_temp = 0;
					keyboard_activated = false;
				}
				pednik_mar[0] = v_temp;
				pednik_mar[1] = v_temp;
				pednik_mar[2] = v_temp;
				pednik_mar[3] = v_temp;
				//printf("%i, DOWN, %i\n", test_cnt++, v_temp);
				//printf("Current speed: %i\n", pednik_mar[0]);
			}
			else {
				step = 0;
			}
		}
		tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
		//finish_app = true;
		printf("Finish motor-control-app\n");
		exit(EXIT_SUCCESS);
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
		printf("%5i| Vz: %4i, V: %4i; U: %4.2f; I: %4i; Tm: %d; Ts: %d; p: %d; err: 0x%02X\n",
				disp_cnt++,
				pednik_mar[0],
				poziom_glob[i].pred,
				(float)((float)poziom_glob[i].nap),
				poziom_glob[i].prad,
				poziom_glob[i].temp_modul,
				poziom_glob[i].temp_siln,
				poziom_glob[i].cisnienie,
				poziom_glob[i].bledy);
		if (poziom_glob[i].temp_modul >= 70){
			printf("--- WYSOKA TEMPERATURA! ---\n");
		}
	}

}

