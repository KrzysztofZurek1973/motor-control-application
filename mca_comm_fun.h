#ifndef _MCA_COMM_FUN_H_
#define _MCA_COMM_FUN_H_

typedef enum {	MB_FUN_SET_SPEED = 0x01,
				MB_FUN_SET_NEW_ADDR = 0x41,
				MB_FUN_GET_ADDR = 0x44,
				MB_FUN_GET_SERIAL_NR = 0xA1,
				MB_FUN_SET_SERIAL_NR = 0xAA,
				MB_FUN_DIODES = 0x53,
				MB_FUN_BOOTLOADER = 0x7E,
				MB_FUN_BOOT1 = 0x22,
				MB_FUN_MOR_PROTO = 0xCC,
				MB_FUN_ROT_DIR = 0x33,
				MB_FUN_NONE = 0} modbus_fun_e;

int mca_start_bootloader(char *dev, uint8_t addr);
int mca_write_serial_number(char *dev, uint8_t addr, uint16_t sn);
int mca_write_modbus_addr(char *dev, uint8_t addr);
int mca_read_serial_number(char *dev, uint8_t addr);
int mca_read_modbus_addr(char *dev);
int mca_set_boot_bit(char *dev, uint8_t addr, int boot_bit);
int mca_mor_prot_activate(char *dev, uint8_t addr, int input);
int mca_diodes_test(char *dev, int addr);
int mca_set_rotation_dir(char *dev, uint8_t addr);

#endif
