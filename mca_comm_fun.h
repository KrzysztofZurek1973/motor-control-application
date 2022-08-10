#ifndef _MCA_COMM_FUN_H_
#define _MCA_COMM_FUN_H_

int mca_start_bootloader(uint8_t addr);
int mca_write_serial_number(uint8_t addr, uint16_t sn);
int mca_write_modbus_addr(uint8_t addr);
int mca_read_serial_number(uint8_t addr);
int mca_read_modbus_addr(void);
int mca_set_boot_bit(uint8_t addr, int boot_bit);
int mca_mor_prot_activate(uint8_t addr, int input);
int mca_diodes_test(int addr);

#endif
