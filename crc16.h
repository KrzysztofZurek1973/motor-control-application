/*
 * crc16.h
 *
 *  Created on: 7 mar 2022
 *      Author: Piotr
 */

#ifndef APPLICATION_USER_CRC16_H_
#define APPLICATION_USER_CRC16_H_

uint16_t crc_16(uint8_t *input_str, uint8_t num_bytes);
uint16_t crc_modbus(uint8_t *input_str, uint8_t num_bytes);

#endif /* APPLICATION_USER_CRC16_H_ */
