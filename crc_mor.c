#include <stdint.h>

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
