#ifndef _PULPIT_ZDALNY_H
#define _PULPIT_ZDALNY_H

typedef struct {
	short joy_lewy_x;
	short joy_lewy_y;
	short joy_prawy_x;
	short joy_prawy_y;
	unsigned char Cofnij : 1;
	unsigned char Wysun : 1;
	unsigned char Otworz : 1;
	unsigned char Zamknij : 1;
	unsigned char ToczekB : 1;
	unsigned char ZwalnianieLad : 1;
	unsigned char LadujToczekA : 1;
	unsigned char ZwolnijToczekA : 1;
	unsigned char WindaDol : 1;
	unsigned char WindaGora : 1;
	unsigned char BiegPoziom : 1;
	unsigned char BiegPion : 1;
	unsigned char BiegObr : 1;
} TPulpitZdalny;

int init_pulpit_zdalny(char *dev, int baudrate);
void get_pulpit_zdalny(TPulpitZdalny *out);
bool pulpit_zdalny_is_alive(void);

#endif
