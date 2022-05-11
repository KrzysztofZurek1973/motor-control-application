//#define SYMULATOR
//#define MORSWIN1  // oddzielnie zdefiniowana w MS_alokacja
#define MORSWIN2

#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <stdlib.h>		//tu jest def exit();
#include <linux/rtc.h> 
#include <fcntl.h>
#include <termios.h> 
#include <signal.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/select.h>
#include <unistd.h>
#include <math.h>

//#include "MS_alokacja.h"

#define ZAL 1
#define WYL 0
#define PZ1 20
#define PZ2 12
#define PZ3 43

#define OBR_KAM_PRED 800
#define OBR_SON_PRED 800

#define OBR_KAM_ZAKR 5750		//zakres z za³o¿on¹ obracark¹ LP
//2000 impulsów to 40 stopni
#define OBR_SON_ZAKR 5000

#define bit_set(p,m) ((p) |= (1<<m))
#define bit_clr(p,m) ((p) &= ~(1<<m))

#define RS485_DELAY_AFTER_TRANSFER ms_delay(2);

#define uint8 unsigned char
#define uint16 unsigned short


typedef struct {
	unsigned char Start;
	unsigned char Pojazd;
	unsigned char Ramka;
	unsigned short LiczRam;
	unsigned char Testy;
	unsigned char WlaczanieHydro;
	unsigned char Kamera1;
	unsigned char Kamera2;
	unsigned char StopienOsw[4];
	unsigned char ManipulZwalniak[2];
	unsigned char SterRoboczych;
	short Joysticki[6];
	float ZadKurs;
	float ZadPrzeg;
	float ZadGleb;
	float ZadEcho;
	float ZadPedPion;
	float ZadPredkosc;
	float ZadPozGleb;			__attribute__ ((aligned(8)))			
	double ZadPozDl;			__attribute__ ((aligned(8)))
	double ZadPozSzer;			
	unsigned char Precyzja;
	unsigned char JoystickTryb;
	unsigned char Regulatory;
	unsigned char AutoMisja;
	unsigned char SterAwaryjne;
	unsigned char SaabSter;
	int ZadSilaKabl;
	int ZadPredKabl;
	int ZadDlKabl;
	unsigned char End;
} TWKBPdoMo;

typedef struct {
	unsigned char Start;
	unsigned char Pojazd;
	unsigned char Ramka;
	unsigned short LiczRam;
	unsigned char PodcKadl;
	unsigned short NrSer;
	float Kurs;
	float Przeg;
	float Przechyl;
	float Glebokosc;
	float Echo;
	float NapTrans;
	short SkretyKabl;
	short KadlTemp;
	__attribute__ ((aligned(8))) double DlugGeogr;	
	__attribute__ ((aligned(8))) double SzerGeogr;	
	short PredkoscX;
	short PredkoscY;
	char PolObrKamGD;
	char PolObrKamLP;
	char PolObrSon;
	unsigned char ModulManipul;
	short ObrPedZad[8];
	short ObrPedMierz[8];
	float PradyPed[8];
	unsigned char Niespr[4];
	unsigned char DaneDodatkowe;
	unsigned char ModulBojowy;
	unsigned char NiesprModBoj;
	unsigned char PodcMBojowy;
	short PolWindy;
	short PolObrWytyku;
	float PredDzwiek;
	unsigned char Misja;
	unsigned char PCUStatus;
	int PCU_voltage[2];
	int PCU_current[2];
	int PCU_IsolationImp[2];
	unsigned char ATCStatus;
	int KablSila;
	int KablDl;
	int KablDlPoz;
	unsigned char TestOdp;
	unsigned short TrafoNap;
	unsigned short TrafoPrad;
	char TrafoTemp;
	unsigned char TrafoHamulec;
	unsigned char End;
} TModoSPP;


typedef struct {
	unsigned char Start;
	unsigned char Pojazd;
	unsigned char Ramka;
	unsigned short LiczRam;
	unsigned char PodcKadl;
	unsigned short NrSer;
	float Kurs;
	float Przeg;
	float Przechyl;
	float Glebokosc;
	float Echo;
	float NapTransp;
	short SkretyKabl;
	short KadlTemp;
	double DlugGeogr;	__attribute__ ((aligned(8)))
	double SzerGeogr;	__attribute__ ((aligned(8)))
	short PredkoscX;
	short PredkoscY;
	char PolObrKamGD;
	char PolObrKamLP;
	char PolObrSon;
	unsigned char ModulManipul;
	short ObrPedZad[8];
	short ObrPedMierz[8];
	float PradyPed[8];
	unsigned char Niespr[4];
	unsigned char DaneDodatkowe;
	unsigned char ModulBojowy;
	unsigned char NiesprModBoj;
	unsigned char PodcMBojowy;
	short PolWindy;
	short PolObrWytyku;
	float PredDzwiek;
	unsigned char Misja;
	unsigned char PCUStatus;
	int PCU_volatage[2];
	int PCU_current[2];
	int PCU_IsolationImp[2];
	unsigned char ATCStatus;
	int KablSila;
	int KablDl;
	int KablDlPoz;
	unsigned char TestOdp;
	unsigned short trafo_nap;	//1 oznacza 1 V
	unsigned short trafo_prad;		//1 odnacza 0,1 A
	char trafo_temp;
	unsigned char trafo_hamulec;

	//==============dane powtarzane z ramki steruj¹cej
    unsigned char Testy;
	unsigned char WlaczanieHydro;
	unsigned char Kamera1;
	unsigned char Kamera2;
	unsigned char StopienOsw[4];
	unsigned char ManipulZwalniak[2];
	unsigned char SterRoboczych;
	short Joysticki[6];
	float ZadKurs;
	float ZadPrzeg;
	float ZadGleb;
	float ZadEcho;
	float ZadPedPion;
	float ZadPredkosc;
	float ZadPozGleb;
	double ZadPozDl;		__attribute__ ((aligned (8)))
	double ZadPozSzer;		__attribute__ ((aligned (8)))
	unsigned char Precyzja;
	unsigned char JoystickTryb;
	unsigned char Regulatory;
	unsigned char AutoMisja;
	unsigned char SterAwaryjne;
	unsigned char SaabSter;
	int ZadSilaKabl;
	int ZadPredKabl;
	int ZadDlKabl;
	//==========================================

	unsigned char End;
} TModoWKBP;

typedef struct {
	unsigned char Ramka;	//0x23
	unsigned short LicznikRamek;
	unsigned short SterowaniePZ1;
	unsigned char SterowaniePZ2;
	unsigned char Lampy[4];
	unsigned char SterowaniePZ3;
	unsigned char KamKanal;
	unsigned char SterObrKam[3];
	short PolObrKam;
	short PredObrKam;
	unsigned char ParamKam[4];
	unsigned char PolarKam[2];
	unsigned char SterObrSon[3];
	short PolObrSon;
	short PredObrSon;
	unsigned char ParamSon[4];
	unsigned char PolarSon[2];
	unsigned char SterPrad;
	unsigned char LimitPrad[2];
	unsigned short WartGran;
} TWKBPdoPZ;

typedef struct {
	unsigned char Ramka;	//0x23
	unsigned short LicznikRamek;
	float PZ1Prad[6];
	float PZ1Nap[6];
	unsigned char TempLampy[4];
	unsigned short Cisnienie;
	float PZ2Prad;
	float PZ2Nap;
	float CzujZal;
	float NapAkuTrans;
	float ObrKamPrad;
	float ObrKamNap;
	float ObrSonPrad ;
	float ObrSonNap;
	float TempKadl;
	short PolObrKam;
	short PredObrKam;
	unsigned char StanKranKam;
	short ZakrObrKam[2];
	unsigned char ParamKam[4];
	unsigned char ParamKamE[4];
	unsigned char PolarKam[2];
	unsigned char PolarKamE[2];
	short PolObrSon;
	short PredObrSon;
	unsigned char StanKranSon;
	short ZakrObrSon[2];
	unsigned char ParamSon[4];
	unsigned char ParamSonE[4];
	unsigned char PolarSon[2];
	unsigned char PolarSonE[2];
	unsigned char LimitPrad[2];
	unsigned char LimitPradE[2];
	unsigned short WartGranKam;
	unsigned short WartGranKamE;
	unsigned short WartGranSon;
	unsigned short WartGranSonE;	
} TPZdoWKBP;

typedef struct {
	unsigned short napiecie;	//1 oznacza 1 V
	unsigned short prad;	//1 odnacza 0,1 A
	char temp;
	unsigned char hamulec;
	unsigned char bledy;
} TodTrafo;

//struktura dla kompasu MTi Xsens
typedef struct {
	float temp;
	float accX;
	float accY;
	float accZ;
	float gyrX;
	float gyrY;
	float gyrZ;
	float magX;
	float magY;
	float magZ;
	float pitch;
	float roll;
	float yaw;
	unsigned char status;
	unsigned short count;
} TMTi;

typedef struct {
	float yaw;	//kurs +-180
	float pitch;	//pochylenie
	float roll;	//przechy³
	float MagX;
	float MagY;
	float MagZ;
	float AccX;
	float AccY;
	float AccZ;
	float GyrX;
	float GyrY;
	float GyrZ;
} TVectNav;

typedef struct {
	//z 3 (IMU data)
	float Accel[3];		//[m/s^2]	
	float Gyro[3];		//[rad/s]
	float Temp;			//[st. C]
	//z 4 (Magnitude)
	float Mag[3];	//[a.u]
	//z 6 (Euler)
	float euler[3];			//[rad]
	float euler_acc[3];		//accuracy [rad]
	//z 9 (Ship Motion)
	float SSH[3];		//[m]
	float AccXYZ[3];	//[m/s^2]
	float VelXYZ[3];	//[m/s]
} TSBG;


typedef struct {
	//dane z ramki GPGGA
	float time;
	double latitude;
	char kier1;
	double longitude;
	char kier2;
	unsigned char quality;
	unsigned char sat_num;
	float HDOP;
	float altitude;
	float geoid_height;
	
	//dane z ramki PGRME
	float hor_pos;
	float vert_pos;
	float pos_err;
} TGPS;
	
	
typedef struct {
	unsigned char Ramka;	//0x24
	unsigned short LicznikRamek;
	unsigned char WlaczanieHydro;
	short marszowy[4];
	short manewrowy[4];
	short m_bojowy[4];
	unsigned char lampy[4];
	unsigned char obr_kam;
	unsigned char obr_son;
	unsigned char kamera1;
} TPGdoMo;

#ifdef MORSWIN1
typedef struct {
	unsigned short pred;
	unsigned short prad;
	unsigned char temp;
	unsigned char bledy;
} TodSilnik;
#endif

#ifdef MORSWIN2
typedef struct {
	unsigned char addr;
	unsigned char funkcja;
	unsigned char ver;
	unsigned char ster_bits;
	short pred;
	unsigned short nap;
	unsigned short prad;		//mA
	short temp_modul;
	short temp_siln;
	short cisnienie;
	unsigned char bledy;
} TodSilnik;
#endif

typedef struct {
	unsigned char Ramka;	//0x24
	unsigned short LicznikRamek;
	TodSilnik marszowy[4];
	TodSilnik manewrowy[4];
	TodSilnik m_bojowy[4];
	TodTrafo trafo;
	unsigned char podcisnienie;
	unsigned char temperatura;
	//TMTi ahrs;
	TVectNav ahrs;
	float glebokosc;
	float echo;
	unsigned char bledy[6];
	unsigned char alarmy;
} TModoPG;

typedef struct {
	float Kurs;
	float Przeg;
	float Gleb;
	float Echo;
	float PedPion;
	float Pred;
	float XNed;
	float YNed;
	double DlGeogr;		
	double SzerGeogr;
	float GlebGeogr;
	double LLD_wp[3];
	float NED_wp[3];
	unsigned char RegKurs : 1;
	unsigned char RegPrzeg : 1;
	unsigned char RegPrzech : 1;
	unsigned char RegGleb : 1;
	unsigned char RegEcho : 1;
	unsigned char RegPred : 1;
	unsigned char RegPoz : 1;
	unsigned char RegPedPion : 1;
	unsigned char GuidanceSTART : 1;
	unsigned char GuidancePAUSE : 1;
	unsigned char GuidanceUPDATE : 1;
	unsigned char GuidanceLATLONG : 1;
	unsigned char SterAwaryjne;  // PG NEW
	unsigned char strojenie;
} TwartZad;	
	
typedef struct {
	float yaw;		//kurs 0-360
	float pitch;	//pochylenie
	float roll;		//przechy³
	float GyrX;
	float GyrY;
	float GyrZ;	
	double latitude;	//GPS
	double longitude;	//GPS
	unsigned short znacznik_czasu;	//DVL
	short PredkoscX;	//DVL
	short PredkoscY;	//DVL
	float glebokosc;	//Keller
	float echo;			//Echosonda Tritech
} TPomiar;

typedef struct {
	unsigned char STX;
	unsigned char Nadawca;
	double OkrPozDl;
	double OkrPozSzer;
	double OkrKurs;
	double PojPozDl;
	double PojPozSzer;
	double PojPozDlObl;
	double PojPozSzerObl;
	unsigned char ETX;
} TSPPdoMarcin;

typedef struct {
	unsigned char STX;			
	unsigned char senderId;		//! Identyfikator nadawcy - "Monitor.exe"
	unsigned short count_ore;	//! Numer pomiaru z ORE (zwiêkszany po ka¿dym prawid³owym pomiarze)
	unsigned char correct; 		//! Bajt poprawno¶ci danych
	double distance;  			//! ORE odleg³o¶æ
	double bearing;  			//! ORE namiar
	double dep_angle; 			//! ORE kat nachylenia

	double rov_lambda;		//! pozycja pojazdu (d³ugo¶æ geograf.)
	double rov_fi; 			//! pozycja pojazdu (szerko¶æ geograf.)
	double rov_x; 			//! pozycja wzglêdna pojazdu X (Po uwzglêdnieniu orientacji okrêtu)
	double rov_y; 			//! pozycja wzglêdna pojazdu Y (Po uwzglêdnieniu orientacji okrêtu)
	double ship_lambda; 	//! pozycja okrêtu (d³ugo¶æ geograf.)
	double ship_fi; 		//! pozycja okrêtu (szeroko¶æ geograf.)
	double ship_course; 	//! kurs okrêtu
	unsigned char ETX;
} TMarcindoSPP;

typedef struct {
    unsigned char Ramka;   //tu mo¿na zaproponowaæ jak±¶ warto¶æ
    unsigned short LicznikRamek;
} TSerwisZapyt;


typedef struct {
    unsigned char Ramka;   //tu mo¿na zaproponowaæ jak±¶ warto¶æ
    unsigned short LicznikRamek;
   TodSilnik SilnikDane[8];
} TSerwisOdp; 

typedef struct {
	unsigned char Ramka;	//funkcja ramki
	unsigned short LiczRam;	
	float x;
	float y;
	double longitude;
	double latitude;
	float depth;
	float accuracy;
} THiPAP;

typedef struct {
	unsigned char Ramka;	//funkcja ramki 101
	unsigned short LiczRam;	
	float course;			
	float heading;			//skierowanie dziobu
	double longitude;		//[rad]
	double latitude;		//[rad]
	float speed;			//[m/s]
} TShipData;

typedef struct {
	unsigned char Ramka;	//funkcja ramki
	unsigned short LiczRam;	
	double longitude;
	double latitude;
	float depth;
	unsigned char cell_count;
	float depth_cell[255];
	float direction[255];
	float speed[255];
} TWaterProfile;

typedef struct {
	THiPAP HiPAP;
	TShipData ShipData;
	TWaterProfile WaterProfile;
} TNaviData;
 
 
 
 typedef struct {   // informacja o dostepnosci sensorow i aktorow na potrzeby automatyki
	// bledy
	// awarie pednikow lub sterownikow
	unsigned char  LD_err:1;   
	unsigned char  PD_err:1;   
	unsigned char  LR_err:1;   
	unsigned char  PR_err:1;
	unsigned char  PLD_err:1;
	unsigned char  PPD_err:1;
	unsigned char  PLR_err:1;
	unsigned char  PPR_err:1;
	
	// wylaczenia pednikow z WKBP
	unsigned char  LD_off:1;   
	unsigned char  PD_off:1;   
	unsigned char  LR_off:1;   
	unsigned char  PR_off:1;
	unsigned char  PLD_off:1;
	unsigned char  PPD_off:1;
	unsigned char  PLR_off:1;
	unsigned char  PPR_off:1;
	
	// sensory wylaczenie/blad/brak_komunikacji
	unsigned char  DVL_off :1;
	unsigned char  echo_off:1;
	unsigned char  AHRS_err:1;
	unsigned char  keller_err:1;	
} TMSdostepnosc;


void timer_18ms(void);
void *automatyka_func(void *cookie);
void *glowny_func(void *cookie);
void *pedniki_func(void *cookie);
