#ifndef DS18B20_H
#define DS18B20_H

#include "ioCC2530.h"

/******************************************************************************
*******************              Commonly used types        *******************
******************************************************************************/
typedef unsigned char       BOOL;

// Data
typedef unsigned char       BYTE;
typedef unsigned short      WORD;
typedef unsigned long       DWORD;

// Unsigned numbers
typedef unsigned char       UINT8;
typedef unsigned short      UINT16;
typedef unsigned long       UINT32;

// Signed numbers
typedef signed char         INT8;
typedef signed short        INT16;
typedef signed long         INT32;

#define IN_DQ  P1_1
#define CL_DQ  IN_DQ=0;asm("NOP");asm("NOP")
#define SET_DQ IN_DQ=1;asm("NOP");asm("NOP") 
#define SET_OUT P1DIR|=0x02;asm("NOP");asm("NOP")
#define SET_IN  P1DIR&=~0x02;asm("NOP");asm("NOP")

extern unsigned char id[8];
extern unsigned char sensor_data_value[2];
extern UINT8 ch[9];

void Delay_nus(UINT16 n) ;
void write_1820(unsigned char x) ; 
unsigned char read_1820(void);  
void init_1820(void) ; 
void read_data(UINT8 * pSensorValue);
void get_id(void);
void ds18b20_main(void);
void DataChange(UINT8 * pSensorValue,UINT8 * pChBuf);
#endif

