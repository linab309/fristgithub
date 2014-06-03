#ifndef _DEBUG_H_38D1DB43_50DF_4682_817A_07D64146E600
#define _DEBUG_H_38D1DB43_50DF_4682_817A_07D64146E600

#include "hal_types.h"
#include "hal_assert.h"
#include "hal_board.h"
#include "hal_defs.h"
#include "hal_mcu.h"
#include "hal_uart.h"
#if defined MT_TASK
#include "MT_UART.h"
#endif


//#define DEBUG_CH

#if defined(ZDO_COORDINATOR)
	#define DEBUG_PORT		1	// 0 or 1
#else
	#define DEBUG_PORT		0	// 0 or 1
#endif

#if DEBUG_PORT == 0
	#define CTRL_PORT	1
#elif DEBUG_PORT == 1
	#define CTRL_PORT	0
#else
	#error DEBUG_PORT error
#endif

#ifdef DEBUG_CH
#define dbgopen(x)		UartOpen(x)
#define dbgsend(x)	UartSend x

#define dbghex(x)	ChDebugHex x
#define dbgint(x)	ChDebugSigned x
#define dbguint(x)	ChDebugUnsigned x
#define dbgstr(x)	ChDebugStr x
#define dbgtag(x)	ChDebugTag x
#else
////莫要分成多行
#define dbgopen(x)	
#define dbgsend(x)	

#define dbghex(x)	
#define dbgint(x)	
#define dbguint(x)	
#define dbgstr(x)	
#define dbgtag(x)	

#endif

#ifdef DEBUG_CH
void UartOpen(halUARTCfg_t *config);
uint16 UartSend(uint8 *buf, uint16 len);
uint16 UartSend2(uint8 *buf, uint16 len);
#endif

#if 1
char ChDebugHex(const char *format,uint32 hex);
char ChDebugSigned(const char *format,int32 decimal);
char ChDebugUnsigned(const char *format,uint32 decimal);
char ChDebugStr(const char *format,char* str);
char ChDebugTag(const char *format);

#endif

#endif
