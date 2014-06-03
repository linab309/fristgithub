
#include "chdebug.h"

#ifdef DEBUG_CH
#define CSR_MODE                   0x80
#define CSR_RE                     0x40
#define CSR_SLAVE                  0x20
#define CSR_FE                     0x10
#define CSR_ERR                    0x08
#define CSR_RX_BYTE                0x04
#define CSR_TX_BYTE                0x02
#define CSR_ACTIVE                 0x01

// UxUCR - USART UART Control Register.
#define UCR_FLUSH                  0x80
#define UCR_FLOW                   0x40
#define UCR_D9                     0x20
#define UCR_BIT9                   0x10
#define UCR_PARITY                 0x08
#define UCR_SPB                    0x04
#define UCR_STOP                   0x02
#define UCR_START                  0x01

#define UTX0IE                     0x04
#define UTX1IE                     0x08

#define P2DIR_PRIPO                0xC0
#if (DEBUG_PORT == 0)
#define PxOUT                      P0
#define PxDIR                      P0DIR
#define PxSEL                      P0SEL
#define UxCSR                      U0CSR
#define UxUCR                      U0UCR
#define UxDBUF                     U0DBUF
#define UxBAUD                     U0BAUD
#define UxGCR                      U0GCR
#define URXxIE                     URX0IE
#define URXxIF                     URX0IF
#define UTXxIE                     UTX0IE
#define UTXxIF                     UTX0IF
#else
#define PxOUT                      P1
#define PxDIR                      P1DIR
#define PxSEL                      P1SEL
#define UxCSR                      U1CSR
#define UxUCR                      U1UCR
#define UxDBUF                     U1DBUF
#define UxBAUD                     U1BAUD
#define UxGCR                      U1GCR
#define URXxIE                     URX1IE
#define URXxIF                     URX1IF
#define UTXxIE                     UTX1IE
#define UTXxIF                     UTX1IF
#endif

#if (DEBUG_PORT == 0)
#define HAL_UART_PERCFG_BIT        0x01         // USART0 on P0, Alt-1; so clear this bit.
#define HAL_UART_Px_RX_TX          0x0C         // Peripheral I/O Select for Rx/Tx.
#define HAL_UART_Px_RTS            0x20         // Peripheral I/O Select for RTS.
#define HAL_UART_Px_CTS            0x10         // Peripheral I/O Select for CTS.
#define HAL_UART_PRIPO             0x00
#else
#define HAL_UART_PERCFG_BIT        0x02         // USART1 on P1, Alt-2; so set this bit.
#define HAL_UART_Px_RTS            0x20         // Peripheral I/O Select for RTS.
#define HAL_UART_Px_CTS            0x10         // Peripheral I/O Select for CTS.
#define HAL_UART_Px_RX_TX          0xc0         // Peripheral I/O Select for Rx/Tx.
#define HAL_UART_PRIPO             0x40
#endif


static void UartInit(void)
{
	// Set P2 priority - USART0 over USART1 if both are defined.
	P2DIR &= ~P2DIR_PRIPO;
	P2DIR |= HAL_UART_PRIPO;

#if (DEBUG_PORT == 0)
	PERCFG &= ~HAL_UART_PERCFG_BIT;    // Set UART0 I/O location to P0.
#else
	PERCFG |= HAL_UART_PERCFG_BIT;     // Set UART1 I/O location to P1.
//	PERCFG &= ~HAL_UART_PERCFG_BIT;    // Set UART1 I/O location to P0.
#endif
//	PxSEL &=~0x3c;
	PxSEL  |= HAL_UART_Px_RX_TX;       // Enable Tx and Rx on P1.
	ADCCFG &= ~HAL_UART_Px_RX_TX;      // Make sure ADC doesnt use this.
	UxCSR = CSR_MODE;                  // Mode is UART Mode.
	UxUCR = UCR_FLUSH;                 // Flush it.
	UTXxIF = 0;
	//UxGCR |= 11;				       
}
void UartOpen(halUARTCfg_t *config)
{
	UartInit();
	//isrCfg.uartCB = config->callBackFunc;
	// Only supporting subset of baudrate for code size - other is possible.
	//HAL_UART_ASSERT((config->baudRate == HAL_UART_BR_9600) ||
	//                (config->baudRate == HAL_UART_BR_19200) ||
	//                (config->baudRate == HAL_UART_BR_38400) ||
	//                (config->baudRate == HAL_UART_BR_57600) ||
	//                (config->baudRate == HAL_UART_BR_115200));
	//
	if (config->baudRate == HAL_UART_BR_57600 ||
			config->baudRate == HAL_UART_BR_115200)
	{
		UxBAUD = 216;
	}
	else
	{
		UxBAUD = 59;
	}
	
	switch (config->baudRate)
	{
		case HAL_UART_BR_9600:
			UxGCR = 8;
			break;
		case HAL_UART_BR_19200:
			UxGCR = 9;
			break;
		case HAL_UART_BR_38400:
		case HAL_UART_BR_57600:
			UxGCR = 10;
			break;
		default:
			UxGCR = 11;
			break;
	}

	// 8 bits/char; no parity; 1 stop bit; stop bit hi.
	if (config->flowControl)
	{
		UxUCR = UCR_FLOW | UCR_STOP;
		PxSEL |= HAL_UART_Px_RTS | HAL_UART_Px_CTS;
	}
	else
	{
		UxUCR |= UCR_STOP;
	}

	UxCSR |= CSR_RE;
	URXxIE = 1;
	//UTXxIF = 1;  // Prime the ISR pump.
}
uint16 UartSend(uint8 *buf, uint16 len)
{
	uint16 cnt;
	//uint8 res = 0;

	//whhile(UxCSR&0x01 != 0);
	for (cnt = 0; cnt < len; cnt++)
	{
		UxDBUF = *buf++;
		//res = *buf;
		while(UTXxIF == 0);
		UTXxIF = 0;		
	}


	return cnt;
}
uint16 UartSend2(uint8 *buf, uint16 len)
{
	HalUARTWrite(DEBUG_PORT,buf,len);
}

#endif
#if 1
XDATA static char strTemp_gen[70] = {0};//用于调试输出
char ChDebugHex(const char *format,uint32 hex)
{
	int16 i;
	int16 indexDest = 0;
	char err=0;
	int16 len = strlen(format);
	//strcpy(strTemp_gen,format);
	for(i=0;i<len;i++)
		if(format[i] == '%')
		{
			if(format[i+1] == 'x')// %x
			{
				strTemp_gen[indexDest++] = '0';
				strTemp_gen[indexDest++] = 'x';
				{
					int k;
					char isNotZeroStart=0;
					for(k=7;k>=0;k--)
					{
						unsigned char c = ((hex>>(4*k))&0x0f);
						if(c == 0 && ! isNotZeroStart && k != 0)
							continue;
						isNotZeroStart = 1;
						if(c<10)
							strTemp_gen[indexDest++] = c + '0';
						else
							strTemp_gen[indexDest++] = c + 'A' - 10;
					}
				}
				i+=2-1;
			}
			/*
			else if((format[i+1] >= '1' && format[i+1] <= '9') 
				&& format[i+2] == 'x') // %4x
			{
				{
					char num = format[i+1] - '0';
					int k;
					strTemp_gen[indexDest++] = '0';
					strTemp_gen[indexDest++] = 'x';
					for(k=num-1;k>=0;k--)
					{
						char c = ((hex>>(4*k))&0x0f);

						if(c == 0)
							strTemp_gen[indexDest++] = ' ';
						else if(c<10)
							strTemp_gen[indexDest++] = c + '0';
						else
							strTemp_gen[indexDest++] = c + 'A' - 10;
					}
				}
				i+=3-1;
				
			}
			*/
			else if(format[i+1] == '0'
				&&(format[i+2] >= '1' && format[i+2] <= '9') 
				&& format[i+3] == 'x') // %04x
			{
				{
					char num = format[i+2] - '0';
					int k;
					strTemp_gen[indexDest++] = '0';
					strTemp_gen[indexDest++] = 'x';
					for(k=num-1;k>=0;k--)
					{
						char c = ((hex>>(4*k))&0x0f);

						if(c<10)
							strTemp_gen[indexDest++] = c + '0';
						else
							strTemp_gen[indexDest++] = c + 'A' - 10;
					}
				}
				i+=4-1;
				
			}
			else
				strTemp_gen[indexDest++] = format[i];

		}
		else
			strTemp_gen[indexDest++] = format[i];
	//strTemp_gen[indexDest++] = 0;
	UartSend(strTemp_gen,indexDest);
	return err;
}
char ChDebugSigned(const char *format,int32 decimal)//有符号十进制
{
	int16 i;
	int16 indexDest = 0;
	char err=0;
	int16 len = strlen(format);
	//strcpy(strTemp_gen,format);
	for(i=0;i<len;i++)
		if(format[i] == '%')
		{
			if(format[i+1] == 'd')// %d
			{
				{
					int16 l = i;
					int16 r;
					long value = decimal;
					char c;
					if(value < 0)
					{
						strTemp_gen[indexDest++] = '-';
						value = -value;
						l++;
					}
					do
					{
						c = value%10;
						strTemp_gen[indexDest++] = c + '0';
						value /= 10;
					}while(value != 0);
					////swap the string
					r = indexDest-1;
					while(l < r)
					{
						c = strTemp_gen[l];
						strTemp_gen[l] = strTemp_gen[r];
						strTemp_gen[r] = c;
						l++;
						r--;
					}
					
				}
			i+=2-1;
			}
			else
				strTemp_gen[indexDest++] = format[i];
		}
		else
			strTemp_gen[indexDest++] = format[i];
	//strTemp_gen[indexDest++] = 0;
	UartSend(strTemp_gen,indexDest);
	return err;
}
char ChDebugUnsigned(const char *format,uint32 decimal)//无符号十进制
{
	int16 i;
	int16 indexDest = 0;
	char err=0;
	int16 len = strlen(format);
	//strcpy(strTemp_gen,format);
	for(i=0;i<len;i++)
		if(format[i] == '%')
		{
			if(format[i+1] == 'd')// %d
			{
				{
					int l = i;
					int r;
					long value = decimal;
					char c;
					if(value < 0)
					{
						strTemp_gen[indexDest++] = '-';
						value = -value;
						l++;
					}
					do
					{
						c = value%10;
						strTemp_gen[indexDest++] = c + '0';
						value /= 10;
					}while(value != 0);
					////swap the string
					r = indexDest-1;
					while(l < r)
					{
						c = strTemp_gen[l];
						strTemp_gen[l] = strTemp_gen[r];
						strTemp_gen[r] = c;
						l++;
						r--;
					}
					
				}
				i+=2-1;
			}
			else
				strTemp_gen[indexDest++] = format[i];
			

		}
		else
			strTemp_gen[indexDest++] = format[i];
	//strTemp_gen[indexDest++] = 0;
	UartSend(strTemp_gen,indexDest);
	return err;
}
char ChDebugStr(const char *format,char* str)
{
	int16 i;
	int16 indexDest = 0;
	char err=0;
	int16 len = strlen(format);
	//strcpy(strTemp_gen,format);
	for(i=0;i<len;i++)
		if(format[i] == '%')
		{
			if(format[i+1] == 's')// %s
			{
				strTemp_gen[indexDest] = '\0';
				strcpy(&strTemp_gen[indexDest],str);
				indexDest += strlen(str);
				i+=2-1;
			}
			else
				strTemp_gen[indexDest++] = format[i];
		}
		else
			strTemp_gen[indexDest++] = format[i];
	//strTemp_gen[indexDest++] = 0;
	UartSend(strTemp_gen,indexDest);
	return err;
}
char ChDebugTag(const char *format)
{
	char err=0;
	int16 len = strlen(format);
	//strcpy(strTemp_gen,format);
	//strTemp_gen[indexDest++] = 0;
	UartSend((unsigned char*)format,len);
	return err;
}

#endif
