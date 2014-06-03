/******************************************************************************
	Filename:       GenericApp.c
	Revised:        $Date: 2012-03-07 01:04:58 -0800 (Wed, 07 Mar 2012) $
	Revision:       $Revision: 29656 $

	Description:    Generic Application (no Profile).


	Copyright 2004-2012 Texas Instruments Incorporated. All rights reserved.

	IMPORTANT: Your use of this Software is limited to those specific rights
	granted under the terms of a software license agreement between the user
	who downloaded the software, his/her employer (which must be your employer)
	and Texas Instruments Incorporated (the "License"). You may not use this
	Software unless you agree to abide by the terms of the License. The License
	limits your use, and you acknowledge, that the Software may not be modified,
	copied or distributed unless embedded on a Texas Instruments microcontroller
	or used solely and exclusively in conjunction with a Texas Instruments radio
	frequency transceiver, which is integrated into your product. Other than for
	the foregoing purpose, you may not use, reproduce, copy, prepare derivative
	works of, modify, distribute, perform, display or sell this Software and/or
	its documentation for any purpose.

	YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
	PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
	INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
	NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
	TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
	NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
	LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
	INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
	OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
	OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
	(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

	Should you have any questions regarding your right to use this Software,
	contact Texas Instruments Incorporated at www.TI.com.
******************************************************************************/

/*********************************************************************
	This application isn't intended to do anything useful, it is
	intended to be a simple example of an application's structure.

	This application sends "Hello World" to another "Generic"
	application every 5 seconds.  The application will also
	receives "Hello World" packets.

	The "Hello World" messages are sent/received as MSG type message.

	This applications doesn't have a profile, so it handles everything
	directly - itself.

	Key control:
		SW1:
		SW2:  initiates end device binding
		SW3:
		SW4:  initiates a match description request
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "osal_nv.h"

#include "GenericApp.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
	#include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* RTOS */
#if defined( IAR_ARMCM3_LM )
#include "RTOS_App.h"
#endif  
#include "chdebug.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// This list should be filled with Application specific Cluster IDs.
const cId_t GenericApp_ClusterList[GENERICAPP_MAX_CLUSTERS] =
{
	GENERICAPP_CLUSTERID
};

const SimpleDescriptionFormat_t GenericApp_SimpleDesc =
{
	GENERICAPP_ENDPOINT,              //  int Endpoint;
	GENERICAPP_PROFID,                //  uint16 AppProfId[2];
	GENERICAPP_DEVICEID,              //  uint16 AppDeviceId[2];
	GENERICAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
	GENERICAPP_FLAGS,                 //  int   AppFlags:4;
	GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
	(cId_t *)GenericApp_ClusterList,  //  byte *pAppInClusterList;
	GENERICAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
	(cId_t *)GenericApp_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in GenericApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t GenericApp_epDesc;


void UartControlSend(uint8*buf,uint8 len);
/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */
/*********************************************************************
 * TYPEDEFS
 */
#if !defined( SERIAL_APP_PORT )
#define SERIAL_APP_PORT  0
#endif

#if !defined( SERIAL_APP_BAUD )
//#define SERIAL_APP_BAUD  HAL_UART_BR_38400
#define SERIAL_APP_BAUD  HAL_UART_BR_115200
#endif

// When the Rx buf space is less than this threshold, invoke the Rx callback.
#if !defined( SERIAL_APP_THRESH )
#define SERIAL_APP_THRESH  64
#endif

#if !defined( SERIAL_APP_RX_SZ )
#define SERIAL_APP_RX_SZ  128
#endif

#if !defined( SERIAL_APP_TX_SZ )
#define SERIAL_APP_TX_SZ  128
#endif

// Millisecs of idle time after a byte is received before invoking Rx callback.
#if !defined( SERIAL_APP_IDLE )
#define SERIAL_APP_IDLE  6
#endif

// Loopback Rx bytes to Tx for throughput testing.
#if !defined( SERIAL_APP_LOOPBACK )
#define SERIAL_APP_LOOPBACK  FALSE
#endif

// This is the max byte count per OTA message.
#if !defined( SERIAL_APP_TX_MAX )
#define SERIAL_APP_TX_MAX  256
#endif


/*********************************************************************
 * GLOBAL VARIABLES
 */

XDATA static uint8 SerialApp_Buf[SERIAL_APP_TX_MAX];
XDATA static uint8 SerialApp_Len = 0;

XDATA static uint8 bufBig_gen[256] = {0};//大的缓存，用于读写flash等
#if defined(ZDO_COORDINATOR)




#endif
#if ! defined(RTR_NWK)
static uint16 GroupId_gen = 0;
static uint8 ledBrightness_gen = 0xff;
static uint8 ledYellow_gen = 0xff;
static uint8 ledColorR_gen = 0xff;
static uint8 ledColorG_gen = 0xff;
static uint8 ledColorB_gen = 0xff;
#endif


#define GENERICAPP_CMDID_REPORT_SHORTADDR			0x0001
#define GENERICAPP_CMDID_ZD0_RESTART					0x0002
#define GENERICAPP_CMDID_SYNC_STATUS					0x0007
#define GENERICAPP_CMDID_LIGHT			0x0011
#define GENERICAPP_CMDID_SET_DEVICE_NAME	0x0021
#define GENERICAPP_CMDID_READ_DEVICE_NAME	0x0022
#define GENERICAPP_CMDID_REPORT_DEVICE_NAME	0x0023
#define GENERICAPP_CMDID_ADD_TO_GROUP		0x0031
#define GENERICAPP_CMDID_REMOVE_FROM_GROUP	0x0032
#define GENERICAPP_CMDID_READ_GROUP_ID		0x0033
#define GENERICAPP_CMDID_REPORT_GROUP_ID		0x0034
////因为zigbee自带的组播消息在快速反复操作时，会有消息丢失的现象，故改用广播消息中判断
#define GENERICAPP_CMDID_BROADCAST_GROUP_CTRL		0x0041		
/*********************************************************************
 * LOCAL VARIABLES
 */
byte GenericApp_TaskID;   // Task ID for internal task/event processing
													// This variable will be received when
													// GenericApp_Init() is called.
devStates_t GenericApp_NwkState;


byte GenericApp_TransID;  // This is the unique message ID (counter)

afAddrType_t GenericApp_DstAddr;

#define ZIGBEE_PROTOCOL_DATA_START			2

#define UART_PROTOCOL_SEQ_START			2
#define UART_PROTOCOL_ACK_START			6
#define UART_PROTOCOL_CMDID_START			7
#define UART_PROTOCOL_DATA_START				9

extern XDATA uint32 ramdomSeed;
XDATA uint8 syncStatusCnt = 0;
XDATA uint8 syncStatusOk = 0;
/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
static void GenericApp_HandleKeys( byte shift, byte keys );
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
static void GenericApp_SendMessage(afAddrMode_t addrMode,uint16 addr ,uint8* buf,uint16 buflen );
static void GenericApp_SendMessageCmd(afAddrMode_t addrMode,uint16 addr ,uint16 cmd,uint8* buf,uint16 buflen );

#if defined( IAR_ARMCM3_LM )
static void GenericApp_ProcessRtosMessage( void );
#endif
static void SerialApp_CallBack(uint8 port, uint8 event);
void PrintDeviceList_gen(void);
void initTimer();
#if ! defined(RTR_NWK)
void ProcessMsgLight(uint8* data);

#endif
void ReportStatus();
void UartControlSendCmd(uint16 retCmdId,uint8*buf,uint8 dataLen);
uint8 CalcChecksum(uint8*buf,uint8 len);
void UartControlSend(uint8*buf,uint8 dataLen);
void UartControlSendAck(uint8*buf);

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */



/*********************************************************************
 * @fn      GenericApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void GenericApp_Init( uint8 task_id )
{
	halUARTCfg_t uartConfig;

#if 0
	uartConfig.configured = TRUE;
	uartConfig.baudRate = HAL_UART_BR_115200;
	uartConfig.flowControl          = FALSE;
	uartConfig.flowControlThreshold = SERIAL_APP_THRESH;
	uartConfig.rx.maxBufSize        = SERIAL_APP_RX_SZ;
	uartConfig.tx.maxBufSize        = SERIAL_APP_TX_SZ;
	uartConfig.idleTimeout          = SERIAL_APP_IDLE;
	uartConfig.intEnable            = TRUE;
	uartConfig.callBackFunc         = SerialApp_CallBack;
	HalUARTOpen(DEBUG_PORT,&uartConfig);

	dbgtag(("hello world\r\n"));

#endif
#if 1
	uartConfig.configured = TRUE;
	uartConfig.baudRate = HAL_UART_BR_115200;
	uartConfig.flowControl          = FALSE;
	uartConfig.flowControlThreshold = SERIAL_APP_THRESH;
	uartConfig.rx.maxBufSize        = SERIAL_APP_RX_SZ;
	uartConfig.tx.maxBufSize        = SERIAL_APP_TX_SZ;
	uartConfig.idleTimeout          = SERIAL_APP_IDLE;
	uartConfig.intEnable            = TRUE;
	uartConfig.callBackFunc         = NULL;//SerialApp_CallBack;
	dbgopen(&uartConfig);

	dbgtag(("hello world\r\n"));
	//dbgint(("short=%d\r\n",sizeof(short)));
	//dbgint(("int=%d\r\n",sizeof(int)));
	//dbgint(("long=%d\r\n",sizeof(long)));

	uartConfig.configured = TRUE;
	uartConfig.baudRate = HAL_UART_BR_115200;
	uartConfig.flowControl          = FALSE;
	uartConfig.flowControlThreshold = SERIAL_APP_THRESH;
	uartConfig.rx.maxBufSize        = SERIAL_APP_RX_SZ;
	uartConfig.tx.maxBufSize        = SERIAL_APP_TX_SZ;
	uartConfig.idleTimeout          = SERIAL_APP_IDLE;
	uartConfig.intEnable            = TRUE;
	uartConfig.callBackFunc         = SerialApp_CallBack;
	HalUARTOpen(CTRL_PORT,&uartConfig);
#endif
	#if defined(ZDO_COORDINATOR)
	
	#endif

	GenericApp_TaskID = task_id;
	GenericApp_NwkState = DEV_INIT;
	GenericApp_TransID = 0;

	// Device hardware initialization can be added here or in main() (Zmain.c).
	// If the hardware is application specific - add it here.
	// If the hardware is other parts of the device add it in main().

#if defined(ZDO_COORDINATOR)
	GenericApp_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
	GenericApp_DstAddr.endPoint = 0;
	GenericApp_DstAddr.addr.shortAddr = 0;
#else
	GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
	GenericApp_DstAddr.addr.shortAddr = 0;
	// Take the first endpoint, Can be changed to search through endpoints
	GenericApp_DstAddr.endPoint = GENERICAPP_ENDPOINT;


#endif

	// Fill out the endpoint description.
	GenericApp_epDesc.endPoint = GENERICAPP_ENDPOINT;
	GenericApp_epDesc.task_id = &GenericApp_TaskID;
	GenericApp_epDesc.simpleDesc
						= (SimpleDescriptionFormat_t *)&GenericApp_SimpleDesc;
	GenericApp_epDesc.latencyReq = noLatencyReqs;


	#if defined(ZDO_COORDINATOR)
	//若条目不存在，初始化条目的内容为0
	//memset(bufBig_gen,0,sizeof(bufBig_gen));
	UartControlSendCmd(0x9001,bufBig_gen,0);

	#elif ! defined(RTR_NWK)
	initTimer();
	//若条目存在，则读出
	#endif
	// Register the endpoint description with the AF
	afRegister( &GenericApp_epDesc );

	// Register for all key events - This app will handle all key events
	RegisterForKeys( GenericApp_TaskID );

	// Update the display
#if defined ( LCD_SUPPORTED )
	//HalLcdWriteString( "GenericApp", HAL_LCD_LINE_1 );
#endif

	//ZDO_RegisterForZDOMsg( GenericApp_TaskID, End_Device_Bind_rsp );
	//ZDO_RegisterForZDOMsg( GenericApp_TaskID, Match_Desc_rsp );

#if defined( IAR_ARMCM3_LM )
	// Register this task with RTOS task initiator
	RTOS_RegisterApp( task_id, GENERICAPP_RTOS_MSG_EVT );
#endif

}

/*********************************************************************
 * @fn      GenericApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 GenericApp_ProcessEvent( uint8 task_id, uint16 events )
{
	afIncomingMSGPacket_t *MSGpkt;
	afDataConfirm_t *afDataConfirm;

	// Data Confirmation message fields
	byte sentEP;
	ZStatus_t sentStatus;
	byte sentTransID;       // This should match the value sent
	(void)task_id;  // Intentionally unreferenced parameter

	if ( events & SYS_EVENT_MSG )
	{
		MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
		while ( MSGpkt )
		{
			dbghex(("event=%x,",MSGpkt->hdr.event));
			dbghex(("addr=%x\r\n",NLME_GetShortAddr()));
		 	switch ( MSGpkt->hdr.event )
			{
				case ZDO_CB_MSG:
					GenericApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
					break;

				case KEY_CHANGE:
					GenericApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
					break;

				case AF_DATA_CONFIRM_CMD:
					// This message is received as a confirmation of a data packet sent.
					// The status is of ZStatus_t type [defined in ZComDef.h]
					// The message fields are defined in AF.h
					afDataConfirm = (afDataConfirm_t *)MSGpkt;
					sentEP = afDataConfirm->endpoint;
					sentStatus = afDataConfirm->hdr.status;
					sentTransID = afDataConfirm->transID;
					(void)sentEP;
					(void)sentTransID;

					// Action taken when confirmation is received.
					if ( sentStatus != ZSuccess )
					{
						// The data wasn't delivered -- Do something
					}
					break;

				case AF_INCOMING_MSG_CMD:
					GenericApp_MessageMSGCB( MSGpkt );
					break;

				case ZDO_STATE_CHANGE:
					GenericApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
					dbghex(("ZDO_STATE_CHANGE=%x,",GenericApp_NwkState));
					dbghex(("addr=%x\r\n",NLME_GetShortAddr()));
					#if defined(ZDO_COORDINATOR)
						if ( (GenericApp_NwkState == DEV_ZB_COORD))
						{
							GenericApp_SendMessageCmd(afAddrBroadcast, 0xffff,GENERICAPP_CMDID_ZD0_RESTART, bufBig_gen, 0);							
						}
					#elif ! defined(RTR_NWK)
						if ( GenericApp_NwkState == DEV_END_DEVICE )
						{
							syncStatusCnt = 0;
							srand(ramdomSeed);
							osal_start_timerEx( GenericApp_TaskID,EVT_SyncStatus_Generic,rand()%300 );
						}
					#endif

					break;

				default:
					break;
			}

			// Release the memory
			osal_msg_deallocate( (uint8 *)MSGpkt );

			// Next
			MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( GenericApp_TaskID );
		}

		// return unprocessed events
		return (events ^ SYS_EVENT_MSG);
	}

	// Send a message out - This event is generated by a timer
	//  (setup in GenericApp_Init()).
	if ( events & EVT_SyncStatus_Generic )
	{
		// Send "the" message
		//GenericApp_SendTheMessage();

		// Setup to send message again

		// if coordinator restart
		#if ! defined(RTR_NWK)
		if( ! syncStatusOk)
		{
			ReportStatus();
			syncStatusCnt++;
			//这个可归纳为没应答则超时重发
			
			if(syncStatusCnt < 5)
			{
				uint16 delay;
				srand(ramdomSeed);
				delay = rand()%500 + 500;
				osal_start_timerEx( GenericApp_TaskID,EVT_SyncStatus_Generic,delay );
				dbgint(("clock=%d\r\n",osal_getClock()));
			}
			
		}
		#endif
		// return unprocessed events
		return (events ^ EVT_SyncStatus_Generic);
	}

	
#if defined( IAR_ARMCM3_LM )
	// Receive a message from the RTOS queue
	if ( events & GENERICAPP_RTOS_MSG_EVT )
	{
		// Process message from RTOS queue
		GenericApp_ProcessRtosMessage();

		// return unprocessed events
		return (events ^ GENERICAPP_RTOS_MSG_EVT);
	}
#endif

	// Discard unknown events
	return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      GenericApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
	switch ( inMsg->clusterID )
	{
		case End_Device_Bind_rsp:
			if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
			{
				// Light LED
				HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
			}
#if defined( BLINK_LEDS )
			else
			{
				// Flash LED to show failure
				HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
			}
#endif
			break;

		case Match_Desc_rsp:
			{
				ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
				if ( pRsp )
				{
					if ( pRsp->status == ZSuccess && pRsp->cnt )
					{
						GenericApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
						GenericApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
						// Take the first endpoint, Can be changed to search through endpoints
						GenericApp_DstAddr.endPoint = pRsp->epList[0];

						dbgint(("endPoint=%d\r\n",GenericApp_DstAddr.endPoint));

						// Light LED
						HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
					}
					osal_mem_free( pRsp );
				}
			}
			break;
	}
}

/*********************************************************************
 * @fn      GenericApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void GenericApp_HandleKeys( uint8 shift, uint8 keys )
{
	zAddrType_t dstAddr;

	// Shift is used to make each button/switch dual purpose.
	if ( 0 )
	{
		if ( keys & HAL_KEY_SW_1 )
		{
		}
		if ( keys & HAL_KEY_SW_2 )
		{
		}
		if ( keys & HAL_KEY_SW_3 )
		{
		}
		if ( keys & HAL_KEY_SW_4 )
		{
		}
	}
	else
	{
		if ( keys & HAL_KEY_SW_1 )
		{
			// Since SW1 isn't used for anything else in this application...
#if defined( SWITCH1_BIND )
			// we can use SW1 to simulate SW2 for devices that only have one switch,
			keys |= HAL_KEY_SW_2;
#elif defined( SWITCH1_MATCH )
			// or use SW1 to simulate SW4 for devices that only have one switch
			keys |= HAL_KEY_SW_4;
#endif
		}

		if ( keys & HAL_KEY_SW_2 )
		{
			HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

			// Initiate an End Device Bind Request for the mandatory endpoint
			dstAddr.addrMode = Addr16Bit;
			dstAddr.addr.shortAddr = 0x0000; // Coordinator
			ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(),
														GenericApp_epDesc.endPoint,
														GENERICAPP_PROFID,
														GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
														GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
														FALSE );
		}

		if ( keys & HAL_KEY_SW_3 )
		{
		}

		if ( keys & HAL_KEY_SW_4 )
		{
			HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
			// Initiate a Match Description Request (Service Discovery)
			dstAddr.addrMode = AddrBroadcast;
			dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
			ZDP_MatchDescReq( &dstAddr, NWK_BROADCAST_SHORTADDR,
												GENERICAPP_PROFID,
												GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
												GENERICAPP_MAX_CLUSTERS, (cId_t *)GenericApp_ClusterList,
												FALSE );
		}
	}
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      GenericApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
#if 1
static void GenericApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
	int16 i;
	uint16 cmdId;
#if 1
	dbgint(("Message Length=%d\r\n",pkt->cmd.DataLength));
        
	for(i = 0;i<pkt->cmd.DataLength;i++)
	{
		dbghex((" %02x",pkt->cmd.Data[i]));
		if(i%8 == 7)
		{
			dbgtag(("\r\n"));
		}
	}
	dbgtag(("\r\n"));
#endif
	switch ( pkt->clusterId )
	{
		case GENERICAPP_CLUSTERID:
			cmdId = (pkt->cmd.Data[0])|(pkt->cmd.Data[1]<<8);
			switch(cmdId)
			{
			
			case GENERICAPP_CMDID_REPORT_SHORTADDR:
				#if defined(ZDO_COORDINATOR)
				{
					for(i = 2;i<pkt->cmd.DataLength;i++)
						bufBig_gen[UART_PROTOCOL_DATA_START + i - 2] = pkt->cmd.Data[i];

					#if 0
					dbgint(("Message Length=%d\r\n",pkt->cmd.DataLength));
					for(i = 0;i<pkt->cmd.DataLength;i++)
					{
						dbghex((" %02x",bufBig_gen[i]));
						if(i%8 == 7)
						{
							dbgtag(("\r\n"));
						}
					}
					dbgtag(("\r\n"));
					#endif

					UartControlSendCmd(0x9010, bufBig_gen,pkt->cmd.DataLength -2);

					HalLedBlink(HAL_LED_1, 2,50,300);
				}
				#endif
				break;
			case GENERICAPP_CMDID_ZD0_RESTART:
				#if ! defined(RTR_NWK)
				{
					syncStatusCnt = 0;
					srand(ramdomSeed);
					osal_start_timerEx( GenericApp_TaskID,EVT_SyncStatus_Generic,rand()%3000 );
				}
				#endif
				break;
			case GENERICAPP_CMDID_SYNC_STATUS:
				#if ! defined(RTR_NWK)
				{
					////in:		groupId[2],sync[1],cmds[...]
					GroupId_gen = *((uint16*)(pkt->cmd.Data+2));
					if(pkt->cmd.Data[4] != 0)
					{
						ledBrightness_gen = pkt->cmd.Data[5];
						ledYellow_gen = pkt->cmd.Data[6];
						ledColorR_gen = pkt->cmd.Data[7];
						ledColorG_gen = pkt->cmd.Data[8];
						ledColorB_gen = pkt->cmd.Data[9];
					}
					syncStatusOk = 1;
				}
				#endif
				break;
			case GENERICAPP_CMDID_LIGHT:
				#if ! defined(RTR_NWK)
				{
					ProcessMsgLight(&pkt->cmd.Data[2]);
				}
				#endif
				break;

			case GENERICAPP_CMDID_ADD_TO_GROUP:
				#if ! defined(RTR_NWK)
				{
					GroupId_gen = *((uint16*)(pkt->cmd.Data+2));
				}
				#endif
				break;
			case GENERICAPP_CMDID_REMOVE_FROM_GROUP:
				#if ! defined(RTR_NWK)
				{
					GroupId_gen = 0;
				}
				#endif
				break;
			case GENERICAPP_CMDID_BROADCAST_GROUP_CTRL:
				#if ! defined(RTR_NWK)
				{
					uint16 groupId = *((uint16*)(pkt->cmd.Data+2));
					if(groupId == GroupId_gen)
					{
						uint16 cmdId2 = (pkt->cmd.Data[4])|(pkt->cmd.Data[5]<<8);
						switch(cmdId2)
						{
						case GENERICAPP_CMDID_LIGHT:
							ProcessMsgLight(&pkt->cmd.Data[6]);
							break;
						case GENERICAPP_CMDID_REMOVE_FROM_GROUP:
							#if ! defined(RTR_NWK)
							{
								GroupId_gen = 0;
							}
							#endif
							break;
						}
					}
				}
				#endif
				break;
			}
			break;
	}
}
#endif

/*********************************************************************
 * @fn      GenericApp_SendTheMessage
 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_SendMessage(afAddrMode_t addrMode,uint16 addr ,uint8* buf,uint16 buflen )
{
	
	GenericApp_DstAddr.addrMode = addrMode;
	GenericApp_DstAddr.endPoint = GENERICAPP_ENDPOINT;
	GenericApp_DstAddr.addr.shortAddr = addr;
	
	
	if ( AF_DataRequest( &GenericApp_DstAddr, &GenericApp_epDesc,
											 GENERICAPP_CLUSTERID,
											 buflen,
											 (byte *)buf,
											 &GenericApp_TransID,
											 AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
	{
	}
	else
	{} 
}

/*********************************************************************
 * @fn      GenericApp_SendTheMessage
 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_SendMessageCmd(afAddrMode_t addrMode,uint16 addr ,uint16 cmd,uint8* buf,uint16 buflen )
{
	int16 i;
	for(i=buflen+1;i>1;i--)
		buf[i] = buf[i-2];
	buf[0] = cmd&0xff;
	buf[1] = cmd>>8;
	
	GenericApp_SendMessage(addrMode,addr,buf,buflen + 2);
}

#if defined( IAR_ARMCM3_LM )
/*********************************************************************
 * @fn      GenericApp_ProcessRtosMessage
 *
 * @brief   Receive message from RTOS queue, send response back.
 *
 * @param   none
 *
 * @return  none
 */
static void GenericApp_ProcessRtosMessage( void )
{
	osalQueue_t inMsg;

	if ( osal_queue_receive( OsalQueue, &inMsg, 0 ) == pdPASS )
	{
		uint8 cmndId = inMsg.cmnd;
		uint32 counter = osal_build_uint32( inMsg.cbuf, 4 );

		switch ( cmndId )
		{
			case CMD_INCR:
				counter += 1;  /* Increment the incoming counter */
											 /* Intentionally fall through next case */

			case CMD_ECHO:
			{
				userQueue_t outMsg;

				outMsg.resp = RSP_CODE | cmndId;  /* Response ID */
				osal_buffer_uint32( outMsg.rbuf, counter );    /* Increment counter */
				osal_queue_send( UserQueue1, &outMsg, 0 );  /* Send back to UserTask */
				break;
			}
			
			default:
				break;  /* Ignore unknown command */    
		}
	}
}
#endif
#if ! defined(RTR_NWK)
void ReportStatus()
{
	{
		//report short addr ...info... to the coordinator
		uint16 shortAddr = NLME_GetShortAddr();
		//uint8 group;
		dbghex(("Addr=%x\r\n",shortAddr));
		bufBig_gen[0] = shortAddr&0xff;
		bufBig_gen[1] = shortAddr>>8;
		
		
		GenericApp_SendMessageCmd(afAddr16Bit,0,GENERICAPP_CMDID_REPORT_SHORTADDR,bufBig_gen,2);
	}
}
#endif
/*********************************************************************
 * @fn      SerialApp_CallBack回调函数
 *
 * @brief   Send data OTA.
 *
 * @param   port - UART port.
 * @param   event - the UART port event flag.
 *
 * @return  none
 */
 /////串口协议(简单，帧最长为256字节)
 /////len[1],data[n],crc[1]
 /////中括号内表示字节数，len:data长度，不包含len字节和crc字节。data:	cmdID(low),cmdID(high),......
 /////收到每条命令，都要有相应的回复，获取命令回复数据内容，操作命令回复ok，
 /////		不支持的命令回复not supported,校验出错回复crc err
static void SerialApp_CallBack(uint8 port, uint8 event)
{
#if 1
	(void)port;

	//int16 i=0;
	
	if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)) &&
#if SERIAL_APP_LOOPBACK
			(SerialApp_Len < SERIAL_APP_TX_MAX))
#else
			!SerialApp_Len)
#endif
	{
		if (!SerialApp_Len && 
			//(SerialApp_Len = HalUARTRead(SERIAL_APP_PORT, SerialApp_Buf, SERIAL_APP_TX_MAX)))
			(SerialApp_Len = HalUARTRead(CTRL_PORT, SerialApp_Buf, SERIAL_APP_TX_MAX)))
		{
			dbgint(("serial receive Len=%d,",SerialApp_Len));
			dbghex(("data=%02x,",SerialApp_Buf[0]));
			dbghex(("%02x,",SerialApp_Buf[1]));
			dbghex(("%02x,",SerialApp_Buf[2]));
			dbghex(("%02x,",SerialApp_Buf[3]));
			dbghex(("%02x,",SerialApp_Buf[4]));
			dbghex(("%02x\r\n",SerialApp_Buf[5]));
                        
			// Pre-pend sequence number to the Tx message.
			uint8 dataLen = SerialApp_Buf[1];
			uint16 cmdid ;

			#if 1
			/*
			if(dataLen > (256-2) || dataLen + 2 != SerialApp_Len)
			{
				//len error
				UartControlSendSimple(SerialApp_Buf,0,0x0004);
				goto exit;
			}
			*/
			if(CalcChecksum(&SerialApp_Buf[UART_PROTOCOL_SEQ_START],dataLen) != SerialApp_Buf[dataLen+2])
			{
				//crc error,do not send ack
				//UartControlSendSimple(SerialApp_Buf,0,0x0005);
				goto exit;
			}
			if(SerialApp_Buf[UART_PROTOCOL_ACK_START] == 1)
			{
				// is the ack frame,do nothing
				goto exit;
			}
			#endif


			cmdid = (SerialApp_Buf[UART_PROTOCOL_CMDID_START+0])|(SerialApp_Buf[UART_PROTOCOL_CMDID_START+1]<<8);

			memcpy(bufBig_gen,SerialApp_Buf,SerialApp_Len);
			UartControlSendAck(bufBig_gen);	
			////len,cmdID(low),cmdID(high),...,crc
			switch(cmdid) //解析命令字
			{

			#if defined(ZDO_COORDINATOR)
			case 0x0001://read nv items
				{
					uint16 id = (SerialApp_Buf[UART_PROTOCOL_DATA_START+0])|(SerialApp_Buf[UART_PROTOCOL_DATA_START+1]<<8);
					/* Length of item in NV memory */
					uint16 len = osal_nv_item_len(id);
					uint8 *pRetBuf;

					dbgint(("id[%x] ",id));
					dbgint(("len=%d\r\n",len));
					pRetBuf = osal_mem_alloc(len);
					if (pRetBuf != NULL)
					{
						if ((osal_nv_read(id,0, len, pRetBuf)) == ZSUCCESS)
						{
							int16 i;
							for(i = 0;i<len;i++)
							{
								dbghex(("%02x ",pRetBuf[i]));
								if(i%8 == 7)
								{
									dbgtag(("\r\n"));

								}
							}
							dbgtag(("\r\n"));
						}
					}
					osal_mem_free(pRetBuf);
				}
				break;
			case 0x0003://send control cmds to any Devices
				{
					////in:		addrmode[1],addr[2],cmds[...]
					////inSub:	cmds[...]
					uint16 addr = (SerialApp_Buf[UART_PROTOCOL_DATA_START+1])|(SerialApp_Buf[UART_PROTOCOL_DATA_START+2]<<8);
					uint16 cmd = (SerialApp_Buf[UART_PROTOCOL_DATA_START+3])|(SerialApp_Buf[UART_PROTOCOL_DATA_START+4]<<8);

					GenericApp_SendMessageCmd((afAddrMode_t)SerialApp_Buf[UART_PROTOCOL_DATA_START+0], addr,cmd,&SerialApp_Buf[UART_PROTOCOL_DATA_START+3] , dataLen-UART_PROTOCOL_CMDID_START-3);
				}
				break;

			case 0x0007://sync device status
				{
					////in:		addr[2],groupId[2],sync[1],cmds[...]
					////inSub:	groupId[2],sync[1],cmds[...]
					uint16 addr = (SerialApp_Buf[UART_PROTOCOL_DATA_START+0])|(SerialApp_Buf[UART_PROTOCOL_DATA_START+1]<<8);

					GenericApp_SendMessageCmd(afAddr16Bit, addr,GENERICAPP_CMDID_SYNC_STATUS,&SerialApp_Buf[UART_PROTOCOL_DATA_START+2] , dataLen-UART_PROTOCOL_CMDID_START-2);
				}
				break;

			case 0x00021://add to a group
				{
					////in:		addr[2],group ID[2]
					////inSub:	group ID[2]
					uint16 addr = (SerialApp_Buf[UART_PROTOCOL_DATA_START+0])|(SerialApp_Buf[UART_PROTOCOL_DATA_START+1]<<8);
					

					GenericApp_SendMessageCmd(afAddr16Bit, addr,GENERICAPP_CMDID_ADD_TO_GROUP, &SerialApp_Buf[UART_PROTOCOL_DATA_START+2], dataLen-UART_PROTOCOL_CMDID_START-2);
				}
				break;
			case 0x00022://remove from a group
				{
					////in:		addr[2],group ID[2]
					////inSub:	group ID[2]
					uint16 addr = (SerialApp_Buf[UART_PROTOCOL_DATA_START+0])|(SerialApp_Buf[UART_PROTOCOL_DATA_START+1]<<8);
					

					GenericApp_SendMessageCmd(afAddr16Bit, addr,GENERICAPP_CMDID_REMOVE_FROM_GROUP, &SerialApp_Buf[UART_PROTOCOL_DATA_START+2], dataLen-UART_PROTOCOL_CMDID_START-2);
				}
				break;

			case 0x0030://endDevice control
				{
					////in:		addr[2],cmds[2],data[...]
					////inSub:	data[...]
					uint16 addr = (SerialApp_Buf[UART_PROTOCOL_DATA_START+0])|(SerialApp_Buf[UART_PROTOCOL_DATA_START+1]<<8);
					uint16 cmd = (SerialApp_Buf[UART_PROTOCOL_DATA_START+2])|(SerialApp_Buf[UART_PROTOCOL_DATA_START+3]<<8);

					dbghex(("0x0030 data=%02x,",SerialApp_Buf[UART_PROTOCOL_DATA_START+0]));
					dbghex(("%02x,",SerialApp_Buf[UART_PROTOCOL_DATA_START+1]));
					dbghex(("%02x,",SerialApp_Buf[UART_PROTOCOL_DATA_START+2]));
					dbghex(("%02x,",SerialApp_Buf[UART_PROTOCOL_DATA_START+3]));
					dbghex(("%02x,",SerialApp_Buf[UART_PROTOCOL_DATA_START+4]));
					dbghex(("%02x\r\n",SerialApp_Buf[UART_PROTOCOL_DATA_START+5]));
					GenericApp_SendMessageCmd(afAddr16Bit, addr,cmd, &SerialApp_Buf[UART_PROTOCOL_DATA_START+4], dataLen-UART_PROTOCOL_CMDID_START-4);

				}
				break;
			case 0x0040://group control
				{
					////in:		group id[2],cmds[2],data[...]
					////inSub:	group id[2],cmds[2],data[...]

					GenericApp_SendMessageCmd(afAddrBroadcast, 0xffff,GENERICAPP_CMDID_BROADCAST_GROUP_CTRL, &SerialApp_Buf[UART_PROTOCOL_DATA_START], dataLen-UART_PROTOCOL_CMDID_START);

				}
				break;
			case 0x0050://broadcast control
				{
					////in:		cmds[2],data[...]
					////inSub:	data[...]
					uint16 cmd = (SerialApp_Buf[UART_PROTOCOL_DATA_START+0])|(SerialApp_Buf[UART_PROTOCOL_DATA_START+1]<<8);

					GenericApp_SendMessageCmd(afAddrBroadcast, 0xffff,cmd, &SerialApp_Buf[UART_PROTOCOL_DATA_START+2], dataLen-UART_PROTOCOL_CMDID_START-2);

				}
				break;

			#endif
			default:
				{
					//not supported
					//UartControlSendSimple(SerialApp_Buf,0,0x0006);
				}
				break;
			}
				
		}
	}
exit:
	SerialApp_Len = 0;
#endif
}
#define TIMER_FREE
#if ! defined(RTR_NWK)
static volatile uint8 T4Count = 0;
void initTimer()
{
	PERCFG &= ~0x40;
	PERCFG |= 0x03; // Move USART0 and USART1 to Alternative 2 location to allow all Timer 1 channels on P0
	P2DIR = (P2DIR & ~0xC0) | 0xC0; // Give priority to Timer 1
	P0SEL |= 0x7C;

	T1CC0L = 0XFF;
	T1CC0H = 0x00;
	//T1CCTL0 = 0X24;

	T1CC1L = 0X01;
	T1CC1H = 0x00;
	T1CCTL1 = 0X1c;

	T1CC2L = 0X01;//value 0 makes it most bright
	T1CC2H = 0X00;
	T1CCTL2 = 0X1c;

	T1CC3L = 0X01;
	T1CC3H = 0x00;
	T1CCTL3 = 0X1c;

	T1CC4L = 0X01;
	T1CC4H = 0x00;
	T1CCTL4 = 0X1c;

	//T1CTL |= 0x0f;
	T1CTL = 0x0e;
	//T1IE = 1;
	//EA = 1;


}
void ProcessMsgLight(uint8* data)
{
	ledBrightness_gen = data[0];
	ledYellow_gen = data[1];
	ledColorR_gen = data[2];
	ledColorG_gen = data[3];
	ledColorB_gen = data[4];

	//value 0 makes it most bright,so if you want most dark,set value 1
	T1CC1L = ledBrightness_gen == 0?1:ledBrightness_gen;
	T1CC1H = 0;//T1CC1H must be setted after T1CC1L,otherwise it won't work
	T1CC2L = ledColorR_gen == 0?1:ledColorR_gen;
	T1CC2H = 0;
	T1CC3L = ledColorG_gen == 0?1:ledColorG_gen;
	T1CC3H = 0;
	T1CC4L = ledColorB_gen == 0?1:ledColorB_gen;
	T1CC4H = 0;
}
#endif
uint8 CalcChecksum(uint8*buf,uint8 len)
{
	uint16 checksum = 0;
	uint8 i;
	for(i=0;i<len;i++)
		checksum += buf[i];
	return checksum & 0xff;
}

XDATA uint32 uartSendSeq = 0;


////buf复用,len为你的数据长度，不考虑头(len)、尾(crc)
void UartControlSend(uint8*buf,uint8 dataLen)
{
	//int16 i;
	//if(len + 2 >SERIAL_APP_TX_MAX)
	//	;//error
	buf[0] = 0xAA;
	buf[1] = dataLen+5;
	buf[2] = uartSendSeq;
	buf[3] = uartSendSeq>>8;
	buf[4] = uartSendSeq>>16;
	buf[5] = uartSendSeq>>24;
	buf[6] = 0;// not ack
	buf[7 + dataLen] = CalcChecksum(&buf[UART_PROTOCOL_SEQ_START],dataLen+5);
	buf[8 + dataLen] = 0x55;
	HalUARTWrite(CTRL_PORT,buf,dataLen+9);

	uartSendSeq++;
#if 0
	dbgint(("uart send=%d\r\n",dataLen));
        
	for(i = 0;i<dataLen;i++)
	{
		dbghex((" %02x",buf[i]));
		if(i%8 == 7)
		{
			dbgtag(("\r\n"));
		}
	}
	dbgtag(("\r\n"));
#endif
}
void UartControlSendAck(uint8*buf)
{
	//if(len + 2 >SERIAL_APP_TX_MAX)
	//	;//error
	buf[0] = 0xAA;
	buf[1] = 5;
	//buf[2] = uartSendSeq;	//use same seq received
	//buf[3] = uartSendSeq>>8;
	//buf[4] = uartSendSeq>>16;
	//buf[5] = uartSendSeq>>24;
	buf[6] = 1;// ack
	buf[7 ] = CalcChecksum(&buf[UART_PROTOCOL_SEQ_START],5);
	buf[8 ] = 0x55;
	HalUARTWrite(CTRL_PORT,buf,9);

}
void UartControlSendCmd(uint16 retCmdId,uint8*buf,uint8 dataLen)
{
	buf[UART_PROTOCOL_CMDID_START+0] = retCmdId;
	buf[UART_PROTOCOL_CMDID_START+1] = retCmdId>>8;
	UartControlSend(buf,dataLen+2);
}

