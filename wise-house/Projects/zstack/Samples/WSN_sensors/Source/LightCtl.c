/**************************************************************************************************
  Filename:       LightCtl.c
  Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
  Revision:       $Revision: 19453 $

  Description:    Generic Application (no Profile).


  Copyright 2004-2009 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
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
**************************************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends "Hello World" to another "Generic"
  application every 15 seconds.  The application will also
  receive "Hello World" packets.

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

#include "LightCtl.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_adc.h"
#include "hal_key.h"
#include "hal_uart.h"

#include "DS18B20.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */
#if !defined( SERIAL_APP_PORT )
#define SERIAL_APP_PORT  0
#endif

#if !defined( SERIAL_APP_BAUD )
#define SERIAL_APP_BAUD  HAL_UART_BR_38400
//#define SERIAL_APP_BAUD  HAL_UART_BR_115200
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
#define SERIAL_APP_TX_MAX  10
#endif

#define MAXSENSOR 4

/*********************************************************************
 * GLOBAL VARIABLES
 */

static uint8 SerialApp_Buf[SERIAL_APP_TX_MAX];
static uint8 SerialApp_Len;

uint8 sensor_TempData[MAXSENSOR][2];
uint8 sensor_Gas[MAXSENSOR];
uint8 sensor_LightLevel[MAXSENSOR];
uint8 sensor_LightOnOff[MAXSENSOR];
uint8 sensor_Alarm[MAXSENSOR];

//static uint8 SerialApp_RxSeq;
//static uint8 SerialApp_RspBuf[SERIAL_APP_RSP_CNT];


// This list should be filled with Application specific Cluster IDs.
const cId_t LightCtl_ClusterList[LightCtl_MAX_CLUSTERS] =
{
  LightCtl_CLUSTERID
};

const SimpleDescriptionFormat_t LightCtl_SimpleDesc =
{
  LightCtl_ENDPOINT,              //  int Endpoint;
  LightCtl_PROFID,                //  uint16 AppProfId[2];
  LightCtl_DEVICEID,              //  uint16 AppDeviceId[2];
  LightCtl_DEVICE_VERSION,        //  int   AppDevVer:4;
  LightCtl_FLAGS,                 //  int   AppFlags:4;
  LightCtl_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)LightCtl_ClusterList,  //  byte *pAppInClusterList;
  LightCtl_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)LightCtl_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in LightCtl_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t LightCtl_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte LightCtl_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // LightCtl_Init() is called.
devStates_t LightCtl_NwkState;


byte LightCtl_TransID;  // This is the unique message ID (counter)

afAddrType_t LightCtl_DstAddr;
uint8 SensorID = 2; 

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void LightCtl_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
void LightCtl_HandleKeys( byte shift, byte keys );
void LightCtl_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void LightCtl_SendTheMessage( void );

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */
static void SerialApp_CallBack(uint8 port, uint8 event);

uint8 hextoword1(uint8 t );
uint8 hextoword2(uint8 t);
void UartShowNtkInfo(bool flag,uint16 short_ddr,uint8 *pIeeeAddrBuf);
void AfSendNtkInfo(void);
void HalSendFrame(uint8 cmd,uint8 id,uint8 dataL,uint8 dataH);

uint8 myApp_ReadGas( void );
uint8 myApp_ReadLightLevel( void );
uint8 myApp_ReadLightOnOff( void );
uint8 myApp_ReadAlarm( void );

uint8 hextoword1(uint8 t )//十六进制转换成十进制函数
{
  uint8 abc;
  uint8 cba;
  uint8 xx1;
  abc=t;
  cba=0xf0;
  abc=(abc&cba)>>4;
  if(abc<10)
  {
    xx1=abc+48;
  }
  else
  {
    xx1=abc+55;
  }
  return xx1;
}
uint8 hextoword2(uint8 t)
{
  uint8 abc;
  uint8 cba;
  uint8 xx2;
  abc=t;
  cba=0x0f;
  abc=abc&cba;
  if(abc<10)
  {
    xx2=abc+48;
  }
  else
  {
    xx2=abc+55;
  }
  return xx2;
}

void UartShowNtkInfo(bool flag,uint16 short_ddr,uint8 *pIeeeAddrBuf)//往串口送网络信息函数
{
  //  显示网络地址变量
    uint8 yy1;
    uint8 yy2;
    uint8 str_1[ ]="my short address is:";
    uint8 str_2[ ]="build the network successfully";
    uint8 str_5[ ]="join the network successfully ";
    uint8 str_3[ ]={'\n'};
    uint8 str_4[ ]="my ieee address is:";
    uint8 shortaddr[7];
    uint8 *pointer1;
    uint8 *pointer2;
    uint8 *pointer3;
    uint8 *pointer4;
    
    //显示本地网络地址
    //显示本地网络地址
    //short_ddr=NLME_GetShortAddr();
    yy1=(uint8)((short_ddr&0xff00)>>8);
    yy2=(uint8)short_ddr;
    shortaddr[0]=48;
    shortaddr[1]=120;
    shortaddr[2]=hextoword1(yy1);
    shortaddr[3]=hextoword2(yy1);
    shortaddr[4]=hextoword1(yy2);
    shortaddr[5]=hextoword2(yy2);
    shortaddr[6]='\n';
    pointer1=&shortaddr[0];
    pointer2=&str_1[0];
    if(flag == 0x00)
    {
      pointer3=&str_2[0];
    }
    else
    {
      pointer3=&str_5[0];
    }
    pointer4=&str_3[0];
    HalUARTWrite(0,pointer4,1); 
    HalUARTWrite(0,pointer3,29);
    HalUARTWrite(0,pointer4,1);            
    HalUARTWrite(0,pointer2,20);            
    HalUARTWrite(0,pointer1,7);
  
    
    uint8 i;
    uint8 *xad;
    uint8 ieeeAddr_buf[Z_EXTADDR_LEN*2+1];
  
    // Display the extended address.
    //xad = aExtendedAddress + Z_EXTADDR_LEN - 1;
    xad = pIeeeAddrBuf;
  
    for (i = 0; i < Z_EXTADDR_LEN*2; xad--)
    {
      uint8 ch;
      ch = (*xad >> 4) & 0x0F;
      ieeeAddr_buf[i++] = ch + (( ch < 10 ) ? '0' : '7');
      ch = *xad & 0x0F;
      ieeeAddr_buf[i++] = ch + (( ch < 10 ) ? '0' : '7');
    }
    ieeeAddr_buf[Z_EXTADDR_LEN*2] = '\0';
    HalUARTWrite(0,str_4,19);
    HalUARTWrite(0,ieeeAddr_buf,Z_EXTADDR_LEN*2);
    HalUARTWrite(0,pointer4,1);
}

void AfSendNtkInfo(void)//发送网络信息函数
{
  LightCtl_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  LightCtl_DstAddr.endPoint = LightCtl_ENDPOINT;
  LightCtl_DstAddr.addr.shortAddr = 0x0000;  //0x796F;0x0000
  
  uint16 short_ddr;
  uint8 short_ddr_H;
  uint8 short_ddr_L;
  //  uint8 *pointer1;
  uint8 tmpBuf[11];
  
  uint8 *xad;    
  
  short_ddr=NLME_GetShortAddr();
  short_ddr_H=(uint8)((short_ddr&0xff00)>>8);
  short_ddr_L=(uint8)short_ddr;
  
  tmpBuf[0]=0x80;                           //一字节存放数据命令字
  tmpBuf[1]=short_ddr_H;              //一字节存放源地址高8位
  tmpBuf[2]=short_ddr_L;              //一字节存放源地址低8位
  
  xad = NLME_GetExtAddr();
  tmpBuf[3]=*xad++;
  tmpBuf[4]=*xad++;
  tmpBuf[5]=*xad++;
  tmpBuf[6]=*xad++;
  tmpBuf[7]=*xad++;
  tmpBuf[8]=*xad++;
  tmpBuf[9]=*xad++;
  tmpBuf[10]=*xad;
  
  LightCtl_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  LightCtl_DstAddr.endPoint = LightCtl_ENDPOINT;
  LightCtl_DstAddr.addr.shortAddr = 0x00;
  
  if ( AF_DataRequest( &LightCtl_DstAddr,
                 (endPointDesc_t *)&LightCtl_epDesc,
                  LightCtl_CLUSTERID,
                  11, tmpBuf,
                  &LightCtl_TransID, 
                  AF_DISCV_ROUTE, 
                  AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  ;
  }
  else
  {
  ;
  }
}

void HalSendFrame(uint8 cmd,uint8 id,uint8 dataL,uint8 dataH)
{
  uint8 Ackbuf[8];
  
  Ackbuf[0] = 0xef;
  Ackbuf[1] = cmd;
  Ackbuf[2] = id;
  Ackbuf[3] = dataL;
  Ackbuf[4] = dataH;
  Ackbuf[5] = Ackbuf[1]+Ackbuf[2]+Ackbuf[3]+Ackbuf[4];
  Ackbuf[6] = 0xfe;
  HalUARTWrite ( SERIAL_APP_PORT, Ackbuf, 7);
}

/*********************************************************************
 * @fn      LightCtl_Init初始化函数
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
void LightCtl_Init( byte task_id )
{
  LightCtl_TaskID = task_id;
  LightCtl_NwkState = DEV_INIT;
  LightCtl_TransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  LightCtl_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  LightCtl_DstAddr.endPoint = LightCtl_ENDPOINT;
  LightCtl_DstAddr.addr.shortAddr = 0x00;

  // Fill out the endpoint description.
  LightCtl_epDesc.endPoint = LightCtl_ENDPOINT;
  LightCtl_epDesc.task_id = &LightCtl_TaskID;
  LightCtl_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&LightCtl_SimpleDesc;
  LightCtl_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &LightCtl_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( LightCtl_TaskID );
  
  //--------------------------------config uart------------------------------------------
  halUARTCfg_t uartConfig;

  uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.baudRate             = SERIAL_APP_BAUD;
  uartConfig.flowControl          = TRUE;
  uartConfig.flowControlThreshold = SERIAL_APP_THRESH; // 2x30 don't care - see uart driver.
  uartConfig.rx.maxBufSize        = SERIAL_APP_RX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.tx.maxBufSize        = SERIAL_APP_TX_SZ;  // 2x30 don't care - see uart driver.
  uartConfig.idleTimeout          = SERIAL_APP_IDLE;   // 2x30 don't care - see uart driver.
  uartConfig.intEnable            = TRUE;              // 2x30 don't care - see uart driver.
  uartConfig.callBackFunc         = SerialApp_CallBack;
  HalUARTOpen (SERIAL_APP_PORT, &uartConfig);

#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "SerialApp", HAL_LCD_LINE_2 );
#endif

  // Update the display
#if defined ( LCD_SUPPORTED )
    HalLcdWriteString( "LightCtl", HAL_LCD_LINE_1 );
#endif
    
  //ZDO_RegisterForZDOMsg( LightCtl_TaskID, End_Device_Bind_rsp );
  //ZDO_RegisterForZDOMsg( LightCtl_TaskID, Match_Desc_rsp );
}

/*********************************************************************
 * @fn      LightCtl_ProcessEvent总处理函数
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
UINT16 LightCtl_ProcessEvent( byte task_id, UINT16 events )
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
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( LightCtl_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          //LightCtl_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;
          
        case KEY_CHANGE:
          LightCtl_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
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
          LightCtl_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          LightCtl_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (LightCtl_NwkState == DEV_ZB_COORD)
              || (LightCtl_NwkState == DEV_ROUTER)
              || (LightCtl_NwkState == DEV_END_DEVICE) )
          {
            #if defined(ZDO_COORDINATOR)
            
             UartShowNtkInfo(0,NLME_GetShortAddr(),aExtendedAddress + Z_EXTADDR_LEN - 1); 
            
            #else 
            
            AfSendNtkInfo();
              
            #endif
            
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( LightCtl_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in LightCtl_Init()).
  if ( events & LightCtl_SEND_MSG_EVT )  //在此事件中可以定时向协调器发送节点传感器参数信息
  {
    // Send "the" message
    LightCtl_SendTheMessage();

    // Setup to send message again
    osal_start_timerEx( LightCtl_TaskID,
                        LightCtl_SEND_MSG_EVT,
                      LightCtl_SEND_MSG_TIMEOUT );

    // return unprocessed events
    return (events ^ LightCtl_SEND_MSG_EVT);
  }
 
/*  
  //对接收的串口数据进行处理
  if ( events & UART_RX_CB_EVT )  //串口数据处理
  { 

    //SampleApp_SPI_SendData( databuf, rxlen+1+2 );  
    return (events ^ UART_RX_CB_EVT);
  }
*/

  // Discard unknown events
  return 0;
}


/*********************************************************************
 * @fn      LightCtl_HandleKeys按键处理函数
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
void LightCtl_HandleKeys( byte shift, byte keys )
{
  //zAddrType_t dstAddr;
  
  // Shift is used to make each button/switch dual purpose.
  if ( shift )
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
    if ( keys & HAL_KEY_SW_1 )  //数据发送测试
    {
      HalLedBlink ( HAL_LED_1, 5, 50, 100 );
      
      static byte cnt=0;
      
      byte sendBuf[2];
      
      sendBuf[0] = 0x81;
      sendBuf[1] = cnt;
      

      LightCtl_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
      LightCtl_DstAddr.endPoint = LightCtl_ENDPOINT;
      LightCtl_DstAddr.addr.shortAddr = 0x00;
      
      if ( AF_DataRequest( &LightCtl_DstAddr, &LightCtl_epDesc,
                           LightCtl_CLUSTERID,
                           2,
                           (byte *)sendBuf,
                           &LightCtl_TransID,
                           AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
      {
        // Successfully requested to be sent.
        HalLcdWriteValue( cnt, 16, HAL_LCD_LINE_2 );
        
        cnt++;
      }
      
    }

    if ( keys & HAL_KEY_SW_2 )  //传感器节点 按下SW2则开始像网关定期发送数据包
    {     
      static bool sendFlag = 0;
      
      if(sendFlag == 0)
      {
        sendFlag = 1;
        HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
        osal_start_timerEx( LightCtl_TaskID,
                                LightCtl_SEND_MSG_EVT,
                              LightCtl_SEND_MSG_TIMEOUT );
      }
      else
      {      
        sendFlag = 0;
        HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );
        osal_stop_timerEx(LightCtl_TaskID,LightCtl_SEND_MSG_EVT);
      }
    }

    if ( keys & HAL_KEY_SW_3 )
    {
      HalLedBlink ( HAL_LED_1, 5, 50, 100 );
      
      char sendBuf[3];
      sendBuf[0]=0x83;
      sendBuf[1] = 0x02;
      sendBuf[2] = 0x00;
      
      LightCtl_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
      LightCtl_DstAddr.endPoint = LightCtl_ENDPOINT;
      LightCtl_DstAddr.addr.shortAddr = 0xffff;
      
      
      if ( AF_DataRequest( &LightCtl_DstAddr, &LightCtl_epDesc,
                           LightCtl_CLUSTERID,
                           3,
                           (byte *)sendBuf,
                           &LightCtl_TransID,
                           AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
      {
      }
      else
      {} 
      HalLcdWriteString("turn off led3", HAL_LCD_LINE_2 );
    }

    if ( keys & HAL_KEY_SW_4 )
    {
      HalLedBlink ( HAL_LED_1, 5, 50, 100 );
      
      char sendBuf[3];
      sendBuf[0]=0x83;
      sendBuf[1] = 0x02;
      sendBuf[2] = 0x01;
      
      LightCtl_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
      LightCtl_DstAddr.endPoint = LightCtl_ENDPOINT;
      LightCtl_DstAddr.addr.shortAddr = 0xffff;
      
      
      if ( AF_DataRequest( &LightCtl_DstAddr, &LightCtl_epDesc,
                           LightCtl_CLUSTERID,
                           3,
                           (byte *)sendBuf,
                           &LightCtl_TransID,
                           AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
      {
      }
      else
      {} 
      HalLcdWriteString("turn on led3", HAL_LCD_LINE_2 );
      
    }
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      LightCtl_MessageMSGCB  接收数据处理函数
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void LightCtl_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  //uint16 flashTime;
  uint16 short_ddr;
  
  //uint16 short_ddr;
  uint8 short_ddr_H;
  uint8 short_ddr_L;
  uint8 *pIeeeAddr;  
  uint8 i=0;
  
  
  switch ( pkt->clusterId )
  {
    case LightCtl_CLUSTERID:

    switch(pkt->cmd.Data[0]) //简单协议命令字解析
    {
#if defined(ZDO_COORDINATOR)
    case 0x80://上报节点短地址和IEEE地址
        short_ddr_H=pkt->cmd.Data[1];
        short_ddr_L=pkt->cmd.Data[2];
        
        short_ddr=(short_ddr_H<<8)|short_ddr_L;
        
        pIeeeAddr = &pkt->cmd.Data[3];
        
        UartShowNtkInfo(1,short_ddr,pIeeeAddr + Z_EXTADDR_LEN - 1);
      break;
      
    case 0x81://按键发送数字测试
        HalLcdWriteValue( pkt->cmd.Data[1], 16, HAL_LCD_LINE_2 );
      break;
      
    case 0x82://LightCtl_SendTheMessage事件--->网关接收传感器发来的数据包
      sensor_TempData[pkt->cmd.Data[1]-1][0] = pkt->cmd.Data[2];
      sensor_TempData[pkt->cmd.Data[1]-1][1] = pkt->cmd.Data[3];
      
      sensor_Gas[pkt->cmd.Data[1]-1] = pkt->cmd.Data[4];
      sensor_LightLevel[pkt->cmd.Data[1]-1] = pkt->cmd.Data[5];
      sensor_LightOnOff[pkt->cmd.Data[1]-1] = pkt->cmd.Data[6];
      sensor_Alarm[pkt->cmd.Data[1]-1] = pkt->cmd.Data[7];
      
      for(i=0;i<MAXSENSOR;i++)
      {
        HalSendFrame(0x85,i+1,sensor_Alarm[i],0x00);
        Delay_nus(10);
      }   
      
      break;
#else  //
    case 0x83:  //开关灯设备

      if(SensorID == pkt->cmd.Data[1])
      {
        if(pkt->cmd.Data[2] == 0)
          HalLedSet ( HAL_LED_3, HAL_LED_MODE_OFF );
        else
          HalLedSet ( HAL_LED_3, HAL_LED_MODE_ON );
      }
      
      
      break;
      
#endif
      
    default :
      break;
    }
      
      break;
  }
}

/*********************************************************************
 * @fn      LightCtl_SendTheMessage发送数据函数

 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */
void LightCtl_SendTheMessage( void )
{
  UINT8 theMessageData[8];

  theMessageData[0] = 0x82;                   //一字节存放命令字
  theMessageData[1] = SensorID;                   //一字节存放ID
  //获取温度值 2字节
  read_data(&theMessageData[2]); 
  //获取气体含量1字节
  theMessageData[4] = myApp_ReadGas();
  //获取光强含量 1字节
  theMessageData[5] = myApp_ReadLightLevel();
  //获取灯设备状态 0->off 1->on
  theMessageData[6] = myApp_ReadLightOnOff();
  //获取报警信号
  theMessageData[7] = myApp_ReadAlarm();
  
  //DataChange(&theMessageData[2],ch); //数据处理           
  //HalLcdWriteString("temp is:", HAL_LCD_LINE_1 );
  //HalLcdWriteString( (char *)ch, HAL_LCD_LINE_2 ); 
  
  LightCtl_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  LightCtl_DstAddr.endPoint = LightCtl_ENDPOINT;
  LightCtl_DstAddr.addr.shortAddr = 0x00;  

  if ( AF_DataRequest( &LightCtl_DstAddr, &LightCtl_epDesc,
                       LightCtl_CLUSTERID,
                       //(byte)osal_strlen( theMessageData ) + 1,
                       8,
                       (byte *)&theMessageData,
                       &LightCtl_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
  }
  else
  {
    // Error occurred in request to send.
  }
}

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
static void SerialApp_CallBack(uint8 port, uint8 event)
{
  (void)port;

  uint8 checksum = 0;
  int i=0;
  char sendBuf[3];
  
  if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)) &&
#if SERIAL_APP_LOOPBACK
      (SerialApp_Len < SERIAL_APP_TX_MAX))
#else
      !SerialApp_Len)
#endif
  {
    if (!SerialApp_Len && 
      (SerialApp_Len = HalUARTRead(SERIAL_APP_PORT, SerialApp_Buf, SERIAL_APP_TX_MAX)))
    {
      // Pre-pend sequence number to the Tx message.
      SerialApp_Len = 0;
      checksum = SerialApp_Buf[1]+SerialApp_Buf[2]+SerialApp_Buf[3];

/*      
      //回显数据(测试用)
      word_buffer[0]='l';
      word_buffer[1]='e';
      word_buffer[2]='n';
      word_buffer[3]=':';  
      word_buffer[4]='1';
      word_buffer[5]='2';
      word_buffer[6]='3';
      word_buffer[7]='\n';
      pointer1=word_buffer; 
      HalUARTWrite ( SERIAL_APP_PORT, pointer1, 8 );
*/
      
      
      if((SerialApp_Buf[0] == 0xef)&&(SerialApp_Buf[5] == 0xfe)&&(SerialApp_Buf[4] == checksum))
      {
        switch(SerialApp_Buf[1]) //解析命令字
        {
          case 0xc0://全部查询            
            for(i=0;i<MAXSENSOR;i++)
            {
              HalSendFrame(0x81,i+1,sensor_TempData[i][0],sensor_TempData[i][1]);
              Delay_nus(10);
            } 
            
            Delay_nus(20);
            
            for(i=0;i<MAXSENSOR;i++)
            {
              HalSendFrame(0x82,i+1,sensor_Gas[i],0x00);
              Delay_nus(10);
            } 
            
            Delay_nus(20);
            
            for(i=0;i<MAXSENSOR;i++)
            {
              HalSendFrame(0x83,i+1,sensor_LightLevel[i],0x00);
              Delay_nus(10);
            }
            
            Delay_nus(20);
            
            for(i=0;i<MAXSENSOR;i++)
            {
              HalSendFrame(0x84,i+1,sensor_LightOnOff[i],0x00);
              Delay_nus(10);
            }   
            
            Delay_nus(20);
            
            //发送完毕应答
            HalSendFrame(0x8F,0x00,0x00,0x00);  
            
            break;
            
          case 0xC1://read tempture
            //read_data(sensor_TempData[0]);
            DataChange(sensor_TempData[0],ch); //数据处理
            
            HalLcdWriteString("Rcv temp is:", HAL_LCD_LINE_1 );
            HalLcdWriteString( (char *)ch, HAL_LCD_LINE_2 );
             
            for(i=0;i<MAXSENSOR;i++)
            {
              HalSendFrame(0x81,i+1,sensor_TempData[i][0],sensor_TempData[i][1]);
              Delay_nus(10);
            }         
            
            //发送完毕应答
            HalSendFrame(0x8F,0x00,0x00,0x00);           
            
            break;
            
          case 0xC2://read sensor_Gas
            
            for(i=0;i<MAXSENSOR;i++)
            {
              HalSendFrame(0x82,i+1,sensor_Gas[i],0x00);
              Delay_nus(10);
            }         
            
            //发送完毕应答
            HalSendFrame(0x8F,0x00,0x00,0x00);           
            
            break;
            
          case 0xC3://read sensor_LightLevel
            
            for(i=0;i<MAXSENSOR;i++)
            {
              HalSendFrame(0x83,i+1,sensor_LightLevel[i],0x00);
              Delay_nus(10);
            }         
            
            //发送完毕应答
            HalSendFrame(0x8F,0x00,0x00,0x00);           
            
            break;        
          
            
          case 0xC4://read sensor_LightOnOff
            
            for(i=0;i<MAXSENSOR;i++)
            {
              HalSendFrame(0x84,i+1,sensor_LightOnOff[i],0x00);
              Delay_nus(10);
            }         
            
            //发送完毕应答
            HalSendFrame(0x8F,0x00,0x00,0x00);    
          
          case 0xC5://turn on off lights
            
            
            sendBuf[0]=0x83;
            sendBuf[1] = SerialApp_Buf[2];
            sendBuf[2] = SerialApp_Buf[3];
            
            LightCtl_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
            LightCtl_DstAddr.endPoint = LightCtl_ENDPOINT;
            LightCtl_DstAddr.addr.shortAddr = 0xffff;
            
            
            if ( AF_DataRequest( &LightCtl_DstAddr, &LightCtl_epDesc,
                                 LightCtl_CLUSTERID,
                                 3,
                                 (byte *)sendBuf,
                                 &LightCtl_TransID,
                                 AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
            {
            }
            else
            {} 
            
            break;
            
        }
        
      }
        
    }
  }
  
}


#define HAL_ADC_REF_125V    0x00    /* Internal 1.25V Reference */
#define HAL_ADC_DEC_064     0x00    /* Decimate by 64 : 8-bit resolution */
#define HAL_ADC_DEC_128     0x10    /* Decimate by 128 : 10-bit resolution */
#define HAL_ADC_DEC_512     0x30    /* Decimate by 512 : 14-bit resolution */
#define HAL_ADC_CHN_VDD3    0x0f    /* Input channel: VDD/3 */
#define HAL_ADC_CHN_TEMP    0x0e    /* Temperature sensor */


/******************************************************************************
 * @fn          myApp_ReadGas
 *
 * @brief       瓦斯传感器使用p07口
 *
 * @param
 *
 * @return
 ******************************************************************************/
uint8 myApp_ReadGas( void )
{
  uint8 value;
  
  //P07设为输入
  P0DIR &= ~0x80;  // 设置P0.7为输入方式
  asm("NOP");asm("NOP");

  /* Clear ADC interrupt flag */
  ADCIF = 0;

  ADCCON3 = (0x80 | HAL_ADC_DEC_064 | HAL_ADC_CHANNEL_7);

  /* Wait for the conversion to finish */
  while ( !ADCIF );
  
  asm("NOP");asm("NOP");

  /* Get the result */
  value = ADCH;
  
  return value;
}

/******************************************************************************
 * @fn          myApp_ReadGas
 *
 * @brief       光敏传感器使用p01口
 *
 * @param
 *
 * @return
 ******************************************************************************/
uint8 myApp_ReadLightLevel( void )
{
  uint8 value;
  
  //P01设为输入
  P0DIR &= ~0x02;  // 设置P0.1为输入方式
  
  asm("NOP");asm("NOP");

  /* Clear ADC interrupt flag */
  ADCIF = 0;

  ADCCON3 = (0x80 | HAL_ADC_DEC_064 | HAL_ADC_CHANNEL_1);

  /* Wait for the conversion to finish */
  while ( !ADCIF );
  
  asm("NOP");asm("NOP");

  /* Get the result */
  value = ADCH;
  
  return value;
}

/******************************************************************************
 * @fn          myApp_ReadLightOnOff
 *
 * @brief       灯控使用P11,读取led3的状态
 *
 * @param
 *
 * @return
 ******************************************************************************/
uint8 myApp_ReadLightOnOff( void )
{
  uint8 value;
  
  if(P1_4 == 0)
  {
    value = 0;
  }
  else
  {
    value = 1;
  }
  
  return value;
}

/******************************************************************************
 * @fn          myApp_ReadAlarm
 *
 * @brief       使用P05
 *
 * @param
 *
 * @return
 ******************************************************************************/

uint8 myApp_ReadAlarm( void )
{
  uint8 value;
  
  if(P0_5 == 0)
  {
    value = 0;
  }
  else
  {
    value = 1;
  }
  
  return value;
}

/*********************************************************************
*********************************************************************/
