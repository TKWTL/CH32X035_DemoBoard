/********************************** (C) COPYRIGHT *******************************
* File Name          : PD_process.c
* Author             : WCH
* Version            : V1.0.1
* Date               : 2025/10/27
* Description        : This file provides all the PD firmware functions.
*********************************************************************************
* Copyright (c) 2023 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include "debug.h"
#include <string.h>
#include "pd.h"
#include "pd_port.h"

#define PD_EXT_TYPE_EXTENDED_CONTROL      0x10U
#define PD_EXT_TYPE_EPR_SOURCE_CAP        0x11U
#define PD_EXT_CTRL_EPR_GET_SOURCE_CAP    1U
#define PD_EXT_CTRL_EPR_GET_SINK_CAP      2U
#define PD_EXT_CTRL_EPR_KEEPALIVE         3U
#define PD_EXT_CTRL_EPR_KEEPALIVE_ACK     4U
#define PD_EPR_MODE_ENTER                 1U
#define PD_EPR_MODE_ENTER_ACK             2U
#define PD_EPR_MODE_ENTER_SUCCESS         3U
#define PD_EPR_MODE_ENTER_FAILED          4U
#define PD_EPR_MODE_EXIT                  5U
#define PD_EXT_CHUNK_DATA_MAX             26U
#define PD_EPR_CAP_BUFFER_SIZE            64U
#define PD_EPR_GET_CAP_DELAY_MS           120U
#define PD_EPR_SOURCE_CAP_TIMEOUT_MS      500U
#define PD_EPR_ENTER_TIMEOUT_MS           550U
#define PD_EPR_KEEPALIVE_PERIOD_MS        375U
#define PD_EPR_KEEPALIVE_ACK_TIMEOUT_MS   100U
#define PD_VBUS_POWERED_SUPPRESS_HARD_RESET 1U
#define PD_GET_SOURCE_CAP_RETRY_MS          700U
#define PD_GET_SOURCE_CAP_MAX_RETRIES       3U
#define PD_VBUS_DETACH_THRESHOLD_MV         3500U
#define PD_VBUS_DETACH_DEBOUNCE_COUNT       3U

/* Internal WCH-derived protocol helpers.  Only PD_Init/PD_Task/getters are
 * exported through pd.h. */
static void PD_Rx_Mode(void);
static void PD_SINK_Init(void);
static void PD_PHY_Reset(void);
static UINT8 PD_Detect(void);
static void PD_Det_Proc(void);
static void PD_Load_Header(UINT8 ex, UINT8 msg_type);
static UINT8 PD_Send_Handle(const UINT8 *pbuf, UINT8 len);
static void PD_Main_Proc(void);
static void PD_PDO_Analyse(UINT8 pdo_idx, UINT8 *srccap, UINT16 *current, UINT16 *voltage);



static __attribute__ ((aligned(4))) uint8_t PD_Rx_Buf[34];                           /* PD receive buffer */
static __attribute__ ((aligned(4))) uint8_t PD_Tx_Buf[34];                           /* PD send buffer */


static UINT16 Tmr_Ms_Dlt;                                                         /* elapsed milliseconds from free-running SysTick */

static PD_CONTROL PD_Ctl;                                                              /* PD Control Related Structures */

static UINT8 Adapter_SrcCap[30];                                                    /* SrcCap message from the adapter */

static __IO UINT8 PDO_Len;
static __IO UINT8 PD_Selected_PDO;
static __IO UINT16 PD_Selected_mV;
static __IO UINT16 PD_Selected_mA;
static __IO UINT8 PD_SourcePDO_Count;
static UINT8 PD_SourcePDO_Raw[28];
static __IO UINT8 PD_GetSrcCap_Sent;

/* USB PD 3.1 EPR sink state.  The CH32X035 PHY can transport these
 * messages; the WCH USBPD_SNK example does not provide the EPR policy
 * engine, so this file implements the small subset needed for 28V Fixed. */
static __IO UINT8 PD_Source_EPR_Capable;
static __IO UINT8 PD_EPR_ModeActive;
static __IO UINT8 PD_EPR_ContractActive;
static __IO UINT8 PD_SPR_ContractActive;
static __IO UINT8 PD_EPR_SourcePDO_Count;
static UINT8 PD_EPR_SourcePDO_Raw[PD_EPR_CAP_BUFFER_SIZE];

#define EPR_ST_OFF                 0u
#define EPR_ST_SPR_NEGOTIATING     1u
#define EPR_ST_WAIT_ENTER_ACK      2u
#define EPR_ST_WAIT_ENTER_SUCCESS  3u
#define EPR_ST_WAIT_SOURCE_CAP     4u
#define EPR_ST_WAIT_REQUEST_ACCEPT 5u
#define EPR_ST_WAIT_REQUEST_PSRDY  6u
#define EPR_ST_ACTIVE              7u

static __IO UINT8  PD_EPR_State;
static __IO UINT8  PD_SoftResetRecoveryPending;
static __IO UINT8  PD_ProtocolRecoveryCount;
static __IO UINT8  PD_EPR_FailedForAttach;
static __IO UINT8  PD_EPR_GetCapSent;
static __IO UINT8  PD_EPR_KeepAliveWaitAck;
static UINT16 PD_EPR_TimerMs;
static UINT16 PD_EPR_KeepAliveMs;
static UINT16 PD_EPR_KeepAliveAckMs;
static UINT16 PD_EPR_CapDataSize;
static UINT8  PD_EPR_LastChunk;
static UINT32 PD_EPR_SelectedRawPDO;

static uint32_t s_pd_task_last_ms;
static uint32_t s_pd_detect_last_ms;
static uint8_t s_pd_task_started;
static uint8_t s_pd_get_src_cap_retries;
static uint8_t s_pd_vbus_valid;
static uint8_t s_pd_vbus_detach_count;
static uint16_t s_pd_vbus_mv;

/* SrcCap Table */
static const UINT8 SrcCap_5V3A_Tab[4]  = { 0X2C, 0X91, 0X01, 0X3E };
static const UINT8 SinkCap_5V1A_Tab[4] = { 0X64, 0X90, 0X01, 0X36 };

/* PD3.0 */
static const UINT8 SrcCap_Ext_Tab[28] =
{
    0X18, 0X80, 0X63, 0X00,
    0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X01, 0X00,
    0X00, 0X00, 0X07, 0X03,
    0X00, 0X00, 0X00, 0X00,
    0X00, 0X00, 0X00, 0X03,
    0X00, 0X12, 0X00, 0X00,
};

static const UINT8 Status_Ext_Tab[8] =
{
    0X06, 0X80, 0X16, 0X00,
    0X00, 0X00, 0X00, 0X00,
};


/*********************************************************************
 * @fn      PD_Rx_Mode
 *
 * @brief   This function uses to enter reception mode.
 *
 * @return  none
 */
static void PD_Rx_Mode( void )
{
    PD_Port_RxStart();
}

/*********************************************************************
 * @fn      PD_SINK_Init
 *
 * @brief   This function uses to initialize SNK mode.
 *
 * @return  none
 */
static void PD_SINK_Init( )
{
    PD_Ctl.Flag.Bit.PR_Role = 0;
    PD_Ctl.Flag.Bit.Auto_Ack_PRRole = 0;
    PD_Port_SetPowerRole(PD_PORT_ROLE_SINK);
}

/*********************************************************************
 * @fn      PD_PHY_Reset
 *
 * @brief   This function uses to reset PD PHY.
 *
 * @return  none
 */
static void PD_PHY_Reset( void )
{
    PD_SINK_Init( );
    PD_Ctl.Flag.Bit.Stop_Det_Chk = 0;
    PD_Ctl.Flag.Bit.Connected = 0;
    PD_Ctl.Det_Cnt = 0;
    PD_Ctl.Msg_ID = 0;
    PD_Ctl.Err_Op_Cnt = 0;
    PD_Ctl.Flag.Bit.PD_Version = 0;
    PD_Ctl.PD_State = STA_IDLE;
    PD_Ctl.Flag.Bit.PD_Comm_Succ = 0;
    PD_Selected_PDO = 0;
    PD_Selected_mV = 0;
    PD_Selected_mA = 0;
    PD_SourcePDO_Count = 0;
    PD_GetSrcCap_Sent = 0;
    s_pd_get_src_cap_retries = 0u;
    s_pd_vbus_valid = 0u;
    s_pd_vbus_detach_count = 0u;

    PD_Source_EPR_Capable = 0;
    PD_EPR_ModeActive = 0;
    PD_EPR_ContractActive = 0;
    PD_SPR_ContractActive = 0;
    PD_EPR_SourcePDO_Count = 0;
    PD_EPR_State = EPR_ST_OFF;
    PD_EPR_FailedForAttach = 0;
    PD_EPR_GetCapSent = 0;
    PD_EPR_KeepAliveWaitAck = 0;
    PD_EPR_TimerMs = 0;
    PD_EPR_KeepAliveMs = 0;
    PD_EPR_KeepAliveAckMs = 0;
    PD_EPR_CapDataSize = 0;
    PD_EPR_LastChunk = 0;
    PD_EPR_SelectedRawPDO = 0;
    PD_SoftResetRecoveryPending = 0;
}

/*********************************************************************
 * @fn      PD_Init
 *
 * @brief   This function uses to initialize PD registers and states.
 *
 * @return  none
 */
void PD_Init( void )
{
    PD_Port_Init(PD_Rx_Buf, sizeof(PD_Rx_Buf));

    /* Initialize protocol/policy state. */
    memset(&PD_Ctl.PD_State, 0x00, sizeof(PD_CONTROL));
    Adapter_SrcCap[0] = 1;
    memcpy(&Adapter_SrcCap[1], SrcCap_5V3A_Tab, 4);
    PD_PHY_Reset();
    PD_Rx_Mode();
    s_pd_task_started = 0u;
    s_pd_task_last_ms = 0u;
    s_pd_detect_last_ms = 0u;
    s_pd_get_src_cap_retries = 0u;
    s_pd_vbus_valid = 0u;
    s_pd_vbus_detach_count = 0u;
    s_pd_vbus_mv = 0u;
}

/*********************************************************************
 * @fn      PD_Detect
 *
 * @brief   This function uses to detect CC connection.
 *
 * @return  0:No connection; 1:CC1 connection; 2:CC2 connection
 */
static UINT8 PD_Detect( void )
{
    if(PD_Ctl.Flag.Bit.Connected)
        return 0;

    return (UINT8)PD_Port_DetectAttach();
}

/*********************************************************************
 * @fn      PD_Det_Proc
 *
 * @brief   This function uses to process the return value of PD_Detect.
 *
 * @return  none
 */
static void PD_Det_Proc( void )
{
    UINT8  status;

    if( PD_Ctl.Flag.Bit.Connected )
    {
        /* WCH's SNK reference notes that detach should be judged from VBUS
         * for a bus-powered Sink.  APP feeds the INA226 bus-voltage sample
         * through PD_SetVbusMillivolts(); keep the policy decision here. */
        if(s_pd_vbus_valid && s_pd_vbus_mv < PD_VBUS_DETACH_THRESHOLD_MV)
        {
            if(s_pd_vbus_detach_count < 0xFFu)
                s_pd_vbus_detach_count++;

            if(s_pd_vbus_detach_count >= PD_VBUS_DETACH_DEBOUNCE_COUNT)
            {
                printf("[PD] Disconnect: VBUS=%u mV; clearing contract and re-arming CC detection\r\n",
                       (unsigned)s_pd_vbus_mv);
                PD_PHY_Reset();
                PD_Rx_Mode();
            }
        }
        else
        {
            s_pd_vbus_detach_count = 0u;
        }
    }
    else
    {
        /* PD disconnected, check connection */
        status = PD_Detect( );
        /* Determine connection status */
        if( status == 0 )
        {
            PD_Ctl.Det_Cnt = 0;
        }
        else
        {
            PD_Ctl.Det_Cnt++;
        }
        if( PD_Ctl.Det_Cnt >= 5 )
        {
            PD_Ctl.Det_Cnt = 0;
            PD_Ctl.Flag.Bit.Connected = 1;
            if( PD_Ctl.Flag.Bit.Stop_Det_Chk == 0 )
            {
                /* PD_Detect() already guarantees a valid Sink attach. */
                PD_Port_SelectCC((status == 2) ? PD_PORT_CC2 : PD_PORT_CC1);
                PD_Ctl.PD_State = STA_SRC_CONNECT;
                s_pd_get_src_cap_retries = 0u;
                s_pd_vbus_detach_count = 0u;
                printf("CC%d SRC Connect\r\n", status);
                PD_Ctl.PD_Comm_Timer = 0;
            }
        }
    }
}

/*********************************************************************
 * @fn      PD_Load_Header
 *
 * @brief   This function uses to load pd header packets.
 *
 * @return  none
 */
static void PD_Load_Header( UINT8 ex, UINT8 msg_type )
{
    /* Message Header
       BIT15 - Extended;
       BIT[14:12] - Number of Data Objects
       BIT[11:9] - Message ID
       BIT8 - PortPower Role/Cable Plug  0: SINK; 1: SOURCE
       BIT[7:6] - Revision, 00: V1.0; 01: V2.0; 10: V3.0;
       BIT5 - Port Data Role, 0: UFP; 1: DFP
       BIT[4:0] - Message Type
    */
    PD_Tx_Buf[ 0 ] = msg_type;
    if( PD_Ctl.Flag.Bit.PD_Role )
    {
        PD_Tx_Buf[ 0 ] |= 0x20;
    }
    if( PD_Ctl.Flag.Bit.PD_Version )
    {
        /* PD3.0 */
        PD_Tx_Buf[ 0 ] |= 0x80;
    }
    else
    {
        /* PD2.0 */
        PD_Tx_Buf[ 0 ] |= 0x40;
    }

    PD_Tx_Buf[ 1 ] = PD_Ctl.Msg_ID & 0x0E;
    if( PD_Ctl.Flag.Bit.PR_Role )
    {
        PD_Tx_Buf[ 1 ] |= 0x01;
    }
    if( ex )
    {
        PD_Tx_Buf[ 1 ] |= 0x80;
    }
}

/*********************************************************************
 * @fn      PD_Send_Handle
 *
 * @brief   This function uses to handle sending transactions.
 *
 * @return  0:success; 1:fail
 */
static UINT8 PD_Send_Handle( const UINT8 *pbuf, UINT8 len )
{
    UINT8  tx_ok;
    UINT8  cnt;
    UINT8  is_request;
    PD_Port_TxDiag tx_diag = {0};

    if( ( len % 4 ) != 0 )
    {
        return( DEF_PD_TX_FAIL );
    }
    if( len > 28 )
    {
        return( DEF_PD_TX_FAIL );
    }

    cnt = len >> 2;
    PD_Tx_Buf[ 1 ] |= ( cnt << 4 );
    for( cnt = 0; cnt != len; cnt++ )
    {
        PD_Tx_Buf[ 2 + cnt ] = pbuf[ cnt ];
    }

    /* An incoming Source message may have arrived immediately after a previous
     * foreground transaction re-enabled RX.  Never mask USBPD IRQ or start a
     * new foreground packet until its automatic GoodCRC has physically ended. */
    if(!PD_Port_WaitAutoAckComplete(1000u))
    {
        printf("[PD] TX deferred: auto-GoodCRC still in flight\r\n");
        return DEF_PD_TX_FAIL;
    }

    is_request = ((PD_Tx_Buf[0] & 0x1Fu) == DEF_TYPE_REQUEST) ? 1u : 0u;

    /* TIMING-CRITICAL: normal SOP TX, RX turnaround and GoodCRC polling are one
     * atomic PHY transaction. Never insert printf/flush/I2C/scheduler work here.
     * This matches the known-good C140 3-attempt / 250*3us sequence. */
    if(PD_Port_TransactSOP(PD_Tx_Buf, (uint8_t)(len + 2u), 3u, &tx_diag))
    {
        PD_Ctl.Msg_ID += 2;
        tx_ok = 1u;
    }
    else
    {
        tx_ok = 0u;
    }

    PD_Rx_Mode();


    if(tx_ok)
        return DEF_PD_TX_OK;

    if(is_request)
    {
        printf("[PD] failed Request: %02X %02X %02X %02X %02X %02X, MsgID=%u, ack->TX=%lu us, attempts=%u\r\n",
               PD_Tx_Buf[0], PD_Tx_Buf[1], PD_Tx_Buf[2], PD_Tx_Buf[3],
               PD_Tx_Buf[4], PD_Tx_Buf[5],
               (unsigned)((PD_Tx_Buf[1] >> 1) & 0x07u),
               (unsigned long)tx_diag.ack_to_first_tx_us,
               (unsigned)tx_diag.attempts);
    }

    {
        uint16_t ack_started = 0u;
        uint16_t ack_completed = 0u;
        PD_Port_GetAutoAckStats(&ack_started, &ack_completed);
        printf("[PD] TX diag: auto-GoodCRC started/completed=%u/%u\r\n",
               (unsigned)ack_started, (unsigned)ack_completed);
    }

    if(tx_diag.saw_hard_reset)
    {
        printf("[PD] TX diag: Source Hard Reset observed during sender-response window\r\n");
    }
    else if(tx_diag.saw_frame)
    {
        printf("[PD] TX diag: expected GoodCRC, last RX count=%u type=0x%02X stat=0x%02X\r\n",
               (unsigned)tx_diag.byte_count,
               (unsigned)tx_diag.message_type,
               (unsigned)tx_diag.pd_status);
    }
    else
    {
        printf("[PD] TX diag: no RX frame observed in GoodCRC window\r\n");
    }

    return( DEF_PD_TX_FAIL );
}

/*********************************************************************
 * @fn      PDO_Request
 *
 * @brief   This function uses to Send the specified PDO.
 *
 * @return  none
 */
void PDO_Request( UINT8 pdo_index )
{
    UINT16 Current, Voltage;
    UINT16 request_ma;
    UINT32 rdo;
    UINT8 payload[4];
    UINT8 status;

    if ((pdo_index > PDO_Len) || (pdo_index == 0))
    {
        printf("[PD] invalid SPR PDO index %u\r\n", pdo_index);
        PD_Ctl.PD_State = STA_TX_SOFTRST;
        return;
    }

    PD_PDO_Analyse(pdo_index, &Adapter_SrcCap[1], &Current, &Voltage);
    request_ma = Current;
    if(request_ma > PD_SPR_REQUEST_MAX_MA) request_ma = PD_SPR_REQUEST_MAX_MA;

    /* Fixed/Variable RDO: Object Position B31:28, No USB Suspend B24,
     * EPR Mode Capable B22, operating/max current in 10mA units.
     * We deliberately leave B23=0 so the Source must use chunked Extended
     * Messages; PD_Rx_Buf is sized for one 26-byte Extended chunk. */
    rdo = ((UINT32)(pdo_index & 0x0Fu) << 28) | (1UL << 24);
#if PD_EPR_ENABLE
    if(PD_Source_EPR_Capable && !PD_EPR_FailedForAttach)
        rdo |= (1UL << 22);
#endif
    rdo |= ((UINT32)((request_ma / 10u) & 0x03FFu) << 10);
    rdo |=  (UINT32)((request_ma / 10u) & 0x03FFu);

    payload[0] = (UINT8)(rdo);
    payload[1] = (UINT8)(rdo >> 8);
    payload[2] = (UINT8)(rdo >> 16);
    payload[3] = (UINT8)(rdo >> 24);

    PD_Selected_PDO = pdo_index;
    PD_Selected_mV = Voltage;
    PD_Selected_mA = request_ma;
    PD_Ctl.ReqPDO_Idx = pdo_index;

    /* Do not print/flush between Source_Capabilities GoodCRC and Request.
     * The Source's SenderResponseTimer is already running in this interval. */
    PD_Load_Header(0x00, DEF_TYPE_REQUEST);
    status = PD_Send_Handle(payload, 4);

    if(status == DEF_PD_TX_OK)
        PD_Ctl.PD_State = STA_RX_ACCEPT_WAIT;

    printf("[PD] SPR Request PDO%u: %u mV, %u mA%s\r\n",
           pdo_index, Voltage, request_ma,
           (rdo & (1UL << 22)) ? ", EPR-capable RDO" : "");

    if(status != DEF_PD_TX_OK && PD_Port_HardResetPending())
    {
        printf("[PD] SPR Request terminated by Source Hard Reset\r\n");
        PD_Ctl.PD_State = STA_IDLE;
    }
    else if(status != DEF_PD_TX_OK)
    {
        printf("[PD] SPR Request TX/GoodCRC failed; scheduling Soft Reset\r\n");
        PD_Ctl.PD_State = STA_TX_SOFTRST;
    }

    PD_Ctl.PD_Comm_Timer = 0;
    PD_Ctl.Flag.Bit.PD_Comm_Succ = 1;
}

/*********************************************************************
 * @fn      PD_Save_Adapter_SrcCap
 *
 * @brief   This function uses to save the adapter SrcCap information.
 *
 * @return  none
 */
static void PD_Save_Adapter_SrcCap( void )
{
    UINT8  i, len;

    /* Calculate the number of NDO's (Number of Data Objects) in the Message Header. */
    len = ( ( PD_Rx_Buf[ 1 ] >> 4 ) & 0x07 );

    /* Preserve the exact Source_Capabilities payload for terminal diagnostics
     * before the original WCH sample trims APDOs for its fixed-PDO request path. */
    PD_SourcePDO_Count = len;
    memcpy(PD_SourcePDO_Raw, &PD_Rx_Buf[2], (len << 2));

    /* Remove the PPS section */
    for( i = 0; i < len; i++ )
    {
        if( ( PD_Rx_Buf[ 2 + ( i << 2 ) + 3 ] & 0xC0 ) == 0xC0 )
        {
            break;
        }
    }

    PDO_Len = i;

    /* Modify SrcCap information */
       /* BIT[31:30] - Fixed Supply */
       /* BIT29 - Dual-Role Power */
       /* BIT28 - USB Suspend Power */
       /* BIT27 - Unconstrained Power */
       /* BIT26 - USB Communications */
       /* BIT25 - Dual-Role Data */
       /* BIT24 - Unchunked Extended Message Supported */
       /* BIT23 - EPR Mode Capable */
       /* BIT22 - Reserved,shall be set to zero */
       /* BIT[21:20] - Peak Current */
       /* BIT[19:10] - Voltage in 50mV units */
       /* BIT[9:0] - Maximum Current in 10mA units */
    /* Keep the received PDO bytes unchanged so printed raw PDO values remain exact. */

    /* Save the adapter's SrcCap information */
    PD_Rx_Buf[ 1 ] &= 0x8F;
    PD_Rx_Buf[ 1 ] |= i << 4;
    Adapter_SrcCap[ 0 ] = i;
    memcpy( &Adapter_SrcCap[ 1 ], &PD_Rx_Buf[ 2 ], ( i << 2 ) );
}

/*********************************************************************
 * @fn      PD_PDO_Analyse
 *
 * @brief   This function uses to analyse PDO's voltage and current.
 *
 * @return  none
 */
static void PD_PDO_Analyse( UINT8 pdo_idx, UINT8 *srccap, UINT16 *current, UINT16 *voltage )
{
    UINT32 temp32;

    temp32 = srccap[ (  ( pdo_idx - 1 ) << 2 ) + 0 ] +
                        ( (UINT32)srccap[ ( ( pdo_idx - 1 ) << 2 ) + 1 ] << 8 ) +
                        ( (UINT32)srccap[ ( ( pdo_idx - 1 ) << 2 ) + 2 ] << 16 );

    /* Calculation of current values */
    if( current != NULL )
    {
        *current = ( temp32 & 0x000003FF ) * 10;
    }

    /* Calculation of voltage values */
    if( voltage != NULL )
    {
        temp32 = temp32 >> 10;
        *voltage = ( temp32 & 0x000003FF ) * 50;
    }
}


/*********************************************************************
 * @fn      PD_Select_Highest_Fixed_PDO
 *
 * @brief   Select highest fixed PDO not exceeding board policy limit.
 *          Tie-breaker: higher available current.
 *
 * @return  PDO index (1..PDO_Len), 0 if none is usable.
 */
static UINT8 PD_Select_Highest_Fixed_PDO(void)
{
    UINT8 i;
    UINT8 best = 0;
    UINT16 best_mv = 0;
    UINT16 best_ma = 0;

    for(i = 1; i <= PDO_Len; i++)
    {
        UINT8 *p = &Adapter_SrcCap[1 + ((i - 1) << 2)];
        UINT32 raw = (UINT32)p[0] |
                     ((UINT32)p[1] << 8) |
                     ((UINT32)p[2] << 16) |
                     ((UINT32)p[3] << 24);
        UINT8 supply_type = (UINT8)(raw >> 30);
        UINT16 mv;
        UINT16 ma;

        if(supply_type != 0) continue;   /* fixed PDO only */
        mv = (UINT16)(((raw >> 10) & 0x3FFu) * 50u);
        ma = (UINT16)((raw & 0x3FFu) * 10u);
        if(mv > PD_REQUEST_MAX_FIXED_MV) continue;

        if((mv > best_mv) || ((mv == best_mv) && (ma > best_ma)))
        {
            best = i;
            best_mv = mv;
            best_ma = ma;
        }
    }

    PD_Selected_PDO = best;
    PD_Selected_mV = best_mv;
    PD_Selected_mA = best_ma;
    PD_Ctl.ReqPDO_Idx = best;
    return best;
}

static void PD_Print_Source_PDOs(void)
{
    UINT8 i;
    printf("[PD] Source_Capabilities: %u PDO(s); fixed-request policy max %u mV:\r\n",
           PD_SourcePDO_Count, (unsigned)PD_REQUEST_MAX_FIXED_MV);

    for(i = 1; i <= PD_SourcePDO_Count; i++)
    {
        UINT8 *p = &PD_SourcePDO_Raw[(i - 1) << 2];
        UINT32 raw = (UINT32)p[0] |
                     ((UINT32)p[1] << 8) |
                     ((UINT32)p[2] << 16) |
                     ((UINT32)p[3] << 24);
        UINT8 supply_type = (UINT8)(raw >> 30);

        if(supply_type == 0)
        {
            UINT16 mv = (UINT16)(((raw >> 10) & 0x3FFu) * 50u);
            UINT16 ma = (UINT16)((raw & 0x3FFu) * 10u);
            printf("  PDO%u raw=%08lx FIXED %u mV %u mA%s\r\n",
                   i, (unsigned long)raw, mv, ma,
                   (i == PD_Selected_PDO) ? "  <REQUEST>" : "");
        }
        else if(supply_type == 1)
        {
            UINT16 min_mv = (UINT16)(((raw >> 10) & 0x3FFu) * 50u);
            UINT16 max_mv = (UINT16)(((raw >> 20) & 0x3FFu) * 50u);
            UINT32 max_mw = (UINT32)(raw & 0x3FFu) * 250u;
            printf("  PDO%u raw=%08lx BATTERY %u-%u mV, %lu mW\r\n",
                   i, (unsigned long)raw, min_mv, max_mv, (unsigned long)max_mw);
        }
        else if(supply_type == 2)
        {
            UINT16 min_mv = (UINT16)(((raw >> 10) & 0x3FFu) * 50u);
            UINT16 max_mv = (UINT16)(((raw >> 20) & 0x3FFu) * 50u);
            UINT16 ma = (UINT16)((raw & 0x3FFu) * 10u);
            printf("  PDO%u raw=%08lx VARIABLE %u-%u mV, %u mA\r\n",
                   i, (unsigned long)raw, min_mv, max_mv, ma);
        }
        else
        {
            UINT8 apdo_type = (UINT8)((raw >> 28) & 0x03u);
            if(apdo_type == 0)
            {
                UINT16 min_mv = (UINT16)(((raw >> 8) & 0xFFu) * 100u);
                UINT16 max_mv = (UINT16)(((raw >> 17) & 0xFFu) * 100u);
                UINT16 ma = (UINT16)((raw & 0x7Fu) * 50u);
                printf("  PDO%u raw=%08lx PPS_APDO %u-%u mV, %u mA (not requested)\r\n",
                       i, (unsigned long)raw, min_mv, max_mv, ma);
            }
            else
            {
                printf("  PDO%u raw=%08lx APDO subtype=%u (not requested)\r\n",
                       i, (unsigned long)raw, apdo_type);
            }
        }
    }
}

static UINT32 PD_ReadU32LE(const UINT8 *p)
{
    return (UINT32)p[0] |
           ((UINT32)p[1] << 8) |
           ((UINT32)p[2] << 16) |
           ((UINT32)p[3] << 24);
}

static void PD_WriteU32LE(UINT8 *p, UINT32 v)
{
    p[0] = (UINT8)v;
    p[1] = (UINT8)(v >> 8);
    p[2] = (UINT8)(v >> 16);
    p[3] = (UINT8)(v >> 24);
}

static UINT8 PD_Send_Extended(UINT8 msg_type, UINT16 ext_header,
                              const UINT8 *data, UINT8 data_len)
{
    UINT8 payload[28];
    UINT8 total = (UINT8)(data_len + 2u);
    UINT8 padded = (UINT8)((total + 3u) & 0xFCu);

    if(padded == 0) padded = 4;
    if(padded > sizeof(payload)) return DEF_PD_TX_FAIL;

    memset(payload, 0, padded);
    payload[0] = (UINT8)ext_header;
    payload[1] = (UINT8)(ext_header >> 8);
    if(data_len && data) memcpy(&payload[2], data, data_len);

    PD_Load_Header(0x01, msg_type);
    return PD_Send_Handle(payload, padded);
}

static UINT8 PD_Send_Extended_Control(UINT8 ctrl_type)
{
    UINT8 ecdb[2];
    UINT16 ext_header;

    /* We advertise no unchunked-extended support in the RDO, so use the
     * chunked format even though this ECDB is only two bytes. */
    ext_header = (UINT16)(0x8000u | 2u);   /* Chunked=1, Chunk#0, DataSize=2 */
    ecdb[0] = ctrl_type;
    ecdb[1] = 0;
    return PD_Send_Extended(PD_EXT_TYPE_EXTENDED_CONTROL, ext_header, ecdb, 2);
}

static UINT8 PD_Send_Extended_Chunk_Request(UINT8 msg_type, UINT8 chunk_number)
{
    UINT16 ext_header;
    ext_header = (UINT16)(0x8000u | 0x0400u |
                          ((UINT16)(chunk_number & 0x0Fu) << 11));
    /* RequestChunk=1 requires DataSize=0.  Two padding bytes are emitted by
     * PD_Send_Extended so NDO=1. */
    return PD_Send_Extended(msg_type, ext_header, NULL, 0);
}

static UINT8 PD_Send_EPR_Mode(UINT8 action, UINT8 data)
{
    UINT8 p[4];
    UINT32 eprmdo = ((UINT32)action << 24) | ((UINT32)data << 16);
    PD_WriteU32LE(p, eprmdo);
    PD_Load_Header(0x00, DEF_TYPE_EPR_MODE);
    return PD_Send_Handle(p, 4);
}

static void PD_LocalProtocolRecover(const char *reason)
{
    PD_ProtocolRecoveryCount++;
    if(reason)
        printf("[PD] protocol recovery #%u: %s; Hard Reset suppressed on VBUS-powered board\r\n",
               (unsigned)PD_ProtocolRecoveryCount, reason);

    PD_PHY_Reset();
    PD_EPR_FailedForAttach = 1u;
    PD_Ctl.Flag.Bit.Connected = 0u;
    PD_Ctl.Det_Cnt = 0u;
    PD_Ctl.PD_Comm_Timer = 0u;
    PD_Ctl.Flag.Bit.Stop_Det_Chk = 0u;
    PD_GetSrcCap_Sent = 0u;
    PD_Rx_Mode();
}

static void PD_EPR_Fallback(const char *reason)
{
    if(reason) printf("[PD] EPR fallback: %s; keeping SPR contract\r\n", reason);
    PD_EPR_FailedForAttach = 1;
    PD_EPR_State = EPR_ST_OFF;
    PD_EPR_ModeActive = 0;
    PD_EPR_ContractActive = 0;
    PD_EPR_GetCapSent = 0;
    PD_EPR_KeepAliveWaitAck = 0;
    PD_EPR_TimerMs = 0;
    PD_EPR_KeepAliveMs = 0;
}

static void PD_EPR_Exit_To_SPR(const char *reason)
{
    if(reason) printf("[PD] EPR exit: %s\r\n", reason);
    (void)PD_Send_EPR_Mode(PD_EPR_MODE_EXIT, 0);
    PD_EPR_Fallback(NULL);
}

static void PD_Print_EPR_Source_PDOs(void)
{
    UINT8 i;
    printf("[PD] EPR_Source_Capabilities: %u PDO slots (%u bytes)\r\n",
           PD_EPR_SourcePDO_Count, PD_EPR_CapDataSize);

    for(i = 1; i <= PD_EPR_SourcePDO_Count; i++)
    {
        UINT32 raw = PD_ReadU32LE(&PD_EPR_SourcePDO_Raw[(i - 1u) << 2]);
        UINT8 supply_type = (UINT8)(raw >> 30);

        if(raw == 0)
        {
            printf("  PDO%u raw=00000000 <empty>\r\n", i);
        }
        else if(supply_type == 0)
        {
            UINT16 mv = (UINT16)(((raw >> 10) & 0x3FFu) * 50u);
            UINT16 ma = (UINT16)((raw & 0x3FFu) * 10u);
            printf("  PDO%u raw=%08lx FIXED %u mV %u mA%s\r\n",
                   i, (unsigned long)raw, mv, ma,
                   (i == PD_Selected_PDO) ? "  <EPR TARGET>" : "");
        }
        else if(supply_type == 1)
        {
            UINT16 min_mv = (UINT16)(((raw >> 10) & 0x3FFu) * 50u);
            UINT16 max_mv = (UINT16)(((raw >> 20) & 0x3FFu) * 50u);
            UINT32 max_mw = (UINT32)(raw & 0x3FFu) * 250u;
            printf("  PDO%u raw=%08lx BATTERY %u-%u mV %lu mW\r\n",
                   i, (unsigned long)raw, min_mv, max_mv, (unsigned long)max_mw);
        }
        else if(supply_type == 2)
        {
            UINT16 min_mv = (UINT16)(((raw >> 10) & 0x3FFu) * 50u);
            UINT16 max_mv = (UINT16)(((raw >> 20) & 0x3FFu) * 50u);
            UINT16 ma = (UINT16)((raw & 0x3FFu) * 10u);
            printf("  PDO%u raw=%08lx VARIABLE %u-%u mV %u mA\r\n",
                   i, (unsigned long)raw, min_mv, max_mv, ma);
        }
        else
        {
            UINT8 apdo_type = (UINT8)((raw >> 28) & 0x03u);
            if(apdo_type == 0)
            {
                UINT16 min_mv = (UINT16)(((raw >> 8) & 0xFFu) * 100u);
                UINT16 max_mv = (UINT16)(((raw >> 17) & 0xFFu) * 100u);
                UINT16 ma = (UINT16)((raw & 0x7Fu) * 50u);
                printf("  PDO%u raw=%08lx PPS %u-%u mV %u mA\r\n",
                       i, (unsigned long)raw, min_mv, max_mv, ma);
            }
            else if(apdo_type == 1)
            {
                UINT16 min_mv = (UINT16)(((raw >> 8) & 0xFFu) * 100u);
                UINT16 max_mv = (UINT16)(((raw >> 17) & 0x1FFu) * 100u);
                UINT16 pdp_w = (UINT16)(raw & 0xFFu);
                printf("  PDO%u raw=%08lx AVS %u-%u mV PDP=%u W\r\n",
                       i, (unsigned long)raw, min_mv, max_mv, pdp_w);
            }
            else
            {
                printf("  PDO%u raw=%08lx APDO subtype=%u\r\n",
                       i, (unsigned long)raw, apdo_type);
            }
        }
    }
}

static UINT8 PD_Select_EPR_28V_Fixed(void)
{
    UINT8 i;
    UINT8 best = 0;
    UINT16 best_ma = 0;

    for(i = 8; i <= PD_EPR_SourcePDO_Count; i++)
    {
        UINT32 raw = PD_ReadU32LE(&PD_EPR_SourcePDO_Raw[(i - 1u) << 2]);
        if((raw >> 30) == 0)
        {
            UINT16 mv = (UINT16)(((raw >> 10) & 0x3FFu) * 50u);
            UINT16 ma = (UINT16)((raw & 0x3FFu) * 10u);
            if(mv == PD_EPR_TARGET_MV && ma > best_ma)
            {
                best = i;
                best_ma = ma;
                PD_EPR_SelectedRawPDO = raw;
            }
        }
    }

    if(best)
    {
        PD_Selected_PDO = best;
        PD_Selected_mV = PD_EPR_TARGET_MV;
        PD_Selected_mA = best_ma;
        if(PD_Selected_mA > PD_EPR_REQUEST_MAX_MA)
            PD_Selected_mA = PD_EPR_REQUEST_MAX_MA;
        PD_Ctl.ReqPDO_Idx = best;
    }
    return best;
}

static UINT8 PD_Send_EPR_Request_Fixed(UINT8 pdo_index)
{
    UINT8 payload[8];
    UINT32 rdo;
    UINT16 units = (UINT16)(PD_Selected_mA / 10u);

    rdo = ((UINT32)(pdo_index & 0x0Fu) << 28) |
          (1UL << 24) |                 /* No USB Suspend */
          (1UL << 22) |                 /* EPR Mode Capable */
          ((UINT32)(units & 0x03FFu) << 10) |
          (UINT32)(units & 0x03FFu);

    PD_WriteU32LE(&payload[0], rdo);
    PD_WriteU32LE(&payload[4], PD_EPR_SelectedRawPDO); /* exact PDO copy */

    PD_Load_Header(0x00, DEF_TYPE_EPR_REQUEST);
    return PD_Send_Handle(payload, 8);
}

static void PD_EPR_Capabilities_Complete(void)
{
    UINT8 target;
    UINT8 status;

    if((PD_EPR_CapDataSize == 0) || ((PD_EPR_CapDataSize & 3u) != 0))
    {
        PD_EPR_Exit_To_SPR("invalid EPR Source_Capabilities length");
        return;
    }

    PD_EPR_SourcePDO_Count = (UINT8)(PD_EPR_CapDataSize >> 2);
    target = PD_Select_EPR_28V_Fixed();

    /* Print before the EPR_Request.  After transmitting a request, the Source
     * may answer Accept immediately; avoiding printf after TX prevents the
     * next RX packet from being overwritten/lost by the old WCH main-loop
     * receive pattern. */
    PD_Print_EPR_Source_PDOs();

    if(target == 0)
    {
        PD_EPR_Exit_To_SPR("no 28V Fixed PDO");
        return;
    }

    printf("[PD] EPR Request PDO%u: %u mV, %u mA\r\n",
           target, PD_Selected_mV, PD_Selected_mA);
    status = PD_Send_EPR_Request_Fixed(target);
    if(status == DEF_PD_TX_OK)
    {
        PD_EPR_State = EPR_ST_WAIT_REQUEST_ACCEPT;
        PD_Ctl.PD_State = STA_RX_ACCEPT_WAIT;
        PD_Ctl.PD_Comm_Timer = 0;
        PD_EPR_TimerMs = 0;
    }
    else
    {
        PD_EPR_Exit_To_SPR("EPR_Request TX failed");
    }
}

static void PD_Handle_EPR_Source_Capabilities(void)
{
    UINT16 ext_header;
    UINT16 data_size;
    UINT16 offset;
    UINT16 remain;
    UINT8 ndo;
    UINT8 chunked;
    UINT8 request_chunk;
    UINT8 chunk_number;
    UINT8 available;
    UINT8 copy_len;

    ext_header = (UINT16)PD_Rx_Buf[2] | ((UINT16)PD_Rx_Buf[3] << 8);
    data_size = (UINT16)(ext_header & 0x01FFu);
    request_chunk = (UINT8)((ext_header >> 10) & 1u);
    chunk_number = (UINT8)((ext_header >> 11) & 0x0Fu);
    chunked = (UINT8)((ext_header >> 15) & 1u);
    ndo = (UINT8)((PD_Rx_Buf[1] >> 4) & 0x07u);

    if(request_chunk) return;
    if(ndo == 0 || (UINT16)ndo * 4u < 2u) return;
    available = (UINT8)((UINT16)ndo * 4u - 2u);

    if(data_size == 0 || data_size > PD_EPR_CAP_BUFFER_SIZE)
    {
        PD_EPR_Exit_To_SPR("EPR Source_Capabilities too large");
        return;
    }

    if(!chunked)
    {
        if(data_size > available)
        {
            PD_EPR_Exit_To_SPR("unchunked EPR Source_Capabilities exceeds RX buffer");
            return;
        }
        memset(PD_EPR_SourcePDO_Raw, 0, sizeof(PD_EPR_SourcePDO_Raw));
        memcpy(PD_EPR_SourcePDO_Raw, &PD_Rx_Buf[4], data_size);
        PD_EPR_CapDataSize = data_size;
        PD_EPR_Capabilities_Complete();
        return;
    }

    if(chunk_number == 0)
    {
        memset(PD_EPR_SourcePDO_Raw, 0, sizeof(PD_EPR_SourcePDO_Raw));
        PD_EPR_CapDataSize = data_size;
        PD_EPR_LastChunk = 0;
    }
    else
    {
        if(PD_EPR_CapDataSize != data_size || chunk_number != (UINT8)(PD_EPR_LastChunk + 1u))
        {
            PD_EPR_Exit_To_SPR("unexpected EPR chunk sequence");
            return;
        }
        PD_EPR_LastChunk = chunk_number;
    }

    offset = (UINT16)chunk_number * PD_EXT_CHUNK_DATA_MAX;
    if(offset >= data_size || offset >= PD_EPR_CAP_BUFFER_SIZE)
    {
        PD_EPR_Exit_To_SPR("invalid EPR chunk offset");
        return;
    }

    remain = (UINT16)(data_size - offset);
    copy_len = (remain > PD_EXT_CHUNK_DATA_MAX) ? PD_EXT_CHUNK_DATA_MAX : (UINT8)remain;
    if(copy_len > available) copy_len = available;
    memcpy(&PD_EPR_SourcePDO_Raw[offset], &PD_Rx_Buf[4], copy_len);

    if((UINT16)(offset + copy_len) >= data_size)
    {
        PD_EPR_Capabilities_Complete();
    }
    else
    {
        UINT8 next_chunk = (UINT8)(chunk_number + 1u);
        if(PD_Send_Extended_Chunk_Request(PD_EXT_TYPE_EPR_SOURCE_CAP, next_chunk) != DEF_PD_TX_OK)
            PD_EPR_Exit_To_SPR("EPR chunk request TX failed");
    }
}

static void PD_Handle_Extended_Message(UINT8 msg_type)
{
    if(msg_type == PD_EXT_TYPE_EPR_SOURCE_CAP)
    {
        PD_EPR_GetCapSent = 1;
        PD_EPR_TimerMs = 0;
        PD_Handle_EPR_Source_Capabilities();
    }
    else if(msg_type == PD_EXT_TYPE_EXTENDED_CONTROL)
    {
        UINT16 ext_header = (UINT16)PD_Rx_Buf[2] | ((UINT16)PD_Rx_Buf[3] << 8);
        UINT16 data_size = (UINT16)(ext_header & 0x01FFu);
        UINT8 request_chunk = (UINT8)((ext_header >> 10) & 1u);
        if(!request_chunk && data_size >= 2u)
        {
            UINT8 ctrl_type = PD_Rx_Buf[4];
            if(ctrl_type == PD_EXT_CTRL_EPR_KEEPALIVE_ACK)
            {
                PD_EPR_KeepAliveWaitAck = 0;
                PD_EPR_KeepAliveAckMs = 0;
            }
        }
    }
}

static void PD_Handle_EPR_Mode_Message(void)
{
    UINT32 eprmdo = PD_ReadU32LE(&PD_Rx_Buf[2]);
    UINT8 action = (UINT8)(eprmdo >> 24);
    UINT8 data = (UINT8)(eprmdo >> 16);

    if(action == PD_EPR_MODE_ENTER_ACK)
    {
        if(PD_EPR_State == EPR_ST_WAIT_ENTER_ACK)
        {
            printf("[PD] EPR Mode: Enter Acknowledged\r\n");
            PD_EPR_State = EPR_ST_WAIT_ENTER_SUCCESS;
            PD_EPR_TimerMs = 0;
        }
    }
    else if(action == PD_EPR_MODE_ENTER_SUCCESS)
    {
        if(PD_EPR_State == EPR_ST_WAIT_ENTER_ACK ||
           PD_EPR_State == EPR_ST_WAIT_ENTER_SUCCESS)
        {
            printf("[PD] EPR Mode: Enter Succeeded\r\n");
            PD_EPR_ModeActive = 1;
            PD_EPR_State = EPR_ST_WAIT_SOURCE_CAP;
            PD_EPR_TimerMs = 0;
            PD_EPR_GetCapSent = 0;
        }
    }
    else if(action == PD_EPR_MODE_ENTER_FAILED)
    {
        const char *reason = "unknown";
        if(data == 1) reason = "cable not EPR capable";
        else if(data == 2) reason = "Source failed to become VCONN Source";
        else if(data == 3) reason = "EPR-capable bit missing in RDO";
        else if(data == 4) reason = "Source unable to enter EPR now";
        else if(data == 5) reason = "Source PDO not EPR capable";
        PD_EPR_Fallback(reason);
    }
    else if(action == PD_EPR_MODE_EXIT)
    {
        PD_EPR_Fallback("Source exited EPR Mode");
    }
}

/*********************************************************************
 * @fn      PD_Main_Proc
 *
 * @brief   This function uses to process PD status.
 *
 * @return  none
 */
static void PD_Main_Proc( )
{
    UINT8 status;
    UINT8 pd_header;
    UINT8 var;

    /* Hardware IRQs are terminated inside the BSP and surfaced as events. */
    if(PD_Port_HardResetPending())
    {
        /* A Hard Reset terminates the whole protocol session.  Do not let a
         * message event captured just before/alongside IF_RX_RESET be parsed
         * after the session state has been cleared: that can otherwise turn a
         * stale PS_RDY into a fake PDO0 contract. */
        PD_Port_ClearHardResetEvent();
        PD_Port_ClearMessageEvent();
        printf("[PD] Source Hard Reset received; contract invalid, re-arming Sink\r\n");
        PD_PHY_Reset();
        PD_Rx_Mode();
        return;
    }

    PD_Ctl.PD_BusIdle_Timer += Tmr_Ms_Dlt;

    if(PD_EPR_State != EPR_ST_OFF)
        PD_EPR_TimerMs = (UINT16)(PD_EPR_TimerMs + Tmr_Ms_Dlt);

    if(PD_EPR_State == EPR_ST_ACTIVE)
    {
        PD_EPR_KeepAliveMs = (UINT16)(PD_EPR_KeepAliveMs + Tmr_Ms_Dlt);
        if(PD_EPR_KeepAliveWaitAck)
            PD_EPR_KeepAliveAckMs = (UINT16)(PD_EPR_KeepAliveAckMs + Tmr_Ms_Dlt);

        if(PD_EPR_KeepAliveWaitAck && PD_EPR_KeepAliveAckMs > PD_EPR_KEEPALIVE_ACK_TIMEOUT_MS)
        {
            printf("[PD] EPR KeepAlive ACK timeout; retrying\r\n");
            PD_EPR_KeepAliveWaitAck = 0;
            PD_EPR_KeepAliveAckMs = 0;
        }

        if(!PD_EPR_KeepAliveWaitAck && PD_EPR_KeepAliveMs >= PD_EPR_KEEPALIVE_PERIOD_MS)
        {
            if(PD_Send_Extended_Control(PD_EXT_CTRL_EPR_KEEPALIVE) == DEF_PD_TX_OK)
            {
                PD_EPR_KeepAliveWaitAck = 1;
                PD_EPR_KeepAliveMs = 0;
                PD_EPR_KeepAliveAckMs = 0;
            }
        }
    }
    else if((PD_EPR_State == EPR_ST_WAIT_ENTER_ACK ||
             PD_EPR_State == EPR_ST_WAIT_ENTER_SUCCESS) &&
            PD_EPR_TimerMs > PD_EPR_ENTER_TIMEOUT_MS)
    {
        PD_EPR_Fallback("EPR Mode entry timeout");
    }
    else if(PD_EPR_State == EPR_ST_WAIT_SOURCE_CAP)
    {
        if(!PD_EPR_GetCapSent && PD_EPR_TimerMs > PD_EPR_GET_CAP_DELAY_MS)
        {
            PD_EPR_GetCapSent = 1;
            if(PD_Send_Extended_Control(PD_EXT_CTRL_EPR_GET_SOURCE_CAP) == DEF_PD_TX_OK)
            {
                printf("[PD] EPR_Get_Source_Cap sent\r\n");
                PD_EPR_TimerMs = 0;
            }
            else
            {
                PD_EPR_Exit_To_SPR("EPR_Get_Source_Cap TX failed");
            }
        }
        else if(PD_EPR_GetCapSent && PD_EPR_TimerMs > PD_EPR_SOURCE_CAP_TIMEOUT_MS)
        {
            PD_EPR_Exit_To_SPR("EPR_Source_Capabilities timeout");
        }
    }

    switch( PD_Ctl.PD_State )
    {
        case STA_DISCONNECT:
            printf("Disconnect\r\n");
            PD_PHY_Reset( );
            break;

        case STA_SRC_CONNECT:
            PD_Ctl.PD_Comm_Timer += Tmr_Ms_Dlt;

            /* A Source normally sends Source_Capabilities after Attach.  If
             * our PHY joined late (for example after holding RST), explicitly
             * ask for them.  Previous code only tried once, then called
             * PD_PHY_Reset() without clearing Connected, which could leave the
             * state machine permanently stuck in STA_IDLE. */
            if(PD_Ctl.PD_Comm_Timer >= PD_GET_SOURCE_CAP_RETRY_MS)
            {
                if(s_pd_get_src_cap_retries < PD_GET_SOURCE_CAP_MAX_RETRIES)
                {
                    s_pd_get_src_cap_retries++;
                    printf("[PD] no Source_Capabilities; Get_Source_Cap retry %u/%u\r\n",
                           (unsigned)s_pd_get_src_cap_retries,
                           (unsigned)PD_GET_SOURCE_CAP_MAX_RETRIES);
                    PD_Load_Header(0x00, DEF_TYPE_GET_SRC_CAP);
                    status = PD_Send_Handle(NULL, 0);
                    PD_Ctl.PD_Comm_Timer = 0;

                    if(status == DEF_PD_TX_OK)
                    {
                        PD_GetSrcCap_Sent = 1u;
                        printf("[PD] Get_Source_Cap sent; waiting for Source_Capabilities\r\n");
                    }
                    else
                    {
                        printf("[PD] Get_Source_Cap TX failed; re-arming physical attach detection\r\n");
                        PD_PHY_Reset();
                        PD_Rx_Mode();
                    }
                }
                else
                {
                    printf("[PD] Source_Capabilities timeout after %u retries; re-arming attach detection\r\n",
                           (unsigned)PD_GET_SOURCE_CAP_MAX_RETRIES);
                    PD_PHY_Reset();
                    PD_Rx_Mode();
                }
            }
            break;

        case STA_RX_ACCEPT_WAIT:
        case STA_RX_PS_RDY_WAIT:
            PD_Ctl.PD_Comm_Timer += Tmr_Ms_Dlt;
            if(PD_Ctl.PD_Comm_Timer > 499)
            {
                printf("[PD] response timeout in state %u after %u ms; trying Soft Reset\r\n",
                       (unsigned)PD_Ctl.PD_State, (unsigned)PD_Ctl.PD_Comm_Timer);
                PD_Ctl.Flag.Bit.Stop_Det_Chk = 0;
                PD_SoftResetRecoveryPending = 1u;
                PD_Ctl.PD_State = STA_TX_SOFTRST;
                PD_Ctl.PD_Comm_Timer = 0;
            }
            break;

        case STA_RX_PS_RDY:
            PD_Ctl.PD_State = STA_IDLE;
            break;

        case STA_TX_SOFTRST:
            PD_Load_Header(0x00, DEF_TYPE_SOFT_RESET);
            status = PD_Send_Handle(NULL, 0);
            PD_Ctl.PD_Comm_Timer = 0;
            if(status == DEF_PD_TX_OK)
            {
                PD_SoftResetRecoveryPending = 1u;
                PD_Ctl.PD_State = STA_RX_ACCEPT_WAIT;
                printf("[PD] Soft Reset sent; waiting for Accept\r\n");
            }
            else
            {
#if PD_VBUS_POWERED_SUPPRESS_HARD_RESET
                PD_SoftResetRecoveryPending = 0u;
                PD_LocalProtocolRecover("Soft Reset TX failed");
#else
                PD_Ctl.PD_State = STA_TX_HRST;
#endif
            }
            break;

        case STA_TX_HRST:
#if PD_VBUS_POWERED_SUPPRESS_HARD_RESET
            PD_SoftResetRecoveryPending = 0u;
            PD_LocalProtocolRecover("Hard Reset requested by policy engine");
#else
            PD_Ctl.Flag.Bit.Stop_Det_Chk = 1;
            PD_SPR_ContractActive = 0;
            PD_EPR_ContractActive = 0;
            PD_Port_SendHardReset();
            PD_Rx_Mode();
            PD_Ctl.PD_State = STA_IDLE;
            PD_Ctl.PD_Comm_Timer = 0;
            printf("[PD] Hard Reset sent; previous contract invalid\r\n");
            PD_EPR_Fallback(NULL);
#endif
            break;

        default:
            break;
    }

    if(PD_Port_MessagePending())
    {
        UINT8 is_extended;

        /* Consume the current IRQ event. If another packet arrives while a
         * response helper has RX enabled, the BSP raises a fresh event. */
        PD_Port_ClearMessageEvent();
        PD_Ctl.Adapter_Idle_Cnt = 0;
        pd_header = PD_Rx_Buf[0] & 0x1F;
        is_extended = (PD_Rx_Buf[1] & 0x80u) ? 1u : 0u;

        /* Track the partner revision from every received SOP message.  EPR
         * still uses the Rev3.x Message Header encoding (10b). */
        if(((PD_Rx_Buf[0] >> 6) & 0x03u) >= DEF_PD_REVISION_30)
            PD_Ctl.Flag.Bit.PD_Version = 1;

        if(is_extended)
        {
            PD_Handle_Extended_Message(pd_header);
        }
        else
        {
            switch(pd_header)
            {
                case DEF_TYPE_SRC_CAP:
                {
                    UINT32 pdo1;
                    PD_GetSrcCap_Sent = 1;
                    s_pd_get_src_cap_retries = 0u;
                    Delay_Ms(5);
                    PD_Ctl.Flag.Bit.Stop_Det_Chk = 0;
                    PD_Save_Adapter_SrcCap();

                    pdo1 = (PD_SourcePDO_Count > 0) ? PD_ReadU32LE(&PD_SourcePDO_Raw[0]) : 0;
                    PD_Source_EPR_Capable =
                        (PD_Ctl.Flag.Bit.PD_Version && (pdo1 & (1UL << 23))) ? 1u : 0u;

                    var = PD_Select_Highest_Fixed_PDO();
                    if(var == 0)
                    {
                        var = PDO_INDEX_1;
                        PD_Selected_PDO = var;
                    }

#if PD_EPR_ENABLE
                    if(PD_Source_EPR_Capable && !PD_EPR_FailedForAttach)
                    {
                        PD_EPR_State = EPR_ST_SPR_NEGOTIATING;
                        PD_EPR_TimerMs = 0;
                    }
#endif

                    /* Request first; diagnostics come afterwards so they cannot
                     * consume the Source's tSenderResponse budget. */
                    PDO_Request(var);

                    printf("\r\n");
                    PD_Print_Source_PDOs();
#if PD_EPR_ENABLE
                    if(PD_Source_EPR_Capable && !PD_EPR_FailedForAttach)
                        printf("[PD] Source advertises EPR capability; establish SPR contract first\r\n");
                    else if(!PD_Source_EPR_Capable)
                        printf("[PD] Source is not EPR capable; staying in SPR\r\n");
#endif
                    break;
                }

                case DEF_TYPE_ACCEPT:
                    if(PD_Ctl.PD_State != STA_RX_ACCEPT_WAIT)
                    {
                        printf("[PD] ignored unexpected Accept in state %u\r\n",
                               (unsigned)PD_Ctl.PD_State);
                        break;
                    }

                    if(PD_SoftResetRecoveryPending)
                    {
                        /* Soft Reset terminates the old protocol sequence.  After
                         * Accept both sides restart their message-ID counters; ask
                         * for Source_Capabilities again without dropping VBUS. */
                        PD_SoftResetRecoveryPending = 0u;
                        PD_Ctl.Msg_ID = 0u;
                        PD_GetSrcCap_Sent = 0u;
                        PD_Ctl.PD_State = STA_SRC_CONNECT;
                        PD_Ctl.PD_Comm_Timer = 701u;
                        PD_EPR_State = EPR_ST_OFF;
                        PD_EPR_ModeActive = 0u;
                        PD_EPR_ContractActive = 0u;
                        PD_SPR_ContractActive = 0u;
                        printf("[PD] Soft Reset accepted; requesting Source_Capabilities again\r\n");
                    }
                    else
                    {
                        if(PD_EPR_State == EPR_ST_WAIT_REQUEST_ACCEPT)
                            PD_EPR_State = EPR_ST_WAIT_REQUEST_PSRDY;
                        PD_Ctl.PD_State = STA_RX_PS_RDY_WAIT;
                        PD_Ctl.PD_Comm_Timer = 0;
                    }
                    break;

                case DEF_TYPE_REJECT:
                    if(PD_Ctl.PD_State != STA_RX_ACCEPT_WAIT)
                    {
                        printf("[PD] ignored unexpected Reject in state %u\r\n",
                               (unsigned)PD_Ctl.PD_State);
                        break;
                    }

                    if(PD_SoftResetRecoveryPending)
                    {
                        PD_SoftResetRecoveryPending = 0u;
                        PD_LocalProtocolRecover("Soft Reset rejected by Source");
                    }
                    else if(PD_EPR_State == EPR_ST_WAIT_REQUEST_ACCEPT)
                    {
                        PD_EPR_Exit_To_SPR("28V EPR_Request rejected");
                    }
                    else
                    {
                        PD_Ctl.PD_State = STA_IDLE;
                    }
                    break;

                case DEF_TYPE_PS_RDY:
                    /* PS_RDY is meaningful only after an Accept for our current
                     * Request.  Never infer a power contract merely because a
                     * PS_RDY-shaped packet exists in the RX buffer. */
                    if(PD_Ctl.PD_State != STA_RX_PS_RDY_WAIT)
                    {
                        printf("[PD] ignored unexpected PS_RDY in state %u\r\n",
                               (unsigned)PD_Ctl.PD_State);
                        break;
                    }

                    if((PD_Selected_PDO == 0u) || (PD_Selected_mV < 4500u))
                    {
                        printf("[PD] invalid PS_RDY without a selected PDO; re-arming protocol\r\n");
                        PD_LocalProtocolRecover("PS_RDY without valid Request context");
                        break;
                    }

                    PD_ProtocolRecoveryCount = 0u;
                    if(PD_EPR_State == EPR_ST_SPR_NEGOTIATING)
                    {
                        PD_SPR_ContractActive = 1;
                        printf("[PD] SPR contract ready: PDO%u, %u mV / %u mA\r\n",
                               PD_Selected_PDO, PD_Selected_mV, PD_Selected_mA);
                        printf("[PD] EPR Mode: sending Enter, Sink PDP=%u W\r\n",
                               (unsigned)PD_EPR_SINK_PDP_W);
                        PD_EPR_State = EPR_ST_WAIT_ENTER_ACK;
                        PD_EPR_TimerMs = 0;
                        PD_Ctl.PD_State = STA_IDLE;
                        if(PD_Send_EPR_Mode(PD_EPR_MODE_ENTER, (UINT8)PD_EPR_SINK_PDP_W) != DEF_PD_TX_OK)
                            PD_EPR_Fallback("EPR Mode Enter TX failed");
                    }
                    else if(PD_EPR_State == EPR_ST_WAIT_REQUEST_PSRDY)
                    {
                        PD_SPR_ContractActive = 1;
                        PD_EPR_ContractActive = 1;
                        PD_EPR_ModeActive = 1;
                        PD_EPR_State = EPR_ST_ACTIVE;
                        PD_EPR_TimerMs = 0;
                        PD_EPR_KeepAliveMs = 0;
                        PD_EPR_KeepAliveWaitAck = 0;
                        PD_Ctl.PD_State = STA_IDLE;
                        printf("[PD] EPR contract ready: PDO%u, %u mV / %u mA\r\n",
                               PD_Selected_PDO, PD_Selected_mV, PD_Selected_mA);
                        printf("[PD] EPR KeepAlive period=%u ms\r\n",
                               (unsigned)PD_EPR_KEEPALIVE_PERIOD_MS);
                    }
                    else
                    {
                        PD_SPR_ContractActive = 1;
                        printf("[PD] Contract ready: PDO%u, target %u mV / %u mA\r\n",
                               PD_Selected_PDO, PD_Selected_mV, PD_Selected_mA);
                        PD_Ctl.PD_State = STA_RX_PS_RDY;
                    }
                    break;

                case DEF_TYPE_WAIT:
                    if(PD_Ctl.PD_State != STA_RX_ACCEPT_WAIT)
                    {
                        printf("[PD] ignored unexpected WAIT in state %u\r\n",
                               (unsigned)PD_Ctl.PD_State);
                        break;
                    }

                    if(PD_SoftResetRecoveryPending)
                    {
                        PD_SoftResetRecoveryPending = 0u;
                        PD_LocalProtocolRecover("WAIT received during Soft Reset recovery");
                    }
                    else if(PD_EPR_State == EPR_ST_WAIT_REQUEST_ACCEPT)
                    {
                        printf("[PD] WAIT to EPR_Request is a protocol error; Hard Reset\r\n");
                        PD_Ctl.PD_State = STA_TX_HRST;
                    }
                    else
                    {
                        /* For the normal SPR Request, drop the pending transaction
                         * and wait for a fresh Source_Capabilities/renegotiation. */
                        PD_Ctl.PD_State = STA_SRC_CONNECT;
                        PD_Ctl.PD_Comm_Timer = 701u;
                    }
                    break;

                case DEF_TYPE_EPR_MODE:
                    PD_Handle_EPR_Mode_Message();
                    break;

                case DEF_TYPE_NOT_SUPPORT:
                    if(PD_EPR_State == EPR_ST_WAIT_ENTER_ACK ||
                       PD_EPR_State == EPR_ST_WAIT_ENTER_SUCCESS)
                        PD_EPR_Fallback("EPR Mode not supported by Source");
                    break;

                case DEF_TYPE_GET_SNK_CAP:
                    PD_Load_Header(0x00, DEF_TYPE_SNK_CAP);
                    PD_Send_Handle(SinkCap_5V1A_Tab, sizeof(SinkCap_5V1A_Tab));
                    break;

                case DEF_TYPE_SOFT_RESET:
                    PD_Load_Header(0x00, DEF_TYPE_ACCEPT);
                    PD_Send_Handle(NULL, 0);
                    break;

                case DEF_TYPE_GET_SRC_CAP_EX:
                    PD_Load_Header(0x01, DEF_TYPE_SRC_CAP);
                    PD_Send_Handle(SrcCap_Ext_Tab, sizeof(SrcCap_Ext_Tab));
                    break;

                case DEF_TYPE_GET_STATUS:
                    PD_Load_Header(0x01, DEF_TYPE_GET_STATUS_R);
                    PD_Send_Handle(Status_Ext_Tab, sizeof(Status_Ext_Tab));
                    break;

                case DEF_TYPE_VCONN_SWAP:
                    PD_Load_Header(0x00, DEF_TYPE_REJECT);
                    PD_Send_Handle(NULL, 0);
                    break;

                case DEF_TYPE_VENDOR_DEFINED:
                    if((PD_Rx_Buf[2] & 0xC0) == 0)
                    {
                        Delay_Ms(1);
                        PD_Load_Header(0x00, DEF_TYPE_VENDOR_DEFINED);
                        if((PD_Rx_Buf[3] & 0x60) == 0) PD_Ctl.Flag.Bit.VDM_Version = 0;
                        else PD_Ctl.Flag.Bit.VDM_Version = 1;
                        PD_Rx_Buf[2] |= 0x80;
                        PD_Send_Handle(&PD_Rx_Buf[2], 4);
                    }
                    break;

                default:
                    break;
            }
        }

        /* Re-arm RX only when there is neither a completed packet waiting nor
         * an automatic GoodCRC still on the wire.  The latter is the narrow
         * race that produced diagnostics such as started/completed=6/5. */
        if(!PD_Port_MessagePending() && !PD_Port_AutoAckBusy())
            PD_Rx_Mode();
        PD_Ctl.PD_BusIdle_Timer = 0;
    }
}



void PD_Task(uint32_t now_ms)
{
    uint32_t elapsed;

    if(!s_pd_task_started)
    {
        s_pd_task_started = 1u;
        s_pd_task_last_ms = now_ms;
        s_pd_detect_last_ms = now_ms;
        Tmr_Ms_Dlt = 0u;
    }
    else
    {
        elapsed = (uint32_t)(now_ms - s_pd_task_last_ms);
        if(elapsed > 0xFFFFu) elapsed = 0xFFFFu;
        Tmr_Ms_Dlt = (UINT16)elapsed;
        s_pd_task_last_ms = now_ms;
    }

    if((uint32_t)(now_ms - s_pd_detect_last_ms) >= 5u)
    {
        s_pd_detect_last_ms = now_ms;
        PD_Det_Proc();
    }

    PD_Main_Proc();
}

void PD_SetVbusMillivolts(uint16_t mv)
{
    s_pd_vbus_mv = mv;
    s_pd_vbus_valid = 1u;
}

uint8_t PD_IsConnected(void)
{
    return PD_Ctl.Flag.Bit.Connected ? 1u : 0u;
}

uint8_t PD_IsEPRContractActive(void)
{
    return PD_EPR_ContractActive ? 1u : 0u;
}

uint8_t PD_IsPowerReady(void)
{
    if(!PD_SPR_ContractActive)
        return 0u;

    /* Safety backstop: a real contract can never be PDO0 / 0V. */
    if((PD_Selected_PDO == 0u) || (PD_Selected_mV < 4500u))
        return 0u;

#if PD_EPR_ENABLE
    /* Keep nonessential loads off while an EPR-capable source is moving from
     * the stable SPR contract into the final EPR contract.  If EPR entry
     * fails, PD_EPR_FailedForAttach makes the existing SPR contract usable. */
    if(PD_Source_EPR_Capable && !PD_EPR_FailedForAttach)
        return PD_EPR_ContractActive ? 1u : 0u;
#endif

    return 1u;
}

uint16_t PD_GetContractVoltageMv(void)
{
    return PD_Selected_mV;
}

uint16_t PD_GetContractCurrentMa(void)
{
    return PD_Selected_mA;
}

static UINT8 PD_DecodeDisplayPDO(UINT32 raw,
                                 UINT8 index,
                                 UINT8 from_epr_table,
                                 PD_DisplayPDO *pdo)
{
    UINT8 supply_type;

    if((raw == 0u) || (pdo == NULL))
        return 0u;

    memset(pdo, 0, sizeof(*pdo));
    pdo->index = index;
    supply_type = (UINT8)(raw >> 30);

    if(supply_type == 0u)
    {
        pdo->max_mv = (UINT16)(((raw >> 10) & 0x3FFu) * 50u);
        pdo->min_mv = pdo->max_mv;
        pdo->current_ma = (UINT16)((raw & 0x3FFu) * 10u);
        pdo->type = (from_epr_table && index >= 8u) ?
                    PD_DISPLAY_PDO_EPR : PD_DISPLAY_PDO_SPR;
        return (pdo->max_mv != 0u) ? 1u : 0u;
    }

    if(supply_type == 3u)
    {
        UINT8 apdo_type = (UINT8)((raw >> 28) & 0x03u);

        if(apdo_type == 0u)
        {
            pdo->type = PD_DISPLAY_PDO_PPS;
            pdo->min_mv = (UINT16)(((raw >> 8) & 0xFFu) * 100u);
            pdo->max_mv = (UINT16)(((raw >> 17) & 0xFFu) * 100u);
            pdo->current_ma = (UINT16)((raw & 0x7Fu) * 50u);
            return (pdo->max_mv != 0u) ? 1u : 0u;
        }

        if(apdo_type == 1u)
        {
            pdo->type = PD_DISPLAY_PDO_AVS;
            pdo->min_mv = (UINT16)(((raw >> 8) & 0xFFu) * 100u);
            pdo->max_mv = (UINT16)(((raw >> 17) & 0x1FFu) * 100u);
            pdo->power_w = (UINT16)(raw & 0xFFu);
            return (pdo->max_mv != 0u) ? 1u : 0u;
        }
    }

    /* Battery, Variable and unsupported APDO subtypes are not useful in the
     * compact 5-slot front-panel list. */
    return 0u;
}

static UINT8 PD_DisplayPDOComesBefore(const PD_DisplayPDO *a,
                                      const PD_DisplayPDO *b)
{
    if(a->type != b->type)
        return (a->type > b->type) ? 1u : 0u;
    if(a->max_mv != b->max_mv)
        return (a->max_mv > b->max_mv) ? 1u : 0u;
    return (a->index < b->index) ? 1u : 0u;
}

uint8_t PD_GetDisplayPDOs(PD_DisplayPDO *out, uint8_t max_count)
{
    UINT8 i;
    UINT8 source_count;
    UINT8 from_epr_table;
    UINT8 count = 0u;
    PD_DisplayPDO candidate;

    if((out == NULL) || (max_count == 0u))
        return 0u;

    /* A detached port owns no visible Source capabilities. Counts are reset by
     * PD_PHY_Reset(), but keep this public API defensive against stale RAM/UI. */
    if(!PD_Ctl.Flag.Bit.Connected)
        return 0u;

    /* EPR Source_Capabilities contains the complete PDO slot table. Before
     * that message is available, fall back to ordinary Source_Capabilities.
     * Only use the EPR table while EPR Mode is actually active, so a failed or
     * ended EPR attempt cannot leave a previous EPR table on the display. */
    from_epr_table = (PD_EPR_ModeActive && (PD_EPR_SourcePDO_Count != 0u)) ? 1u : 0u;
    source_count = from_epr_table ? PD_EPR_SourcePDO_Count : PD_SourcePDO_Count;

    for(i = 1u; i <= source_count; ++i)
    {
        UINT32 raw;
        UINT8 pos;
        UINT8 j;

        if(from_epr_table)
            raw = PD_ReadU32LE(&PD_EPR_SourcePDO_Raw[(i - 1u) << 2]);
        else
            raw = PD_ReadU32LE(&PD_SourcePDO_Raw[(i - 1u) << 2]);

        if(!PD_DecodeDisplayPDO(raw, i, from_epr_table, &candidate))
            continue;

        pos = 0u;
        while(pos < count && !PD_DisplayPDOComesBefore(&candidate, &out[pos]))
            pos++;

        if(pos >= max_count)
            continue;

        if(count < max_count)
            count++;

        j = count;
        while(j > (UINT8)(pos + 1u))
        {
            out[j - 1u] = out[j - 2u];
            j--;
        }
        out[pos] = candidate;
    }

    return count;
}
