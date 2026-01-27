/*==================================================================================================
  FINAL – Transmit-Only Test for Logic Analyzer / Oscilloscope
  FlexCAN0 → continuous CAN frames, ID 0x123, 8 bytes, changing payload
  Tested and working on S32K344, S32K358, S32K396 with RTD 6.0.0 (August 2025)
==================================================================================================*/
#include "Lpuart_Uart_Ip.h"
//#include "Flexio_Uart_Ip.h"
#include "Lpuart_Uart_Ip_Irq.h"
//#include "Flexio_Uart_Ip_Irq.h"
//#include "Flexio_Mcl_Ip.h"
#include "string.h"
#include "float.h"

#include "Clock_Ip.h"
#include "FlexCAN_Ip.h"
#include "IntCtrl_Ip.h"
#include "Siul2_Port_Ip_Cfg.h"
#include "Siul2_Dio_Ip_Cfg.h"
#include "Siul2_Dio_Ip.h"
#include "Siul2_Port_Ip.h"
#include "SEGGER_RTT.h"

typedef enum
{
    LPUART_RECEIVER = 9

} Receiver_Module_Type;

// UART
#define UART_LPUART_UART9_CHANNEL  9
#define UART_LPUART_UART8_CHANNEL  8



/* ============================= USER SETTINGS ============================= */
#define CAN_INSTANCE   INST_FLEXCAN_0          /* FlexCAN0  */
#define TX_MB_IDX      8U                      /* Safe TX mailbox */
#define CAN_ID         0x123U
#define DATA_LENGTH    8U

/* Choose one of the three modes */
#define MODE_NORMAL_100MS      1U   /* ~100 ms between frames (default, very clear on analyzer) */
#define MODE_FAST_10MS         0U   /* ~10  ms – good for triggering */
#define MODE_BURST_1MS         0U   /* ~1   ms – stress test / max bandwidth */



#define MSG_ID_CAN0 0xC0FEE
#define MSG_ID_CAN4 0xFACE
#define RX_MB_IDX 1U
//#define TX_MB_IDX 0U
#define RX_MB4_IDX 1U
#define TX_MB4_IDX 0U

#define MIP_SYNC1 0x75
#define MIP_SYNC2 0x65

typedef enum {
    STATE_IDLE,
    STATE_SYNC1,
    STATE_SYNC2,
    STATE_DESC_SET,
    STATE_PAYLOAD_LEN,
    STATE_PAYLOAD,
    STATE_CHECKSUM1,
    STATE_CHECKSUM2
} MipParseState;



typedef struct {
    MipParseState state;
    uint8_t buffer[128];  // Safe size
    uint32_t idx;
    uint8_t payload_len;
    uint8_t sum1, sum2;
    float accel[3], gyro[3];  // Output data
} MipParser;



#define WELCOME_MSG_1 "Hello GK7979 , This message is sent via Uart!\r\n"



volatile int exit_code = 0 ;

uint8 dummyData [8] = {1,2,3,4,5,6,7,9};
uint8 dummyDataC0[8] = {0x0A,0x1A,0x2A,0x3A,0x4A,0x5A,0x7A,0x8A};

extern void CAN0_ORED_0_31_MB_IRQHandler(void);
extern void CAN4_ORED_0_31_MB_IRQHandler(void);

void setupCanXCVR(void);
void setupCan4XCVR(void);

static uint8 tx_data[DATA_LENGTH] = {0xAA, 0x55, 0x00, 0x11, 0x22, 0x33, 0x44, 0x55};

static const Flexcan_Ip_DataInfoType tx_infoG = {
    .msg_id_type = FLEXCAN_MSG_ID_STD,
    .data_length = DATA_LENGTH,
    .is_polling  = TRUE,
    .is_remote   = FALSE
};




Flexcan_Ip_DataInfoType tx_info = {
		.msg_id_type = FLEXCAN_MSG_ID_STD,
		.data_length = 8u,
		.fd_enable = FALSE,
		.fd_padding = FALSE,
		.enable_brs = FALSE,
		.is_polling = TRUE,
		.is_remote = FALSE
};

Flexcan_Ip_DataInfoType rx_info = {
		.msg_id_type = FLEXCAN_MSG_ID_STD,
		.data_length = 8u,
		.fd_enable = FALSE,
		.fd_padding = FALSE,
		.enable_brs = FALSE,
		.is_polling = FALSE,
		.is_remote = FALSE
};

Flexcan_Ip_MsgBuffType rxData, rxFifoData;

const Flexcan_Ip_EnhancedIdTableType CAN0_EnhanceFIFO_IdFilerTable[3]=
{
  /*Enhanced FIFO filter table element 0-2 */
		{
				.filterType = FLEXCAN_IP_ENHANCED_RX_FIFO_ONE_ID_FILTER,
				.isExtendedFrame = true,
				.id2 = 0xABCD, // EXT ID filter
				.id1 = 0x1FFFFFFF, // EXT ID MASK
				.rtr2 = false, //RTR filter
				.rtr1 = true, // RTR mask
		},

		{
				.filterType = FLEXCAN_IP_ENHANCED_RX_FIFO_ONE_ID_FILTER,
				.isExtendedFrame = false,
				.id2 = 0x123,
				.id1 = 0x7FF,
				.rtr2= false,
				.rtr1 = true,

		},


		{
				.filterType = FLEXCAN_IP_ENHANCED_RX_FIFO_ONE_ID_FILTER,
				.isExtendedFrame = false,
				.id2 = 0x456,
				.id1 = 0x7FF,
				.rtr2= false,
				.rtr1 = true,

		}




} ;
















// UART TEST



/* Pre-calculated MIP packets (checksums included) */
//const uint8_t mip_ping[] = {0x75, 0x65, 0x01, 0x02, 0x02, 0x01, 0xe0, 0xc6};  // Len 8
const uint8_t expected_ping_ack[] = {0x75, 0x65, 0x01, 0x04, 0x04, 0xf1, 0x01, 0x00, 0xd5, 0x6a};  // Len 9

// Set IMU Format: Apply (0x01), 2 fields, accel 0x04 dec 100 (10Hz), gyro 0x05 dec 100
const uint8_t mip_set_imu_format[] = {0x75, 0x65, 0x0c, 0x0a, 0x0a, 0x08, 0x01, 0x02, 0x04, 0x00, 0x64, 0x05, 0x00, 0x64, 0xf1, 0x1d};  // Len 16
const uint8_t expected_format_ack[] = {0x75, 0x65, 0x0c, 0x04, 0x04, 0xf1, 0x08, 0x00, 0xe4, 0xf9};  // Len 9

// Enable IMU Stream: Apply (0x01), enable (0x01), reserved (0x00)
const uint8_t mip_enable_stream[] = {0x75, 0x65, 0x0c, 0x05, 0x05, 0x11, 0x01, 0x01, 0x00, 0xf1, 0x12};  // Len 11
const uint8_t expected_enable_ack[] = {0x75, 0x65, 0x0c, 0x04, 0x04, 0xf1, 0x11, 0x00, 0xed, 0x02};  // Len 9

// Function to verify MIP checksum (Fletcher mod 256)
bool verify_mip_checksum(const uint8_t *packet, uint32_t len) {
    uint8_t sum1 = 0, sum2 = 0;
    for (uint32_t i = 0; i < len - 2; i++) {
        sum1 = (sum1 + packet[i]) % 256;
        sum2 = (sum2 + sum1) % 256;
    }
    return (packet[len - 2] == sum1) && (packet[len - 1] == sum2);
}

// Function to parse IMU data packet (accel + gyro: ~34 bytes, big-endian floats)
bool parse_imu_data(const uint8_t *buffer, uint32_t len, float *accel, float *gyro) {
    if (len < 34 || buffer[0] != MIP_SYNC1 || buffer[1] != MIP_SYNC2 || buffer[2] != 0x80) return false;
    uint8_t payload_len = buffer[3];
    if (payload_len != len - 6 || !verify_mip_checksum(buffer, len)) return false;

    uint32_t pos = 4;  // Payload start
    while (pos < 4 + payload_len) {
        uint8_t field_len = buffer[pos++];
        uint8_t desc = buffer[pos++];
        if (field_len < 2 || pos + field_len - 2 > 4 + payload_len) return false;

        if (desc == 0x04) {  // Accel: 3 floats (g's)
            for (int i = 0; i < 3; i++) {
                uint8_t bytes[4] = {buffer[pos + 3], buffer[pos + 2], buffer[pos + 1], buffer[pos]};  // Big to little-endian
                accel[i] = *(float*)bytes;
                pos += 4;
            }
        } else if (desc == 0x05) {  // Gyro: 3 floats (rad/s)
            for (int i = 0; i < 3; i++) {
                uint8_t bytes[4] = {buffer[pos + 3], buffer[pos + 2], buffer[pos + 1], buffer[pos]};
                gyro[i] = *(float*)bytes;
                pos += 4;
            }
        } else {
            pos += field_len - 2;  // Skip unknown
        }
    }
    return true;
}

// Modified Send_Command_And_Get_Response (uses LPUART, polls with timeout)
boolean Send_Command_And_Get_Response(const uint8_t* pBuffer, uint32 tx_len, uint8_t* rxBuffer, uint32 rx_len) {
    volatile Lpuart_Uart_Ip_StatusType status;
    uint32 remainingBytes, T_timeout = 0xFFFFFF;

    // Send command
    status = Lpuart_Uart_Ip_AsyncSend(UART_LPUART_UART9_CHANNEL, pBuffer, tx_len);
    if (status != LPUART_UART_IP_STATUS_SUCCESS) return FALSE;

    do {
        status = Lpuart_Uart_Ip_GetTransmitStatus(UART_LPUART_UART9_CHANNEL, &remainingBytes);
    } while (status == LPUART_UART_IP_STATUS_BUSY && T_timeout-- > 0);

    if (status != LPUART_UART_IP_STATUS_SUCCESS) return FALSE;

    // Receive response
    status = Lpuart_Uart_Ip_AsyncReceive(UART_LPUART_UART9_CHANNEL, rxBuffer, rx_len);
    if (status != LPUART_UART_IP_STATUS_SUCCESS) return FALSE;

    T_timeout = 0xFFFFFF;
    do {
        status = Lpuart_Uart_Ip_GetReceiveStatus(UART_LPUART_UART9_CHANNEL, &remainingBytes);
    } while (status == LPUART_UART_IP_STATUS_BUSY && T_timeout-- > 0);

    return (status == LPUART_UART_IP_STATUS_SUCCESS && remainingBytes == 0);
}




boolean User_Str_Cmp(const uint8 * pBuffer1, const uint8 * pBuffer2, const uint32 length)
{
    uint32 idx = 0;
    for (idx = 0; idx < length; idx++)
    {
        if(pBuffer1[idx] != pBuffer2[idx])
        {
            return FALSE;
        }
    }
    return TRUE;
}







// MIP parse function (based on manual section 2.5)
bool mip_parse_byte(MipParser *parser, uint8_t byte) {
    // Update Fletcher checksum on EVERY byte (except the final two checksum bytes themselves)
    // This must happen BEFORE the switch, so sums include the current byte
    parser->sum1 = (parser->sum1 + byte) % 256;
    parser->sum2 = (parser->sum2 + parser->sum1) % 256;

    switch (parser->state) {
        case STATE_IDLE:
            if (byte == MIP_SYNC1) {
                parser->buffer[0] = byte;
                parser->idx = 1;
                parser->state = STATE_SYNC1;
                parser->sum1 = byte;      // Reset checksum at start of new packet
                parser->sum2 = byte;
            }
            break;

        case STATE_SYNC1:
            if (byte == MIP_SYNC2) {
                parser->buffer[1] = byte;
                parser->idx = 2;
                parser->state = STATE_DESC_SET;  // Next: descriptor set byte
            } else {
                parser->state = STATE_IDLE;  // Bad sync → restart
            }
            break;

        case STATE_DESC_SET:  // ← This is the corrected case (was duplicated STATE_SYNC1)
            parser->buffer[2] = byte;           // Descriptor set (e.g., 0x80 for IMU data)
            parser->idx = 3;
            parser->state = STATE_PAYLOAD_LEN;  // Next: payload length byte
            break;

        case STATE_PAYLOAD_LEN:
            parser->payload_len = byte;
            parser->buffer[3] = byte;
            parser->idx = 4;
            parser->state = (byte == 0) ? STATE_CHECKSUM1 : STATE_PAYLOAD;
            break;

        case STATE_PAYLOAD:
            parser->buffer[parser->idx++] = byte;
            if (parser->idx == 4 + parser->payload_len) {
                parser->state = STATE_CHECKSUM1;
            }
            break;

        case STATE_CHECKSUM1:
            parser->buffer[parser->idx++] = byte;
            if (byte != parser->sum1) {
                parser->state = STATE_IDLE;
                return false;  // Checksum fail
            }
            parser->state = STATE_CHECKSUM2;
            break;

        case STATE_CHECKSUM2:
            parser->buffer[parser->idx++] = byte;
            if (byte == parser->sum2) {
                // Valid packet! Now parse payload fields (IMU data set 0x80)
                uint32_t pos = 4;
                while (pos < 4 + parser->payload_len) {
                    uint8_t field_len = parser->buffer[pos++];
                    uint8_t desc = parser->buffer[pos++];

                    if (desc == 0x04) {  // Scaled Accelerometer Vector (3 floats, g)
                        for (int i = 0; i < 3; i++) {
                            uint8_t bytes[4] = {
                                parser->buffer[pos + 3],
                                parser->buffer[pos + 2],
                                parser->buffer[pos + 1],
                                parser->buffer[pos]
                            };
                            parser->accel[i] = *(float*)bytes;  // Big-endian → little-endian
                            pos += 4;
                        }
                    } else if (desc == 0x05) {  // Scaled Gyro Vector (3 floats, rad/s)
                        for (int i = 0; i < 3; i++) {
                            uint8_t bytes[4] = {
                                parser->buffer[pos + 3],
                                parser->buffer[pos + 2],
                                parser->buffer[pos + 1],
                                parser->buffer[pos]
                            };
                            parser->gyro[i] = *(float*)bytes;
                            pos += 4;
                        }
                    } else {
                        // Skip unknown field
                        pos += field_len - 2;
                    }
                }

                parser->state = STATE_IDLE;
                return true;  // Success: new data ready in parser->accel[] and parser->gyro[]
            } else {
                parser->state = STATE_IDLE;
                return false;  // Checksum fail
            }
            break;

        default:
            parser->state = STATE_IDLE;
            break;
    }

    return false;  // No complete packet yet
}




const uint8_t mip_ping[] = {0x75, 0x65, 0x01, 0x02, 0x02, 0x01, 0xE0, 0xC6};




volatile int count = 0;


int main(void)
{
    /* 1. Clock */


    Clock_Ip_Init(&Clock_Ip_aClockConfig[0]);


    Siul2_Port_Ip_Init(NUM_OF_CONFIGURED_PINS_PortContainer_0_BOARD_InitPeripherals , g_pin_mux_InitConfigArr_PortContainer_0_BOARD_InitPeripherals);


    IntCtrl_Ip_Init(&IntCtrlConfig_0);
        //IntCtrl_Ip_ConfigIrqRouting(&intRouteConfig);
        /* Initializes an UART driver*/
    Lpuart_Uart_Ip_Init(UART_LPUART_UART9_CHANNEL, &Lpuart_Uart_Ip_xHwConfigPB_9);
    Lpuart_Uart_Ip_Init(UART_LPUART_UART8_CHANNEL, &Lpuart_Uart_Ip_xHwConfigPB_8);


    void SEGGER_RTT_Init(void);
    uint8_t rx_buffer[32], tx_buffer[32];
    SEGGER_RTT_ConfigUpBuffer(0, "RTTUP", rx_buffer, sizeof(rx_buffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    SEGGER_RTT_ConfigDownBuffer(0, "RTTDOWN", tx_buffer, sizeof(tx_buffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    SEGGER_RTT_ConfigUpBuffer(0, NULL , NULL , 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);

    SEGGER_RTT_SetTerminal(0);
    SEGGER_RTT_printf(0, "hello world\r\n");
    SEGGER_RTT_WriteString(0, "Hello World from SEGGER!\r\n");

    SEGGER_RTT_WriteString(0, RTT_CTRL_CLEAR);

    SEGGER_RTT_printf(0, "%sTime: %s%s%.7d\n",
                          RTT_CTRL_RESET,
                          RTT_CTRL_BG_BRIGHT_RED,
                          RTT_CTRL_TEXT_BRIGHT_WHITE,
                          2222222
                          );





    /* 2. FlexCAN0 init */
    FlexCAN_Ip_Init(INST_FLEXCAN_0, &FlexCAN_State0, &FlexCAN_Config0);

    /* 3. Start normal operation */
    FlexCAN_Ip_SetStartMode(INST_FLEXCAN_0);







//ping command bytes 7565 0102 0201 E0C6



    uint32 remaining;

    Lpuart_Uart_Ip_SyncSend(
                UART_LPUART_UART9_CHANNEL,
                mip_ping,
                sizeof(mip_ping),
                10000 // timeout us
            );


/*

    Lpuart_Uart_Ip_SyncSend(
            UART_LPUART_UART8_CHANNEL,   // đúng UART nối IMU
            mip_ping,
            sizeof(mip_ping),
            10000                        // timeout us
        );
*/



  /*  Lpuart_Uart_Ip_SyncSend(
                	    	        UART_LPUART_UART8_CHANNEL,
                	    	        (uint8*)WELCOME_MSG_1,
                	    	        strlen(WELCOME_MSG_1),
                	    	        10000);   // timeout us
*/



    MipParser parser;
           memset(&parser, 0, sizeof(parser));
           parser.state = STATE_IDLE;

           uint8 rx_byte;
           bool ping_ok = false;
           uint32 timeout = 0xFFFFFF;





    Lpuart_Uart_Ip_SyncSend(
        UART_LPUART_UART8_CHANNEL,   // đúng UART nối IMU
        mip_ping,
        sizeof(mip_ping),
        10000                        // timeout us
    );





                while (timeout--)
                    {
                        if (Lpuart_Uart_Ip_SyncReceive(
                                UART_LPUART_UART9_CHANNEL,
                                &rx_byte,
                                132,
                                10000 // per-byte timeout
                            ) == LPUART_UART_IP_STATUS_SUCCESS)
                        {
                            // Forward the received byte to UART8
                            Lpuart_Uart_Ip_SyncSend(
                                UART_LPUART_UART8_CHANNEL,
                                &rx_byte,
                                132,
                                10000
                            );
                        }





                    }



  /*
                while (1)
                   {


                       //(void)FlexCAN_Ip_Send(CAN_INSTANCE, TX_MB_IDX, &tx_info, CAN_ID, tx_data);
                       FlexCAN_Ip_Send(INST_FLEXCAN_0
                       		, TX_MB_IDX, &tx_info, CAN_ID, (uint8 *)&dummyDataC0);

                       FlexCAN_Ip_MainFunctionWrite(INST_FLEXCAN_0, TX_MB_IDX);
                             while (FlexCAN_Ip_GetTransferStatus(INST_FLEXCAN_0, TX_MB_IDX)
                                    == FLEXCAN_STATUS_BUSY);

                             for (volatile uint32_t i = 0; i < 4000000U; i++) __asm("nop");
                         }
                     }
*/



                __asm("nop");   // Put breakpoint here


            }
        //}
  //      else
    //    {
            /* FAIL – no ACK */
      //      while (1)
        //    {
          //      __asm("nop");   // Or blink LED here
            //}
    //    }





