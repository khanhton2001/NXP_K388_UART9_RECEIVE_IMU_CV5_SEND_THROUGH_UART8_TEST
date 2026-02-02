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

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include <inttypes.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>



#include "Clock_Ip.h"
#include "FlexCAN_Ip.h"
#include "IntCtrl_Ip.h"
#include "Siul2_Port_Ip_Cfg.h"
#include "Siul2_Dio_Ip_Cfg.h"
#include "Siul2_Dio_Ip.h"
#include "Siul2_Port_Ip.h"
#include "SEGGER_RTT.h"

#include "Lpi2c_Ip.h"
#include "IntCtrl_Ip.h"



typedef enum
{
    LPUART_RECEIVER = 9

} Receiver_Module_Type;

// UART
#define UART_LPUART_UART9_CHANNEL  9
#define UART_LPUART_UART8_CHANNEL  8


/* MIP constants */
#define MIP_SYNC1 0x75
#define MIP_SYNC2 0x65
#define MIP_PACKET_OVERHEAD 6U
#define MIP_MAX_PAYLOAD 255U
#define MIP_MIN_PACKET_LEN (MIP_PACKET_OVERHEAD)
#define MIP_MAX_PACKET_LEN (MIP_MAX_PAYLOAD + MIP_PACKET_OVERHEAD) /* 261 */
#define MIP_BUFFER_LEN 512U /* safe margin - >= MIP_MAX_PACKET_LEN */








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
    STATE_DESC_SET,
    STATE_PAYLOAD_LEN,
    STATE_PAYLOAD,
    STATE_CHECKSUM1,
    STATE_CHECKSUM2
} MipParseState;

typedef struct {
    MipParseState state;
    uint8_t buffer[MIP_BUFFER_LEN];  // packet buffer (ensure large enough for max packet)
    uint32_t idx;         // used while collecting payload + checksum
    uint8_t payload_len;
    float accel[3];
    float gyro[3];
    uint32_t packet_len;  // nonzero when a full packet is available in buffer[0..packet_len-1]
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
//const uint8_t mip_set_imu_format[] = {0x75, 0x65, 0x0c, 0x0a, 0x0a, 0x08, 0x01, 0x02, 0x04, 0x00, 0x64, 0x05, 0x00, 0x64, 0xf1, 0x1d};  // Len 16
const uint8_t expected_format_ack[] = {0x75, 0x65, 0x0c, 0x04, 0x04, 0xf1, 0x08, 0x00, 0xe4, 0xf9};  // Len 9

// Enable IMU Stream: Apply (0x01), enable (0x01), reserved (0x00)
//const uint8_t mip_enable_stream[] = {0x75, 0x65, 0x0c, 0x05, 0x05, 0x11, 0x01, 0x01, 0x00, 0xf1, 0x12};  // Len 11
const uint8_t expected_enable_ack[] = {0x75, 0x65, 0x0c, 0x04, 0x04, 0xf1, 0x11, 0x00, 0xed, 0x02};  // Len 9





// Modified Send_Command_And_Get_Response (uses LPUART, polls with timeout)

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






/* --- helper: big-endian 4-bytes -> float safely --- */
static float be_bytes_to_float(const uint8_t *b)
{
    uint8_t tmp[4] = { b[3], b[2], b[1], b[0] }; // big-endian -> little-endian on little hosts
    float f;
    memcpy(&f, tmp, sizeof(f));
    return f;
}

/* --- corrected Fletcher checksum: covers bytes FROM index 2 up to len-3 --- */
bool verify_mip_checksum(const uint8_t *packet, uint32_t len) {
    if (len < 6) return false; // must be at least sync(2)+desc(1)+len(1)+checksum(2)
    uint8_t sum1 = 0, sum2 = 0;
    // checksum covers descriptor set, payload length, and payload (i = 2 .. len-3)
    for (uint32_t i = 2; i < len - 2; i++) {
        sum1 = (uint8_t)((sum1 + packet[i]) & 0xFF);
        sum2 = (uint8_t)((sum2 + sum1) & 0xFF);
    }
    return (packet[len - 2] == sum1) && (packet[len - 1] == sum2);
}



/* Per-byte MIP parser.
   On success returns true and sets parser->packet_len to the total packet length (bytes stored in parser->buffer). */


/* Per-byte MIP parser.
   On success returns true and sets parser->packet_len to the total packet length (bytes stored in parser->buffer). */
bool mip_parse_byte(MipParser *parser, uint8_t byte) {
    switch (parser->state) {
        case STATE_IDLE:
            if (byte == MIP_SYNC1) {
                parser->buffer[0] = byte;
                parser->idx = 1;
                parser->state = STATE_SYNC1;
            }
            break;

        case STATE_SYNC1:
            if (byte == MIP_SYNC2) {
                parser->buffer[1] = byte;
                parser->idx = 2;
                parser->state = STATE_DESC_SET;
            } else {
                parser->state = STATE_IDLE;
            }
            break;

        case STATE_DESC_SET:
            parser->buffer[2] = byte; // descriptor set
            parser->idx = 3;
            parser->state = STATE_PAYLOAD_LEN;
            break;

        case STATE_PAYLOAD_LEN:
            parser->payload_len = byte;
            parser->buffer[3] = byte;
            parser->idx = 4;
            parser->state = (byte == 0) ? STATE_CHECKSUM1 : STATE_PAYLOAD;
            break;

        case STATE_PAYLOAD:
            if (parser->idx < sizeof(parser->buffer)) {
                parser->buffer[parser->idx++] = byte;
                if (parser->idx == 4 + parser->payload_len) {
                    parser->state = STATE_CHECKSUM1;
                }
            } else {
                // overflow safety
                parser->state = STATE_IDLE;
                parser->idx = 0;
            }
            break;

        case STATE_CHECKSUM1:
            parser->buffer[parser->idx++] = byte;
            parser->state = STATE_CHECKSUM2;
            break;

        case STATE_CHECKSUM2:
            parser->buffer[parser->idx++] = byte;
            {
                uint32_t total_len = parser->idx;
                if (!verify_mip_checksum(parser->buffer, total_len)) {
                    // invalid packet: reset and continue
                    parser->state = STATE_IDLE;
                    parser->idx = 0;
                    parser->packet_len = 0;
                    return false;
                }

                // parse payload fields (payload starts at index 4)
                uint32_t pos = 4;
                for (int i = 0; i < 3; i++) { parser->accel[i] = 0.0f; parser->gyro[i] = 0.0f; }

                while (pos < 4 + parser->payload_len) {
                    if (pos + 1 >= 4 + parser->payload_len) break; // malformed
                    uint8_t field_len = parser->buffer[pos++];
                    uint8_t desc = parser->buffer[pos++];

                    if (field_len < 2) break;

                    if (desc == 0x04) { // accel (3 floats)
                        if (pos + 12 <= 4 + parser->payload_len) {
                            for (int i = 0; i < 3; i++) {
                                parser->accel[i] = be_bytes_to_float(&parser->buffer[pos]);
                                pos += 4;
                            }
                        } else {
                            pos = 4 + parser->payload_len;
                        }
                    } else if (desc == 0x05) { // gyro (3 floats)
                        if (pos + 12 <= 4 + parser->payload_len) {
                            for (int i = 0; i < 3; i++) {
                                parser->gyro[i] = be_bytes_to_float(&parser->buffer[pos]);
                                pos += 4;
                            }
                        } else {
                            pos = 4 + parser->payload_len;
                        }
                    } else {
                        // skip unknown field
                        uint8_t skip = field_len - 2;
                        pos += skip;
                    }
                }

                // mark full packet length so main can forward the exact bytes
                parser->packet_len = total_len;

                // prepare for next packet (don't clear buffer so raw bytes can be forwarded)
                parser->state = STATE_IDLE;
                parser->idx = 0;

                return true;
            }
            break;

        default:
            parser->state = STATE_IDLE;
            parser->idx = 0;
            break;
    }
    return false;
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
                accel[i] = be_bytes_to_float(&buffer[pos]);
                pos += 4;
            }
        } else if (desc == 0x05) {  // Gyro: 3 floats (rad/s)
            for (int i = 0; i < 3; i++) {
                gyro[i] = be_bytes_to_float(&buffer[pos]);
                pos += 4;
            }
        } else {
            pos += field_len - 2;  // Skip unknown
        }
    }
    return true;
}




const uint8_t mip_ping[] = {0x75, 0x65, 0x01, 0x02, 0x02, 0x01, 0xE0, 0xC6};


const uint8_t mip_set_imu_format[] = {
  0x75,0x65,0x0C,0x0A,
  0x0A,
  0x08,
  0x01,        // apply
  0x02,        // 2 fields
  0x04,0x00,0x64,  // accel, decimation=100
  0x05,0x00,0x64,  // gyro,  decimation=100
  0xF1,0x1D
};




const uint8_t mip_enable_stream[] = {
  0x75,0x65,0x0C,0x05,
  0x05,
  0x11,
  0x01, // apply
  0x01, // enable
  0x00,
  0xF1,0x12
};



const uint8_t mip_set_idlestream[] = {
  0x75,0x65,0x01,0x02,
  0x02,
  0x02,
  0xE1,
  0xC7,

};

const uint8_t mip_set_resumestream[] = {
  0x75,0x65,0x01,0x02,
  0x02,
  0x06,
  0xE5,
  0xCB,

};


const uint8_t mip_set_accel_filter[] = {
  0x75, 0x65, 0x0C, 0x09,
  0x09,
  0x50,
  0x01, // apply
  0x04, // accel descriptor
  0x01, // enable filter
  0x01, // manual cutoff
  0x00, 0xB6, // cutoff 182 Hz

  0x00, // reserved
  0x05, 0xF0 // checksum
};

const uint8_t mip_set_gyro_filter[] = {
  0x75, 0x65, 0x0C, 0x09,
  0x09,
  0x50,
  0x01, // apply
  0x05, // gyro descriptor
  0x01, // enable filter
  0x01, // manual cutoff
  0x00, 0xB6, // cutoff 182 Hz
  0x00, // reserved
  0x06, 0xF6 // checksum
};




const uint8_t zero[8] = {0};








volatile int count = 0;


/* Extract accel (desc 0x04) and gyro (desc 0x05) fields from a valid MIP packet.
   Returns true if at least one of accel/gyro fields parsed; values updated (missing fields remain 0).
*/
bool extract_accel_gyro(const uint8_t *packet, uint32_t len, float *accel_out, float *gyro_out) {
    if (!packet || len < 6) return false;
    if (packet[0] != MIP_SYNC1 || packet[1] != MIP_SYNC2) return false;

    uint8_t payload_len = packet[3];
    if (4 + payload_len + 2 != len) return false;

    uint32_t pos = 4;
    bool got_any = false;

    while (pos < 4 + payload_len) {
        if (pos + 1 >= 4 + payload_len) break; /* malformed */
        uint8_t field_len = packet[pos++];  /* includes length byte + descriptor + data */
        uint8_t desc = packet[pos++];
        if (field_len < 2) break;
        uint32_t data_len = (uint32_t)(field_len - 2);
        if (pos + data_len > 4 + payload_len) break; /* malformed */

        if (desc == 0x04 && data_len >= 12) { /* accel: 3 floats */
            accel_out[0] = be_bytes_to_float(&packet[pos + 0]);
            accel_out[1] = be_bytes_to_float(&packet[pos + 4]);
            accel_out[2] = be_bytes_to_float(&packet[pos + 8]);
            got_any = true;
        } else if (desc == 0x05 && data_len >= 12) { /* gyro: 3 floats */
            gyro_out[0] = be_bytes_to_float(&packet[pos + 0]);
            gyro_out[1] = be_bytes_to_float(&packet[pos + 4]);
            gyro_out[2] = be_bytes_to_float(&packet[pos + 8]);
            got_any = true;
        }
        pos += data_len;
    }

    return got_any;
}








#define TRANSFER_SIZE 8U
uint8 txBuffer[TRANSFER_SIZE] = {0x2, 0x79, 0xFF, 0xCA, 0xFE, 0x99, 0x39, 0x77};

#define ONE_BYTE_SIZE 1U
#define TWO_BYTES_SIZE 2U


uint8 rxBufferMaster[TRANSFER_SIZE] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
/* Slave buffer to receive data */
uint8 rxBufferSlave[TRANSFER_SIZE] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
/* Data to be transmitted, used by both slave and master */



volatile uint8_t chip_id = 0xAA;
uint32_t timeout = 0xFFFF;


uint8_t reg_init = 0x21 ;

volatile uint8_t bmi270_init_status = 0xAA ;
volatile Lpi2c_Ip_StatusType dbg_i2c_status;
#define BMI_CHUNK_SIZE 32



#define CPU_FREQ_MHZ   48U   /* change ONLY this if clock changes */

static void Delay_us(uint32 us)
{
    while (us--)
    {
        /* 48 NOPs = 1 us @ 48 MHz */
        __asm volatile (
        	"nop\n nop\n nop\n nop\n nop\n nop\n"
            "nop\n nop\n nop\n nop\n nop\n nop\n"
            "nop\n nop\n nop\n nop\n nop\n nop\n"
            "nop\n nop\n nop\n nop\n nop\n nop\n"
            "nop\n nop\n nop\n nop\n nop\n nop\n"
            "nop\n nop\n nop\n nop\n nop\n nop\n"
            "nop\n nop\n nop\n nop\n nop\n nop\n"
        	"nop\n nop\n nop\n nop\n nop\n nop\n"

        );
    }
}



volatile  uint8_t rx_accel[6] = {0};
volatile  uint8_t rx_accel_gyr[12] = {0};
int16_t acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z;



uint8 tx = 0xAA;
boolean checkOk = TRUE;
uint8 i;


    uint8_t rx[2] = {0};
    uint8_t rx_bmi_init[2] = {0};




    uint8_t txBuf[2] = {0x7C, 0x00} ;


    //init ctrl
    uint8_t tx_initctrl[2] = {0x59, 0x00};

    // complete init ctrl
    uint8_t tx_complete_initctrl[2] = {0x59, 0x01};


    // power ctrl buf
    uint8_t tx_pwr_ctrl[2]= {0x7D, 0x04};

    //accel config buf/
    uint8_t tx_accel_conf[2]= {0x40, 0x17};

    //accel config buf
    uint8_t tx_pwr_conf[2] = {0x7C, 0x03};

    uint8_t reg = 0x00;
    uint8_t accel_reg = 0x0C;





#define MIP_MAX_PACKET  261U
uint8_t packet_buffer[MIP_MAX_PACKET];
uint32_t packet_idx = 0;



static void send_hex_line_uart8(const uint8_t *buf, uint32_t len)
{
    static char line[900];   // enough for max MIP packet (~261 bytes → ~783 chars + \r\n)
    uint32_t pos = 0;

    for (uint32_t i = 0; i < len && pos + 4 < sizeof(line); ++i) {
        int n = snprintf(&line[pos], sizeof(line) - pos, "%02X ", buf[i]);
        if (n <= 0) break;
        pos += (uint32_t)n;
    }

    if (pos + 2 < sizeof(line)) {
        line[pos++] = '\r';
        line[pos++] = '\n';
    }

    if (pos > 0) {
        (void)Lpuart_Uart_Ip_SyncSend(UART_LPUART_UART8_CHANNEL, (uint8_t*)line, pos, 10000);
    }
}




/*
   Convert 12 bytes big-endian IEEE 754 into 3 float values (X, Y, Z)
   Input: pointer to start of 12-byte data
   Output: fills 3 floats in dest[0..2]
   Returns: true if conversion OK, false if pointer invalid
*/
bool convert_12bytes_to_3floats(const uint8_t *src, float dest[3])
{
    if (!src || !dest) return false;

    // IEEE 754 single precision = 4 bytes big-endian per float
    for (int i = 0; i < 3; i++)
    {
        // Use existing be_bytes_to_float (already swaps to little-endian)
        dest[i] = be_bytes_to_float(&src[i * 4]);
    }
    return true;
}


/*
   Convert 12 bytes (big-endian IEEE 754) starting at src into 3 floats
   dest[0] = X, dest[1] = Y, dest[2] = Z
*/
void extract_3floats_from_12bytes(const uint8_t *src, float dest[3])
{
    if (!src || !dest) return;

    for (int i = 0; i < 3; i++) {
        dest[i] = be_bytes_to_float(&src[i * 4]);
    }
}





volatile uint8_t a0  ;
    volatile uint8_t a1  ;
    volatile uint8_t a2 ;
    volatile uint8_t a3 ;
    volatile uint8_t a4 ;
    volatile uint8_t a5 ;
    volatile uint8_t a6 ;
    volatile uint8_t a7 ;
    volatile uint8_t a8  ;
    volatile uint8_t a9 ;
    volatile uint8_t a10 ;
    volatile uint8_t a11;

    // Gyro raw bytes — same reason
    volatile uint8_t g0 ;
    volatile uint8_t g1 ;
    volatile uint8_t g2  ;
    volatile uint8_t g3 ;
    volatile uint8_t g4 ;
    volatile uint8_t g5 ;
    volatile uint8_t g6 ;
    volatile uint8_t g7 ;
    volatile uint8_t g8;
    volatile uint8_t g9  ;
    volatile uint8_t g10 ;
    volatile uint8_t g11 ;



// NEW: Globals for debugger tracking
volatile float accel_x = 0.0f, accel_y = 0.0f, accel_z = 0.0f;
volatile float gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f;
MipParser parser;


int main(void)
{
    /* 1. Clock */
	uint8_t rx_bmi_buffer[2];

    Clock_Ip_Init(&Clock_Ip_aClockConfig[0]);


    Siul2_Port_Ip_Init(NUM_OF_CONFIGURED_PINS_PortContainer_0_BOARD_InitPeripherals , g_pin_mux_InitConfigArr_PortContainer_0_BOARD_InitPeripherals);


    IntCtrl_Ip_Init(&IntCtrlConfig_0);
    Lpi2c_Ip_MasterInit(0U, &I2c_Lpi2cMaster_HwChannel0_Channel0);



    Lpi2c_Ip_MasterSendDataBlocking(
        		    0U,
        		    &reg,       // single byte
        		    1,
        		    FALSE,      // STOP
        		    100
        		);


        		while(((Lpi2c_Ip_MasterGetTransferStatus(0U, NULL_PTR)) == (LPI2C_IP_BUSY_STATUS) && (timeout > 0)))
        				{
        			timeout-- ;
        		}

        		//Delay_us(20000);



        		Lpi2c_Ip_MasterReceiveDataBlocking(
        				0,
						rx_bmi_buffer,
        				2,
        				TRUE,
        				100);

        		chip_id = rx_bmi_buffer[0];






        //IntCtrl_Ip_ConfigIrqRouting(&intRouteConfig);
        /* Initializes an UART driver*/
    Lpuart_Uart_Ip_Init(UART_LPUART_UART9_CHANNEL, &Lpuart_Uart_Ip_xHwConfigPB_9);
    Lpuart_Uart_Ip_Init(UART_LPUART_UART8_CHANNEL, &Lpuart_Uart_Ip_xHwConfigPB_8);


    uint8_t rx_buffer[32], tx_buffer[32];






    /* 2. FlexCAN0 init */
    FlexCAN_Ip_Init(INST_FLEXCAN_0, &FlexCAN_State0, &FlexCAN_Config0);

    /* 3. Start normal operation */
    FlexCAN_Ip_SetStartMode(INST_FLEXCAN_0);







//ping command bytes 7565 0102 0201 E0C6







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




    	   uint8_t rx;
           memset(&parser, 0, sizeof(parser));
           parser.state = STATE_IDLE;

           uint8 rx_byte;
           uint8 rx_buf[92];
           uint32 remaining;
           uint32 rx_len;


           bool ping_ok = false;
           uint32 timeout = 0xFFFFFF;





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




           Lpuart_Uart_Ip_SyncSend(
                    UART_LPUART_UART9_CHANNEL,
			        mip_set_imu_format,
                    sizeof(mip_set_imu_format),
                    10000 // timeout us
           );




           /*
           Lpuart_Uart_Ip_SyncSend(
                    UART_LPUART_UART8_CHANNEL,   // đúng UART nối IMU
			    	mip_set_imu_format,
           		    sizeof(mip_set_imu_format),
           		    10000                        // timeout us
           );
           */



           Lpuart_Uart_Ip_SyncSend(
                               UART_LPUART_UART9_CHANNEL,
							   mip_enable_stream,
                               sizeof(mip_enable_stream),
                               10000 // timeout us
                      );




           	   /*
                      Lpuart_Uart_Ip_SyncSend(
                               UART_LPUART_UART8_CHANNEL,   // đúng UART nối IMU
							   mip_enable_stream,
                      		    sizeof(mip_enable_stream),
                      		    10000                        // timeout us
                      );
                      */




                      Lpuart_Uart_Ip_SyncSend(
                                                     UART_LPUART_UART8_CHANNEL,   // đúng UART nối IMU
                      							   zero,
                                            		    sizeof(zero),
                                            		    10000                        // timeout us
                       );










                   // CONFIG ACCEL AND GYRO
                      Lpuart_Uart_Ip_SyncSend(
                        UART_LPUART_UART9_CHANNEL,
                        mip_set_accel_filter,
                        sizeof(mip_set_accel_filter),
                        10000 // timeout us
                      );

                      /*
                      Lpuart_Uart_Ip_SyncSend(
                        UART_LPUART_UART8_CHANNEL,
                        mip_set_accel_filter,
                        sizeof(mip_set_accel_filter),
                        10000 // timeout us
                      );
                      */

                      Lpuart_Uart_Ip_SyncSend(
                        UART_LPUART_UART9_CHANNEL,
                        mip_set_gyro_filter,
                        sizeof(mip_set_gyro_filter),
                        10000 // timeout us
                      );

                      /*
                      Lpuart_Uart_Ip_SyncSend(
                        UART_LPUART_UART8_CHANNEL,
                        mip_set_gyro_filter,
                        sizeof(mip_set_gyro_filter),
                        10000 // timeout us
                      );
                      */




                      Lpuart_Uart_Ip_SyncSend(
                                                      UART_LPUART_UART9_CHANNEL,
                     								 mip_set_resumestream,
                                                      sizeof(mip_set_resumestream),
                                                      10000 // timeout us
                                           );


























                      char ascii_buf[128];
                      uint8_t header[4];
                          uint32_t received;


                          int count_loop_7 = 0 ;




              //  while (timeout--)
                          uint8_t b;
                          uint8_t desc;
                          float temp_accel[3] = {0.0f, 0.0f, 0.0f};

                          float temp_gyro[3]  = {0.0f, 0.0f, 0.0f};


                      while(1)
    			{




                        if (Lpuart_Uart_Ip_SyncReceive(
                                UART_LPUART_UART9_CHANNEL,

								rx_buf,
							//	&rx,
								    92,
                            // 1,
								10000 // per-byte timeout
                            ) == LPUART_UART_IP_STATUS_SUCCESS)
                        {
                            // Forward the received byte to UART8





                        	Lpuart_Uart_Ip_SyncSend(
                                UART_LPUART_UART8_CHANNEL,
                          //      &rx,
								rx_buf,
                                92,
                                10000
                            );



                        	// Send hex dump of received data
                       // 	send_hex_line_uart8(rx_buf, 92);   // your existing function — perfect!



                        	// Feed all 92 bytes to the parser (this is safe and fast)
                        	        for (int i = 0; i < 92; i++)
                        	        {
                        	        	if (mip_parse_byte(&parser, rx_buf[i]))
                        	        	{
                        	        	    // Full packet received and checksum passed
                        	        	    uint8_t desc = parser.buffer[2];

                        	        	    if (desc == 0x80) // IMU data packet
                        	        	    {
                        	        	        // Your condition: only process when parser.buffer[47] != 0
                        	        	        if (parser.buffer[47] != 0)
                        	        	        {


                        	        	            // Directly extract accel from bytes 20 to 31


                        	        	            // Print nice values to terminal (UART8)
                        	        	            char ascii_buf[128];
                        	        	            snprintf(ascii_buf, sizeof(ascii_buf),
                        	        	                     "A: %.4f %.4f %.4f g | G: %.4f %.4f %.4f rad/s\r\n",
                        	        	                     accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
                        	        	            Lpuart_Uart_Ip_SyncSend(UART_LPUART_UART8_CHANNEL, (uint8_t*)ascii_buf, strlen(ascii_buf), 10000);
                        	        	        }
                        	        	    }
                        	        	}
                        	        }



                        	         a0 = parser.buffer[20];
                        	            a1 = parser.buffer[21];
                        	            a2 = parser.buffer[22];
                        	            a3 = parser.buffer[23];
                        	            a4 = parser.buffer[24];
                        	            a5 = parser.buffer[25];
                        	            a6 = parser.buffer[26];
                        	            a7 = parser.buffer[27];
                        	            a8 = parser.buffer[28];
                        	            a9 = parser.buffer[29];
                        	            a10 = parser.buffer[30];
                        	            a11 = parser.buffer[31];

                        	            // Gyro bytes: parser.buffer[34] to [45]
                        	             g0 = parser.buffer[34];
                        	             g1 = parser.buffer[35];
                        	             g2 = parser.buffer[36];
                        	             g3 = parser.buffer[37];
                        	             g4 = parser.buffer[38];
                        	             g5 = parser.buffer[39];
                        	             g6 = parser.buffer[40];
                        	             g7 = parser.buffer[41];
                        	             g8 = parser.buffer[42];
                        	             g9 = parser.buffer[43];
                        	             g10 = parser.buffer[44];
                        	             g11 = parser.buffer[45];







                        	        extract_3floats_from_12bytes(&parser.buffer[20], temp_accel);
                        	        accel_x = temp_accel[0];
                        	        accel_y = temp_accel[1];
                        	         accel_z = temp_accel[2];

                        	                               	        	            // Directly extract gyro from bytes 34 to 45
                        	                               	        	            extract_3floats_from_12bytes(&parser.buffer[34], temp_gyro);
                        	                               	        	            gyro_x = temp_gyro[0];
                        	                               	        	            gyro_y = temp_gyro[1];
                        	                               	        	            gyro_z = temp_gyro[2];








                        	count_loop_7++;






                        	if(count_loop_7 == 0){
                        	 Lpuart_Uart_Ip_SyncSend(
                        	            UART_LPUART_UART9_CHANNEL,
										mip_set_idlestream,
                        	            sizeof(mip_set_idlestream),
                        	            10000 // timeout us
                        	  );


                        	}















                        	// NEW: Feed each byte in rx_buf to the parser
                    /*
                        	for (int i = 0; i < 102; i++) {
                        	                                if (mip_parse_byte(&parser, rx_buf[i])) {
                        	                                    // Packet complete - check if it's 0x80 frame
                        	                                    if (parser.buffer[2] == 0x80) {
                        	                                        // Accel and gyro already extracted in parser->accel and parser->gyro
                        	                                        // Copy to globals for debugger tracking
                        	                                        accel_x = parser.accel[0];
                        	                                        accel_y = parser.accel[1];
                        	                                        accel_z = parser.accel[2];
                        	                                        gyro_x = parser.gyro[0];
                        	                                        gyro_y = parser.gyro[1];
                        	                                        gyro_z = parser.gyro[2];

                        	                                        // Print data
                        	                                        snprintf(ascii_buf, sizeof(ascii_buf),
                        	                                                 "Accel: %.6f %.6f %.6f g | Gyro: %.6f %.6f %.6f rad/s\r\n",
                        	                                                 accel_x, accel_y, accel_z,
                        	                                                 gyro_x, gyro_y, gyro_z);
                        	                                        Lpuart_Uart_Ip_SyncSend(UART_LPUART_UART8_CHANNEL, (uint8_t*)ascii_buf, strlen(ascii_buf), 10000);
                        	                                    }
                        	                                    // Reset for next packet
                        	                                    parser.packet_len = 0;
                        	                                }
                        	                            }
                        	*/



                        }




                        else
                            {
                                // Timeout (no data for 10 ms) → reset parser if stuck
                                if (parser.state != STATE_IDLE)
                                {
                                    parser.state = STATE_IDLE;
                                    parser.idx = 0;
                                }
                            }








                    }




/*
                      uint8_t b;
                      while (1) {
                          if (Lpuart_Uart_Ip_SyncReceive(UART_LPUART_UART9_CHANNEL, &b, 1, 10000) == LPUART_UART_IP_STATUS_SUCCESS) {
                              (void)Lpuart_Uart_Ip_SyncSend(UART_LPUART_UART8_CHANNEL, &b, 1, 10000);
                          }
                      }
*/





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





