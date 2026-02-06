/*==================================================================================================
  FINAL – Transmit-Only Test for Logic Analyzer / Oscilloscope
  FlexCAN0 → continuous CAN frames, ID 0x123, 8 bytes, changing payload
  Tested and working on S32K344, S32K358, S32K396 with RTD 6.0.0 (August 2025)
==================================================================================================*/

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
#include "Siul2_Port_Ip_Cfg.h"
#include "Siul2_Dio_Ip_Cfg.h"
#include "Siul2_Dio_Ip.h"
#include "Siul2_Port_Ip.h"
// #include "SEGGER_RTT.h"

#include "Lpi2c_Ip.h"
#include "IntCtrl_Ip.h"
#include "math.h"
#include "imu_cv5.h"

#include "OsIf.h"
#include "string.h"





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




volatile int count = 0;




#define TRANSFER_SIZE 8U
uint8 txBuffer[TRANSFER_SIZE] = {0x2, 0x79, 0xFF, 0xCA, 0xFE, 0x99, 0x39, 0x77};

#define ONE_BYTE_SIZE 1U
#define TWO_BYTES_SIZE 2U


uint8 rxBufferMaster[TRANSFER_SIZE] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
/* Slave buffer to receive data */
uint8 rxBufferSlave[TRANSFER_SIZE] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
/* Data to be transmitted, used by both slave and master */



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
















int main(void)
{


	/* LPUART9 : PTB9 uart rx ; PTB10 uart tx
	 * LPUART8: PTF20 uart tx ; PTF21 uart rx
	 *
	 *Lpuart driver Uart Channel 8, channel 9 BAUDRATE 921600
	 *
	 *IntCtrl LPUART9_IRQn
	 *IntCtrl LPUART8_IRQn
	 *
	 */


    /* 1. Clock */
	uint8_t rx_bmi_buffer[2];

	Clock_Ip_Init(&Clock_Ip_aClockConfig[0]);

    Siul2_Port_Ip_Init(NUM_OF_CONFIGURED_PINS_PortContainer_0_BOARD_InitPeripherals , g_pin_mux_InitConfigArr_PortContainer_0_BOARD_InitPeripherals);

    IntCtrl_Ip_Init(&IntCtrlConfig_0);
    Lpi2c_Ip_MasterInit(0U, &I2c_Lpi2cMaster_HwChannel0_Channel0);


    /*
    Lpi2c_Ip_MasterSendDataBlocking(
        		    0U,
        		    &reg,       // single byte
        		    1,
        		    FALSE,      // STOP
        		    100
    );

	*/

    /*
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

	*/

        	//	chip_id = rx_bmi_buffer[0];




    //IntCtrl_Ip_ConfigIrqRouting(&intRouteConfig);
    /* Initializes an UART driver*/



    //uint8_t rx_buffer[32], tx_buffer[32];

    /* 2. FlexCAN0 init */
    FlexCAN_Ip_Init(INST_FLEXCAN_0, &FlexCAN_State0, &FlexCAN_Config0);

    /* 3. Start normal operation */
    FlexCAN_Ip_SetStartMode(INST_FLEXCAN_0);


    imu_cv5_init();


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



                while(1)
    			{

                	imu_cv5_process();

                }


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














/*
#include "log_debug.h"
#include "OsIf.h"
#include "string.h"

static log_level_t current_level = LOG_LEVEL_INFO;
static uint8_t is_initialized = 0;
static uint32 log_start_counter = 0;

void log_init(void) {
    // Note: Uart_Init(NULL_PTR) must be called BEFORE log_init()
    log_start_counter = OsIf_GetCounter(OSIF_COUNTER_DUMMY);
    is_initialized = 1;
}

*/



/*
void log_set_level(log_level_t level) {
    current_level = level;
}

void log_write(log_level_t level, const char* tag, const char* format, ...) {
    if (level > current_level) return;

    char buffer[256];
    char* level_str;
    uint32 timeout = 0xFFFFFF;

    switch(level) {
        case LOG_LEVEL_ERROR:   level_str = "E"; break;
        case LOG_LEVEL_WARN:    level_str = "W"; break;
        case LOG_LEVEL_INFO:    level_str = "I"; break;
        case LOG_LEVEL_DEBUG:   level_str = "D"; break;
        case LOG_LEVEL_VERBOSE: level_str = "V"; break;
        default: return;
    }

    // Get elapsed time (dummy counter - timestamp will be 0.000)
    uint32 elapsed = OsIf_GetElapsed(&log_start_counter, OSIF_COUNTER_DUMMY);
    uint32 sec = elapsed / 1000000U;
    uint32 ms = (elapsed / 1000U) % 1000U;

    int len = snprintf(buffer, sizeof(buffer), "[%lu.%03lu] %s (%s): ",
                      sec, ms, level_str, tag);

    va_list args;
    va_start(args, format);
    len += vsnprintf(buffer + len, sizeof(buffer) - len, format, args);
    va_end(args);

    len += snprintf(buffer + len, sizeof(buffer) - len, "\r\n");

    uint32 bytesTransferred = 0;
    Std_ReturnType ret = Uart_AsyncSend(LOG_UART_CHANNEL, (const uint8*)buffer, len);
    if (ret == E_OK) {
        while (Uart_GetStatus(LOG_UART_CHANNEL, &bytesTransferred, UART_SEND) == UART_STATUS_OPERATION_ONGOING && timeout-- > 0);
    }
}



 */















