#include "board.h"
#include "can.h"

// -------------------------------------------------------------
// Macro Definitions

#define CCAN_BAUD_RATE 500000 					// Desired CAN Baud Rate
#define UART_BAUD_RATE 115200 					// Desired UART Baud Rate
#define GMB_EDGES_PER_ROTATION 49	 		        // Clock edges per wheel rotation for GBM wheel bearing sensor

#define BUFFER_SIZE 8

// -------------------------------------------------------------
// Static Variable Declaration

extern volatile uint32_t msTicks;

// Variables for the wheel velocity tracking
static char rpm_str[23];		// Used for composing UART messages
volatile static uint32_t rpm_ticks;	// The current running average of time
volatile static uint32_t rpm_count;	// Number of samples taken since the last output
volatile static uint16_t curr_rpm;	// Current RPM measurement

CCAN_MSG_OBJ_T rx_msg;
static CCAN_MSG_OBJ_T msg_obj; 			// Message Object data structure for manipulating CAN messages
static RINGBUFF_T can_rx_buffer; 		// Ring Buffer for storing received CAN messages
static CCAN_MSG_OBJ_T _rx_buffer[BUFFER_SIZE]; 	// Underlying array used in ring buffer

static char str[100];				// Used for composing UART messages
static uint8_t uart_rx_buffer[BUFFER_SIZE]; 	// UART received message buffer

static bool can_error_flag;
static uint32_t can_error_info;

static uint32_t lastPrint;

// -------------------------------------------------------------
// Helper Functions

/**
 * Delay the processor for a given number of milliseconds
 * @param ms Number of milliseconds to delay
 */
void _delay(uint32_t ms) {
	uint32_t curTicks = msTicks;
	while ((msTicks - curTicks) < ms);
}

static void Print_Buffer(uint8_t* buff, uint8_t buff_size) {
    Chip_UART_SendBlocking(LPC_USART, "0x", 2);
    uint8_t i;
    for(i = 0; i < buff_size; i++) {
        itoa(buff[i], str, 16);
        if(buff[i] < 16) {
            Chip_UART_SendBlocking(LPC_USART, "0", 1);
        }
        Chip_UART_SendBlocking(LPC_USART, str, 2);
    }
}
// -------------------------------------------------------------

// Interrupt Service Routines

void TIMER32_0_IRQHandler(void){
	Board_Timer0_Reset_Clear();
	rpm_ticks = (rpm_ticks*rpm_count+Board_Timer0_ReadCapture())/(1+rpm_count);	// Continue the running average 
	rpm_count++;    								// Increase the count to allow the running average to be properly computed
}
// -------------------------------------------------------------
// Main Program Loop

int main(void)
{

	//---------------
	// Initialize SysTick Timer to generate millisecond count
	if (Board_SysTick_Init()) {
		// Unrecoverable Error. Hang.
		while(1);
	}
	//---------------
	// Initialize UART Communication
	Board_UART_Init(UART_BAUD_RATE);
	Board_UART_Println("Started up");

	//---------------
	// Initialize GPIO and LED as output
//	Board_LEDs_Init();
//	Board_LED_On(LED0);

	//---------------
	// Initialize CAN  and CAN Ring Buffer

	CAN_Init(CCAN_BAUD_RATE);

//	rpm_count = 0;

	Board_Setup_Timers();

	// For your convenience.
	// typedef struct CCAN_MSG_OBJ {
	// 	uint32_t  mode_id;
	// 	uint32_t  mask;
	// 	uint8_t   data[8];
	// 	uint8_t   dlc;
	// 	uint8_t   msgobj;
	// } CCAN_MSG_OBJ_T;

	/* [Tutorial] How do filters work?

		Incoming ID & Mask == Mode_ID for msgobj to accept message

		Incoming ID : 0xabc
		Mask: 		  0xF0F &
		            -----------
		              0xa0c

		mode_id == 0xa0c for msgobj to accept message

	*/


	/* [Tutorial] How do I send a CAN Message?

		There are 32 Message Objects in the CAN Peripherals Message RAM.
		We need to pick one that isn't setup for receiving messages and use it to send.

		For this exmaple we'll pick 31

		msg_obj.msgobj = 31;
		msg_obj.mode_id = 0x600; 		// CAN ID of Message to Send
		msg_obj.dlc = 8; 				// Byte length of CAN Message
		msg_obj.data[0] = 0xAA; 		// Fill your bytes here
		msg_obj.data[1] = ..;
		..
		msg_obj.data[7] = 0xBB:

		Now its time to send
		LPC_CCAN_API->can_transmit(&msg_obj);

	*/
	uint32_t ret;
	uint32_t reset_can_peripheral_time;
	const uint32_t can_error_delay = 5000;
	bool reset_can_peripheral = false;
	can_error_flag = false;
	can_error_info = 0;
	uint8_t heartbeat[1];
	heartbeat[0] = 0x00; 
	uint8_t t=0x04;

	while (1) {

		if (lastPrint<msTicks-200) {
//			Board_UART_Println("Sending heartbeat");
//			Board_UART_PrintNum(heartbeat[0],16,True;
			CAN_Transmit(0x703,heartbeat,1);
			lastPrint = msTicks;
		}/* //increments velocity msg
		heartbeat[0] = heartbeat[0]+t;
		if (heartbeat[0]>0xF0) {
			heartbeat[0]=0x00;
		}*/
		if(reset_can_peripheral && msTicks > reset_can_peripheral_time) {
		    Board_UART_Println("Attempting to reset CAN peripheral...");
		    CAN_ResetPeripheral();
		    CAN_Init(CCAN_BAUD_RATE);
		    Board_UART_Println("Reset CAN peripheral. ");
		    reset_can_peripheral = false;
		}


            // recieve message if there is a message
		    ret = CAN_Receive(&rx_msg);
		    if(ret == NO_RX_CAN_MESSAGE) {
//		        Board_UART_Println("No CAN message received...");
		    } else if(ret == NO_CAN_ERROR) {
		        Board_UART_Print("Recieved data ");
		        Print_Buffer(rx_msg.data, rx_msg.dlc);
		        Board_UART_Print(" from ");
		        Board_UART_PrintNum(rx_msg.mode_id,16,true);
		    } else {
		        Board_UART_Print("CAN Error: ");
		        Board_UART_PrintNum(ret, 2,true);

		        Board_UART_Print("Will attempt to reset peripheral in ");
		        Board_UART_PrintNum(can_error_delay/1000,10,false);
		        Board_UART_Println(" seconds.");
		        reset_can_peripheral = true;
		        reset_can_peripheral_time = msTicks + can_error_delay;
		    }

		uint8_t count;
		uint8_t data[1];	
        
		if ((count = Chip_UART_Read(LPC_USART, uart_rx_buffer, BUFFER_SIZE)) != 0) {
			switch (uart_rx_buffer[0]) {
				case 'a':
					Board_UART_Println("Sending CAN with ID: 0x600");
					data[0] = 0xAA;
					ret = CAN_Transmit(0x600, data, 1);
					    if(ret != NO_CAN_ERROR) {
						Board_UART_Print("CAN Error: ");
						Board_UART_PrintNum(ret, 2,true);

					    }
					break;
				default:
					Board_UART_Println("Invalid Command");
					break;
			}
		}
	}
}
