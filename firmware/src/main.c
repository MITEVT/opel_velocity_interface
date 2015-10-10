#include "board.h"

// -------------------------------------------------------------
// Macro Definitions

#define CCAN_BAUD_RATE 500000 					// Desired CAN Baud Rate
#define UART_BAUD_RATE 9600 					// Desired UART Baud Rate

#define BUFFER_SIZE 8

// -------------------------------------------------------------
// Static Variable Declaration

extern volatile uint32_t msTicks;

static CCAN_MSG_OBJ_T msg_obj; 					// Message Object data structure for manipulating CAN messages
static RINGBUFF_T CAN_rx_buffer;				// Ring Buffer for storing received CAN messages
static CCAN_MSG_OBJ_T _rx_buffer[BUFFER_SIZE]; 	// Underlying array used in ring buffer

static char str[100];							// Used for composing UART messages
static uint8_t UART_rx_buffer[BUFFER_SIZE]; 	// UART received message buffer

static bool CAN_error_flag;
static uint32_t CAN_error_info;

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

// -------------------------------------------------------------
// CAN Driver Callback Functions

/**	
 * CAN receive callback executed by the Callback handler after a CAN message has been received 
 * @param msg_obj_num the msg_obj number that received a message
 */
void CAN_rx(uint8_t msg_obj_num) {
	Board_UART_Println(":)");
	/* Determine which CAN message has been received */
	msg_obj.msgobj = msg_obj_num;
	/* Now load up the msg_obj structure with the CAN message */
	LPC_CCAN_API->can_receive(&msg_obj);
	if (msg_obj_num == 1) {
		RingBuffer_Insert(&CAN_rx_buffer, &msg_obj);
	}
}

/**
 * CAN transmit callback executed by the Callback handler after
 * a CAN message has been transmitted 
 * @param msg_obj_num the msg_obj number that transmitted a message
 */
void CAN_tx(uint8_t msg_obj_num) {
}

/**
 * CAN error callback executed by the Callback handler after
 * an error has occured on the CAN bus
 * 
 * @param error_info Number describing CAN error
 */
void CAN_error(uint32_t error_info) {
	CAN_error_flag = true;
	CAN_error_info = error_info;
}

// -------------------------------------------------------------
// Interrupt Service Routines

// -------------------------------------------------------------
// Main Program Loop

int main(void) {

	//---------------
	// Initialize SysTick Timer to generate millisecond count
	if (Board_SysTick_Init()) {
		// Unrecoverable Error. Hang.
		while(1);
	}

	//---------------
	// Initialize GPIO and LED as output
	Board_LEDs_Init();
	Chip_GPIO_SetPinState(LPC_GPIO, LED0, true);

	//---------------
	// Initialize UART Communication
	Board_UART_Init(UART_BAUD_RATE);
	Board_UART_Println("Started up");

	//---------------
	// Initialize CAN  and CAN Ring Buffer

	RingBuffer_Init(&CAN_rx_buffer, _rx_buffer, sizeof(CCAN_MSG_OBJ_T), BUFFER_SIZE);
	RingBuffer_Flush(&CAN_rx_buffer);

	Board_CAN_Init(CCAN_BAUD_RATE, CAN_rx, CAN_tx, CAN_error);
 
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

	// /* Configure message object 1 to receive all 11-bit messages */
	msg_obj.msgobj = 1;
	msg_obj.mode_id = 0x000;
	msg_obj.mask = 0x000;
	LPC_CCAN_API->config_rxmsgobj(&msg_obj);

	while (1) {
		uint8_t count;
		if ((count = Board_UART_Read(UART_rx_buffer, BUFFER_SIZE)) != 0) {
			Board_UART_SendBlocking(UART_rx_buffer, count); // Echo user input
			switch (UART_rx_buffer[0]) {
				case 't': // Send a hello world
					Board_UART_Println("\r\nHello World"); 
					break;
				case 's': // Transmit a message
					msg_obj.msgobj = 2;
					msg_obj.mode_id = 0x001;
					msg_obj.dlc = 1;
					msg_obj.data[0] = 0xFF;

					LPC_CCAN_API->can_transmit(&msg_obj);
					Board_UART_Println("\r\nSent CAN Message");

					break;
				case 'i':
					Board_UART_Print("\r\nCANCTRL: ");
					itoa(LPC_CCAN->CANCTRL, str, 2);
					Board_UART_Print(str);
					Board_UART_Print(" CANTEST: ");
					itoa(LPC_CCAN->CANTEST, str, 2);
					Board_UART_Println(str);
					Board_UART_Print("CANSTAT: ");
					itoa(LPC_CCAN->CANSTAT, str, 2);
					Board_UART_Print(str);
					Board_UART_Print(" CANINT: ");
					itoa(LPC_CCAN->CANINT, str, 2);
					Board_UART_Println(str);
					Board_UART_Print("CANEC: ");
					itoa(LPC_CCAN->CANEC, str, 2);
					Board_UART_Println(str);
					break;
			}
		}

		if (!RingBuffer_IsEmpty(&CAN_rx_buffer)) {
			CCAN_MSG_OBJ_T temp_msg;
			RingBuffer_Pop(&CAN_rx_buffer, &temp_msg);
			Board_UART_Println("Received Message");
		}	

	// 	/* [Tutorial] How do I send a CAN Message?

	// 	There are 32 Message Objects in the CAN Peripherals Message RAM.
	// 	We need to pick one that isn't setup for receiving messages and use it to send.

	// 	For this exmaple we'll pick 31

	// 	msg_obj.msgobj = 31;
	// 	msg_obj.mode_id = 0x600; 		// CAN ID of Message to Send
	// 	msg_obj.dlc = 8; 				// Byte length of CAN Message
	// 	msg_obj.data[0] = 0xAA; 		// Fill your bytes here
	// 	msg_obj.data[1] = ..;
	// 	..
	// 	msg_obj.data[7] = 0xBB:

	// 	Now its time to send. But wait, what if that last request hasn't sent yet?
	// 	Let's check if the message object has been sent. We have to access either CANTXREQ1 or CANTXREQ2
	// 		depending on whether our message object is in the set of the 1st 16 or last 16
	// 		the first 16 bits correspond to the state of the tx request for each message object

	// 	if (LPC_CCAN->CANTXREQ2 & 0x0080 == 0) {
	// 		LPC_CCAN_API->can_transmit(&msg_obj);
	// 	}

	// 	*/

		if (CAN_error_flag) {
			Board_UART_Print("CAN Error. Info: ");
			CAN_error_flag = false;

			itoa(CAN_error_info, str, 2);
			Board_UART_Println(str);
		}
	}
}
