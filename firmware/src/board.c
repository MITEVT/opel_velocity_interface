#include "board.h"


// -------------------------------------------------------------
// Board ISRs

/**
 * SysTick Timer Interrupt Handler. Counts milliseconds since start
 */
void SysTick_Handler(void) {
	msTicks++;
}

/**
 * CCAN Interrupt Handler. Calls the isr() API located in the CCAN ROM
 */
void CAN_IRQHandler(void) {
	LPC_CCAN_API->isr();
}

// -------------------------------------------------------------
// Public Functions and Members

const uint32_t OscRateIn = 0;

int8_t Board_SysTick_Init(void) {
	msTicks = 0;

	// Update the value of SystemCoreClock to the clock speed in hz
	SystemCoreClockUpdate();

	// Initialize SysTick Timer to fire interrupt at 1kHz
	return (SysTick_Config (SystemCoreClock / 1000));
}

void Board_LEDs_Init(void) {
	Chip_GPIO_Init(LPC_GPIO);
	Chip_GPIO_WriteDirBit(LPC_GPIO, LED0, true);
}

void Board_UART_Init(uint32_t baudrate) {
	Chip_IOCON_PinMuxSet(LPC_IOCON, UART_RX_IOCON, (IOCON_FUNC1 | IOCON_MODE_INACT));	// Rx pin
	Chip_IOCON_PinMuxSet(LPC_IOCON, UART_TX_IOCON, (IOCON_FUNC1 | IOCON_MODE_INACT));	// Tx Pin

	Chip_UART_Init(LPC_USART);
	Chip_UART_SetBaud(LPC_USART, baudrate);
	Chip_UART_ConfigData(LPC_USART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_DIS));
	Chip_UART_SetupFIFOS(LPC_USART, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
	Chip_UART_TXEnable(LPC_USART);
}

void Board_UART_Print(const char *str) {
	Chip_UART_SendBlocking(LPC_USART, str, strlen(str));
}

void Board_UART_Println(const char *str) {
	Board_UART_Print(str);
	Board_UART_Print("\r\n");
}

void Board_UART_PrintNum(const int num, uint8_t base, bool crlf) {
	static char str[32];
	itoa(num, str, base);
	Board_UART_Print(str);
	if (crlf) Board_UART_Print("\r\n");
}

void Board_UART_SendBlocking(const void *data, uint8_t num_bytes) {
	Chip_UART_SendBlocking(LPC_USART, data, num_bytes);
}

int8_t Board_UART_Read(void *data, uint8_t num_bytes) {
	return Chip_UART_Read(LPC_USART, data, num_bytes);
}

void CAN_baudrate_calculate(uint32_t baud_rate, uint32_t *can_api_timing_cfg)
{
	uint32_t pClk, div, quanta, segs, seg1, seg2, clk_per_bit, can_sjw;
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_CAN);
	pClk = Chip_Clock_GetMainClockRate();

	clk_per_bit = pClk / baud_rate;

	for (div = 0; div <= 15; div++) {
		for (quanta = 1; quanta <= 32; quanta++) {
			for (segs = 3; segs <= 17; segs++) {
				if (clk_per_bit == (segs * quanta * (div + 1))) {
					segs -= 3;
					seg1 = segs / 2;
					seg2 = segs - seg1;
					can_sjw = seg1 > 3 ? 3 : seg1;
					can_api_timing_cfg[0] = div;
					can_api_timing_cfg[1] =
						((quanta - 1) & 0x3F) | (can_sjw & 0x03) << 6 | (seg1 & 0x0F) << 8 | (seg2 & 0x07) << 12;
					return;
				}
			}
		}
	}
}

void Board_CAN_Init(uint32_t baudrate, void (*rx_callback)(uint8_t), void (*tx_callback)(uint8_t), void (*error_callback)(uint32_t)) {

	uint32_t can_api_timing_cfg[2];
	
	CCAN_CALLBACKS_T callbacks = {
		rx_callback,
		tx_callback,
		error_callback,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
	};

	CAN_baudrate_calculate(baudrate, can_api_timing_cfg);

	/* Initialize the CAN controller */
	LPC_CCAN_API->init_can(&can_api_timing_cfg[0], TRUE);
	/* Configure the CAN callback functions */
	LPC_CCAN_API->config_calb(&callbacks);

	/* Enable the CAN Interrupt */
	NVIC_EnableIRQ(CAN_IRQn);
}

void Board_Setup_Timers(void){
    //---------------
    // Initialize the timer
        Chip_TIMER_Init(LPC_TIMER32_0);
        Chip_TIMER_Reset(LPC_TIMER32_0);
        Chip_TIMER_PrescaleSet(LPC_TIMER32_0, 0);
        LPC_TIMER32_0->CCR |= 5; // Set the first and third bits of the capture value in the Capture Control Register (see user manual)

        Chip_TIMER_Init(LPC_TIMER32_1);
        Chip_TIMER_Reset(LPC_TIMER32_1);
        Chip_TIMER_PrescaleSet(LPC_TIMER32_1, 0);
        LPC_TIMER32_1->CCR |= 5; // Set the first and third bits of the capture value in the Capture Control Register (see user manual)

    //---------------
        // Setup timer interrupt
        Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_5, (IOCON_FUNC2|IOCON_MODE_INACT));  // Set up port 1, pin 5 for use in the timer capture function
        NVIC_SetPriority(SysTick_IRQn, 1);      // Give the SysTick function a lower priority
        NVIC_SetPriority(TIMER_32_0_IRQn, 0);   // Ensure that the 32 bit timer capture interrupt has the highest priority
        NVIC_ClearPendingIRQ(TIMER_32_0_IRQn);  // Ensure that there are no pending interrupts on TIMER_32_0_IRQn
        NVIC_EnableIRQ(TIMER_32_0_IRQn);        // Enable interrupts on TIMER_32_0_IRQn
        Chip_TIMER_Enable(LPC_TIMER32_0);                                               // Start the timer
}

void Board_Timer0_Reset_Clear(void){
	Chip_TIMER_Reset(LPC_TIMER32_0);            // Reset the timer immediately 
	Chip_TIMER_ClearCapture(LPC_TIMER32_0, 0);              // Clear the capture

}

uint32_t Board_Timer0_ReadCapture(void){
	return Chip_TIMER_ReadCapture(LPC_TIMER32_0,0);	
}
