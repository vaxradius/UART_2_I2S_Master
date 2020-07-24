
//*****************************************************************************

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

//*****************************************************************************
//
// UART handle.
//
//*****************************************************************************
void *phUART;

#define CHECK_ERRORS(x)                                                       \
    if ((x) != AM_HAL_STATUS_SUCCESS)                                         \
    {                                                                         \
        error_handler(x);                                                     \
    }

volatile uint32_t ui32LastError;

//*****************************************************************************
//
// Catch HAL errors.
//
//*****************************************************************************
void
error_handler(uint32_t ui32ErrorStatus)
{
    ui32LastError = ui32ErrorStatus;

    while (1);
}

//*****************************************************************************
//
// UART buffers.
//
//*****************************************************************************
uint8_t g_pui8TxBuffer[256];
uint8_t g_pui8RxBuffer[2];

//*****************************************************************************
//
// UART configuration.
//
//*****************************************************************************
const am_hal_uart_config_t g_sUartConfig =
{
    //
    // Standard UART settings: 115200-8-N-1
    //
    .ui32BaudRate = 460800,
    .ui32DataBits = AM_HAL_UART_DATA_BITS_8,
    .ui32Parity = AM_HAL_UART_PARITY_NONE,
    .ui32StopBits = AM_HAL_UART_ONE_STOP_BIT,
    .ui32FlowControl = AM_HAL_UART_FLOW_CTRL_NONE,

    //
    // Set TX and RX FIFOs to interrupt at half-full.
    //
    .ui32FifoLevels = (AM_HAL_UART_TX_FIFO_1_2 |
                       AM_HAL_UART_RX_FIFO_1_2),

    //
    // Buffers
    //
    .pui8TxBuffer = g_pui8TxBuffer,
    .ui32TxBufferSize = sizeof(g_pui8TxBuffer),
    .pui8RxBuffer = g_pui8RxBuffer,
    .ui32RxBufferSize = sizeof(g_pui8RxBuffer),
};

//*****************************************************************************
//
// UART0 interrupt handler.
//
//*****************************************************************************
void
am_uart_isr(void)
{
    //
    // Service the FIFOs as necessary, and clear the interrupts.
    //
    uint32_t ui32Status, ui32Idle;
    am_hal_uart_interrupt_status_get(phUART, &ui32Status, true);
    am_hal_uart_interrupt_clear(phUART, ui32Status);
    am_hal_uart_interrupt_service(phUART, ui32Status, &ui32Idle);
}

//*****************************************************************************
//
// UART print string
//
//*****************************************************************************
void
uart_print(char *pcStr)
{
    uint32_t ui32StrLen = 0;
    uint32_t ui32BytesWritten = 0;

    //
    // Measure the length of the string.
    //
    while (pcStr[ui32StrLen] != 0)
    {
        ui32StrLen++;
    }

    //
    // Print the string via the UART.
    //
    const am_hal_uart_transfer_t sUartWrite =
    {
        .ui32Direction = AM_HAL_UART_WRITE,
        .pui8Data = (uint8_t *) pcStr,
        .ui32NumBytes = ui32StrLen,
        .ui32TimeoutMs = 0,
        .pui32BytesTransferred = &ui32BytesWritten,
    };

    CHECK_ERRORS(am_hal_uart_transfer(phUART, &sUartWrite));

    if (ui32BytesWritten != ui32StrLen)
    {
        //
        // Couldn't send the whole string!!
        //
        while(1);
    }
}


//*****************************************************************************
//
// Main
//
//*****************************************************************************
int
Uart_Init(void)
{
    //
    // Initialize the printf interface for UART output.
    //
    CHECK_ERRORS(am_hal_uart_initialize(0, &phUART));
    CHECK_ERRORS(am_hal_uart_power_control(phUART, AM_HAL_SYSCTRL_WAKE, false));
    CHECK_ERRORS(am_hal_uart_configure(phUART, &g_sUartConfig));

    //
    // Enable the UART pins.
    //
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_TX, g_AM_BSP_GPIO_COM_UART_TX);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_RX, g_AM_BSP_GPIO_COM_UART_RX);

    //
    // Enable interrupts.
    //
    NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn + AM_BSP_UART_PRINT_INST));

    //
    // Set the main print interface to use the UART print function we defined.
    //
    am_util_stdio_printf_init(uart_print);

}


#define PDM_DUMP_SIZE                (1024*180)
//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
uint16_t g_ui16PDMDataBuffer[PDM_DUMP_SIZE];

void
pcm_print(void)
{
	int16_t *pi16PDMData = (int16_t *) g_ui16PDMDataBuffer;
#if 1
	for (uint32_t i = 0; i < PDM_DUMP_SIZE; i++)
	{
		am_util_stdio_printf("%d\n", pi16PDMData[i]);
		am_util_delay_ms(1);
	}

#else
	am_util_stdio_printf("#define DUMP_SIZE (%d)\n", PDM_DUMP_SIZE);
	am_util_stdio_printf("const int16_t dump[DUMP_SIZE] = {\n");
	for (uint32_t j = 0; j < PDM_DUMP_SIZE; j++)
	{
		if(j%32==0)
		{
			am_util_stdio_printf("\n", pi16PDMData[j]);
			am_util_delay_ms(1);
		}

		if(j ==  (PDM_DUMP_SIZE-1))
			am_util_stdio_printf("%d ", pi16PDMData[j]);
		else
			am_util_stdio_printf("%d, ", pi16PDMData[j]);
		
		am_util_delay_ms(1);
	}
	am_util_stdio_printf("};\n\n");
#endif
	while(1);
}


uint32_t index = 0;
void pdm_dump(uint16_t *in,uint32_t len)
{
	if(index >= PDM_DUMP_SIZE)
		pcm_print();
	
	memcpy(g_ui16PDMDataBuffer+index, in, len*2);
	index += len;
}


