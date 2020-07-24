
#include <string.h>

#include "am_mcu_apollo.h"
#include "am_bsp.h"

void *g_pvUART;

//*****************************************************************************
//
// Configuration options
//
//*****************************************************************************
//
// define the max packet size
//
#define MAX_UART_BUFFER_SIZE            1024

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
uint8_t g_pui8UARTTXBuffer[MAX_UART_BUFFER_SIZE];
uint8_t g_pui8UARTRXBuffer[MAX_UART_BUFFER_SIZE];

#define AM_UART_INST	0
#define UART_TX_PAD_NUM 48
#define UART_RX_PAD_NUM 49

const am_hal_gpio_pincfg_t g_GPIO_COM_UART_TX =
{
    .uFuncSel            = AM_HAL_PIN_48_UART0TX,
    .eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA
};

const am_hal_gpio_pincfg_t g_GPIO_COM_UART_RX =
{
    .uFuncSel            = AM_HAL_PIN_49_UART0RX
};

volatile bool g_bRx_data_available = false;

void am_uart_isr(void)
{
    uint32_t ui32Status;

    /* Read the masked interrupt status from the UART */
    am_hal_uart_interrupt_status_get(g_pvUART, &ui32Status, true);
    am_hal_uart_interrupt_clear(g_pvUART, ui32Status);
    am_hal_uart_interrupt_service(g_pvUART, ui32Status, 0);

    if (ui32Status & (AM_HAL_UART_INT_RX_TMOUT | AM_HAL_UART_INT_RX)) {
        g_bRx_data_available = true;
    }
}

void am_uart_init(uint32_t baud_rate)
{

	/* Start the UART */
    am_hal_uart_config_t sUartConfig = {
        /* Standard UART settings: 115200-8-N-1*/
        .ui32BaudRate    = baud_rate,
        .ui32DataBits    = AM_HAL_UART_DATA_BITS_8,
        .ui32Parity      = AM_HAL_UART_PARITY_NONE,
        .ui32StopBits    = AM_HAL_UART_ONE_STOP_BIT,
        .ui32FlowControl = AM_HAL_UART_FLOW_CTRL_NONE,

        /* Set TX and RX FIFOs to interrupt at three-quarters full */
        .ui32FifoLevels = (AM_HAL_UART_TX_FIFO_3_4 |
                           AM_HAL_UART_RX_FIFO_3_4),

        /* This code will use the standard interrupt handling for UART TX/RX */
        .pui8TxBuffer = g_pui8UARTTXBuffer,
        .ui32TxBufferSize = MAX_UART_BUFFER_SIZE,
        .pui8RxBuffer = g_pui8UARTRXBuffer,
        .ui32RxBufferSize = MAX_UART_BUFFER_SIZE,
    };

    am_hal_uart_initialize(AM_UART_INST, &g_pvUART);
    am_hal_uart_power_control(g_pvUART, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_uart_configure(g_pvUART, &sUartConfig);
    am_hal_gpio_pinconfig(UART_TX_PAD_NUM, g_GPIO_COM_UART_TX);
    am_hal_gpio_pinconfig(UART_RX_PAD_NUM, g_GPIO_COM_UART_RX);

    /* Make sure to enable the interrupts for RX, since the HAL doesn't already
    know we intend to use them. */
    NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn+AM_UART_INST));
    am_hal_uart_interrupt_enable(g_pvUART,
                                 (AM_HAL_UART_INT_RX | AM_HAL_UART_INT_TX |
                                  AM_HAL_UART_INT_RX_TMOUT | AM_HAL_UART_INT_TXCMP));
    am_hal_interrupt_master_enable();
}

//non-blocking
int8_t am_uart_send(uint16_t size, const uint8_t *data)
{
    uint32_t ret = AM_HAL_STATUS_SUCCESS;
    uint32_t transfer_size;

    am_hal_uart_transfer_t sUartWrite = {
        .ui32Direction = AM_HAL_UART_WRITE,
        .pui8Data = (uint8_t *)data,
        .ui32NumBytes = size,
        .ui32TimeoutMs = 0,//non-blocking
        .pui32BytesTransferred = &transfer_size,
    };

    ret = am_hal_uart_transfer(g_pvUART, &sUartWrite);

	if (ret != AM_HAL_STATUS_SUCCESS)
	    return 1;
	else
	    return 0;
}

//non-blocking
int8_t am_uart_receive(uint16_t size, uint8_t *data)
{
    uint32_t ret = AM_HAL_STATUS_SUCCESS;
    uint32_t transfer_size;

    am_hal_uart_transfer_t sUartRead = {
        .ui32Direction = AM_HAL_UART_READ,
        .pui8Data = (uint8_t *) data,
        .ui32NumBytes = size,
        .ui32TimeoutMs = 0,//non-blocking
        .pui32BytesTransferred = &transfer_size,
    };


	ret = am_hal_uart_transfer(g_pvUART, &sUartRead);

	if (ret != AM_HAL_STATUS_SUCCESS)
	    return 1;
	else
	    return 0;
}

bool am_uart_rx_data_available(void)
{
    uint32_t ui32Critical;

    if (g_bRx_data_available == true)
    {
        ui32Critical = am_hal_interrupt_master_disable();
        g_bRx_data_available = false;
        am_hal_interrupt_master_set(ui32Critical);
        return true;
    } else {
        return false;
    }
}

