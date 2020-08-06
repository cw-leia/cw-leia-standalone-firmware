/*
 * This example shows how can application implement RX and TX DMA for UART.
 * It uses simple packet example approach and 3 separate buffers:
 *
 * - Raw DMA RX buffer where DMA transfers data from UART to memory
 * - Ringbuff for RX data which are processed by application
 * - Ringbuff for TX data to send using TX DMA
 */

/* Includes ------------------------------------------------------------------*/
#include "console.h"
#include "debug.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "ringbuff/ringbuff.h"


/* USART related functions */
void usart_rx_check(void);
void usart_process_data(const void* data, size_t len);
void usart_send_string(const char* str);
uint8_t usart_start_tx_dma_transfer(void);

/**
 * \brief           Calculate length of statically allocated array
 */
#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

/**
 * \brief           Buffer for USART DMA RX
 * \note            Contains RAW unprocessed data received by UART and transfered by DMA
 */
static uint8_t
usart_rx_dma_buffer[CONSOLE_RX_DMA_SIZE];

/**
 * \brief           Create ring buffer for received data
 */
static ringbuff_t
usart_rx_dma_ringbuff;

/**
 * \brief           Ring buffer data array for RX DMA
 */
static uint8_t
usart_rx_dma_ringbuff_data[CONSOLE_RX_RINGBUFF_SIZE];

/**
 * \brief           Create ring buffer for TX DMA
 */
static ringbuff_t
usart_tx_dma_ringbuff;

/**
 * \brief           Ring buffer data array for TX DMA
 */
static uint8_t
usart_tx_dma_ringbuff_data[CONSOLE_TX_RINGBUFF_SIZE];

/**
 * \brief           Length of TX DMA transfer
 */
static size_t
usart_tx_dma_current_len;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int console_init(void)
{

    /* Initialize ringbuff for TX & RX */
    ringbuff_init(&usart_tx_dma_ringbuff, usart_tx_dma_ringbuff_data, sizeof(usart_tx_dma_ringbuff_data));
    ringbuff_init(&usart_rx_dma_ringbuff, usart_rx_dma_ringbuff_data, sizeof(usart_rx_dma_ringbuff_data));

#ifndef USB
    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(CONSOLE_UART_CLOCK);
    LL_AHB1_GRP1_EnableClock(CONSOLE_GPIO_CLOCK);
    LL_AHB1_GRP1_EnableClock(CONSOLE_DMA_CLOCK);

    /* UART2 GPIO Configuration */
    GPIO_InitStruct.Pin = CONSOLE_TX_GPIO_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = CONSOLE_TX_GPIO_AF;
    LL_GPIO_Init(CONSOLE_TX_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CONSOLE_RX_GPIO_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = CONSOLE_RX_GPIO_AF;
    LL_GPIO_Init(CONSOLE_RX_GPIO_PORT, &GPIO_InitStruct);

    /* UART RX DMA Init */
    LL_DMA_SetChannelSelection(CONSOLE_DMA, CONSOLE_DMA_RX_STREAM, CONSOLE_DMA_CHANNEL);
    LL_DMA_SetDataTransferDirection(CONSOLE_DMA, CONSOLE_DMA_RX_STREAM, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel(CONSOLE_DMA, CONSOLE_DMA_RX_STREAM, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(CONSOLE_DMA, CONSOLE_DMA_RX_STREAM, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(CONSOLE_DMA, CONSOLE_DMA_RX_STREAM, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(CONSOLE_DMA, CONSOLE_DMA_RX_STREAM, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(CONSOLE_DMA, CONSOLE_DMA_RX_STREAM, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(CONSOLE_DMA, CONSOLE_DMA_RX_STREAM, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(CONSOLE_DMA, CONSOLE_DMA_RX_STREAM);

    /* UART TX DMA Init */
    LL_DMA_SetChannelSelection(CONSOLE_DMA, CONSOLE_DMA_TX_STREAM, CONSOLE_DMA_CHANNEL);
    LL_DMA_SetDataTransferDirection(CONSOLE_DMA, CONSOLE_DMA_TX_STREAM, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetStreamPriorityLevel(CONSOLE_DMA, CONSOLE_DMA_TX_STREAM, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(CONSOLE_DMA, CONSOLE_DMA_TX_STREAM, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(CONSOLE_DMA, CONSOLE_DMA_TX_STREAM, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(CONSOLE_DMA, CONSOLE_DMA_TX_STREAM, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(CONSOLE_DMA, CONSOLE_DMA_TX_STREAM, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(CONSOLE_DMA, CONSOLE_DMA_TX_STREAM, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(CONSOLE_DMA, CONSOLE_DMA_TX_STREAM);

    LL_DMA_SetPeriphAddress(CONSOLE_DMA, CONSOLE_DMA_RX_STREAM, (uint32_t)&CONSOLE_UART->DR);
    LL_DMA_SetMemoryAddress(CONSOLE_DMA, CONSOLE_DMA_RX_STREAM, (uint32_t)usart_rx_dma_buffer);
    LL_DMA_SetDataLength(CONSOLE_DMA, CONSOLE_DMA_RX_STREAM, ARRAY_LEN(usart_rx_dma_buffer));

    LL_DMA_SetPeriphAddress(CONSOLE_DMA, CONSOLE_DMA_TX_STREAM, (uint32_t)&CONSOLE_UART->DR);

    /* Enable DMA RX HT & TC interrupts */
    LL_DMA_EnableIT_HT(CONSOLE_DMA, CONSOLE_DMA_RX_STREAM);
    LL_DMA_EnableIT_TC(CONSOLE_DMA, CONSOLE_DMA_RX_STREAM);
    /* Enable DMA TX TC interrupts */
    LL_DMA_EnableIT_TC(CONSOLE_DMA, CONSOLE_DMA_TX_STREAM);

    /* DMA interrupt init */
    NVIC_SetPriority(CONSOLE_DMA_RX_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(CONSOLE_DMA_RX_IRQn);

    /* DMA interrupt init */
    NVIC_SetPriority(CONSOLE_DMA_TX_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(CONSOLE_DMA_TX_IRQn);

    /* Configure UART */
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_8;
    LL_USART_Init(CONSOLE_UART, &USART_InitStruct);
    LL_USART_ConfigAsyncMode(CONSOLE_UART);
    LL_USART_EnableDMAReq_RX(CONSOLE_UART);
    LL_USART_EnableDMAReq_TX(CONSOLE_UART);
    LL_USART_EnableIT_IDLE(CONSOLE_UART);

    /* USART interrupt, same priority as DMA channel */
    NVIC_SetPriority(CONSOLE_UART_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
    NVIC_EnableIRQ(CONSOLE_UART_IRQn);

    /* Enable USART and DMA RX */
    LL_DMA_EnableStream(CONSOLE_DMA, CONSOLE_DMA_RX_STREAM);
    LL_USART_Enable(CONSOLE_UART);
#endif
    return 0;
}

#ifdef USB
#include "usbd_cdc_if.h"
int console_putc(char c)
{
    while(CDC_Transmit_FS((uint8_t*)&c, 1, 0) != (USBD_OK)) {}

    return 0;
}

int console_putc_nb(char c)
{
    CDC_Transmit_FS((uint8_t*)&c, 1, 0);
    return 0;
}


int console_write(char* s, size_t len)
{
    while(CDC_Transmit_FS((uint8_t*)s, len, 0) != (USBD_OK)) {}

    return 0;
}
#else
int console_putc(char c)
{
    ringbuff_write(&usart_tx_dma_ringbuff, &c, 1);   /* Write data to transmit buffer */
    usart_start_tx_dma_transfer();

    return 0;
}

int console_putc_nb(char c)
{
	return console_putc(c);
}
int console_write(char* s, size_t len)
{
    ringbuff_write(&usart_tx_dma_ringbuff, s, len);   /* Write data to transmit buffer */
    usart_start_tx_dma_transfer();

    return 0;
}
#endif

int console_write_str(char* s)
{
    return console_write(s, strlen(s));
}

char console_getc(void)
{
    char c;

    while ( ringbuff_read(&usart_rx_dma_ringbuff, &c, 1) != 1);

    return c;
}

int console_read(void* buffer, size_t len)
{
    return ringbuff_read(&usart_rx_dma_ringbuff, buffer, len);
}


/**
 * \brief           Check for new data received with DMA
 */
void
usart_rx_check(void)
{
    static size_t old_pos;
    size_t pos;

    /* Calculate current position in buffer */
    pos = ARRAY_LEN(usart_rx_dma_buffer) - LL_DMA_GetDataLength(CONSOLE_DMA, CONSOLE_DMA_RX_STREAM);

    if (pos != old_pos) {                       /* Check change in received data */
        if (pos > old_pos) {                    /* Current position is over previous one */
            /* We are in "linear" mode */
            /* Process data directly by subtracting "pointers" */
            usart_process_data(&usart_rx_dma_buffer[old_pos], pos - old_pos);

        } else {
            /* We are in "overflow" mode */
            /* First process data to the end of buffer */
            usart_process_data(&usart_rx_dma_buffer[old_pos], ARRAY_LEN(usart_rx_dma_buffer) - old_pos);

            /* Check and continue with beginning of buffer */
            if (pos > 0) {
                usart_process_data(&usart_rx_dma_buffer[0], pos);
            }
        }
    }

    old_pos = pos;                              /* Save current position as old */

    /* Check and manually update if we reached end of buffer */
    if (old_pos == ARRAY_LEN(usart_rx_dma_buffer)) {
        old_pos = 0;
    }
}

/**
 * \brief           Check if DMA is active and if not try to send data
 */
uint8_t
usart_start_tx_dma_transfer(void)
{
    uint32_t old_primask;
    uint8_t started = 0;

    /* Check if DMA is active */
    /* Must be set to 0 */
    old_primask = __get_PRIMASK();
    __disable_irq();

    if (!usart_tx_dma_current_len) {
        /* Check if something to send  */
        usart_tx_dma_current_len = ringbuff_get_linear_block_read_length(&usart_tx_dma_ringbuff);

        if (usart_tx_dma_current_len) {
            /* Disable channel if enabled */
            LL_DMA_DisableStream(CONSOLE_DMA, CONSOLE_DMA_TX_STREAM);

            /* Clear all flags */
            LL_DMA_ClearFlag_TC4(CONSOLE_DMA);
            LL_DMA_ClearFlag_FE4(CONSOLE_DMA);
            LL_DMA_ClearFlag_DME4(CONSOLE_DMA);
            LL_DMA_ClearFlag_TE4(CONSOLE_DMA);
            LL_DMA_ClearFlag_HT4(CONSOLE_DMA);

            /* Start DMA transfer */
            LL_DMA_SetDataLength(CONSOLE_DMA, CONSOLE_DMA_TX_STREAM, usart_tx_dma_current_len);
            LL_DMA_SetMemoryAddress(CONSOLE_DMA, CONSOLE_DMA_TX_STREAM, (uint32_t)ringbuff_get_linear_block_read_address(&usart_tx_dma_ringbuff));

            /* Start new transfer */
            LL_DMA_EnableStream(CONSOLE_DMA, CONSOLE_DMA_TX_STREAM);
            started = 1;
        }
    }

    __set_PRIMASK(old_primask);
    return started;
}

/**
 * \brief           Process received data over UART
 * Data are written to RX ringbuffer for application processing at latter stage
 * \param[in]       data: Data to process
 * \param[in]       len: Length in units of bytes
 */
void
usart_process_data(const void* data, size_t len)
{
    ringbuff_write(&usart_rx_dma_ringbuff, data, len);  /* Write data to receive buffer */
}

/* Interrupt handlers here */
/**
 * \brief           DMA stream interrupt handler for UART RX
 */
void
CONSOLE_DMA_RX_IRQ(void)
{
    /* Check half-transfer complete interrupt */
    if (LL_DMA_IsEnabledIT_HT(CONSOLE_DMA, CONSOLE_DMA_RX_STREAM) && LL_DMA_IsActiveFlag_HT2(CONSOLE_DMA)) {
        LL_DMA_ClearFlag_HT2(CONSOLE_DMA);             /* Clear half-transfer complete flag */
        usart_rx_check();                       /* Check for data to process */
    }

    /* Check transfer-complete interrupt */
    if (LL_DMA_IsEnabledIT_TC(CONSOLE_DMA, CONSOLE_DMA_RX_STREAM) && LL_DMA_IsActiveFlag_TC2(CONSOLE_DMA)) {
        LL_DMA_ClearFlag_TC2(CONSOLE_DMA);             /* Clear transfer complete flag */
        usart_rx_check();                       /* Check for data to process */
    }
}

void
CONSOLE_DMA_TX_IRQ(void)
{
    /* Events for DMA DMA TX */
    /* Check transfer complete */
    if (LL_DMA_IsEnabledIT_TC(CONSOLE_DMA, CONSOLE_DMA_TX_STREAM) && LL_DMA_IsActiveFlag_TC4(CONSOLE_DMA)) {
        LL_DMA_ClearFlag_TC4(CONSOLE_DMA);             /* Clear transfer complete flag */
        ringbuff_skip(&usart_tx_dma_ringbuff, usart_tx_dma_current_len);/* Skip sent data, mark as read */
        usart_tx_dma_current_len = 0;           /* Clear length variable */
        usart_start_tx_dma_transfer();          /* Start sending more data */
    }

    /* Implement other events when needed */
}

/**
 * \brief           USART global interrupt handler
 */
void
CONSOLE_UART_IRQ(void)
{
    // USART2_IRQHandler(void);
    /* Check for IDLE line interrupt */
    if (LL_USART_IsEnabledIT_IDLE(CONSOLE_UART) && LL_USART_IsActiveFlag_IDLE(CONSOLE_UART)) {
        LL_USART_ClearFlag_IDLE(CONSOLE_UART);        /* Clear IDLE line flag */
        usart_rx_check();                       /* Check for data to process */
    }

    /* Implement other events when needed */
}
