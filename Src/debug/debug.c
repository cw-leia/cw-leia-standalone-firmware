#include <stdio.h>
#include "ringbuff/ringbuff.h"

#include "debug.h"
#include "console.h"

#define FORCE_TTY_DEBUG

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef DEBUG

/**
 * \brief           Create ring buffer for TX DMA
 */
static ringbuff_t
usart_tx_ringbuff;

/**
 * \brief           Ring buffer data array for TX DMA
 */
static uint8_t
usart_tx_ringbuff_data[DEBUG_TX_RINGBUFF_SIZE];

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * \brief           Init debug module
 */

void dbg_init()
{
    /* Initialize ringbuff for TX */
    ringbuff_init(&usart_tx_ringbuff, usart_tx_ringbuff_data, sizeof(usart_tx_ringbuff_data));

#if !defined(USB) || defined(FORCE_TTY_DEBUG) 
    /* We us a real USART when USB is not selected */
    LL_USART_InitTypeDef USART_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    DEBUG_UART_ENABLE_CLOCK();
    DEBUG_GPIO_ENABLE_CLOCK();

    /* USART GPIO Configuration */
    GPIO_InitStruct.Pin = DEBUG_GPIO_PIN;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = DEBUG_GPIO_AF;
    LL_GPIO_Init(DEBUG_GPIO_PORT, &GPIO_InitStruct);

    /* Configure UART */
    USART_InitStruct.BaudRate = 115200;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_8;
    LL_USART_Init(DEBUG_UART, &USART_InitStruct);
    LL_USART_ConfigAsyncMode(DEBUG_UART);

    /* Enable UART */
    LL_USART_Enable(DEBUG_UART);
#endif
}

/**
 * \brief           Format string and send it
 */
void dbg_log(const char* format, ...)
{
    char buffer[256];
    va_list args;
    va_start (args, format);
    vsnprintf (buffer, 256, format, args);
    va_end (args);

    ringbuff_write(&usart_tx_ringbuff, buffer, strlen(buffer));   /* Write data to transmit buffer */
}


#ifdef USB
#include "usbd_cdc_if.h"
#endif
void dbg_flush()
{
    char c;

    while (ringbuff_read(&usart_tx_ringbuff, &c, 1) == 1) {
#ifdef USB
        /* NOTE: we drop the sending if no one is on the other hand of the line ...
         * For this, we use a time out in terms of tries (since CDC_Transmit_FS is non blocking).
         * We have to use this time out here since we DO NOT want our debug flushing primitive
         * to freeze the entire platform on case of non ACKing peer!
         */
        uint32_t tries = 0;
        while(CDC_Transmit_FS((uint8_t*)&c, 1, 2) != USBD_OK)
        {
            tries++;
            if(tries > 400){
                break;
            }
        }
#endif
#if !defined(USB) || defined(FORCE_TTY_DEBUG) 
        LL_USART_TransmitData8(DEBUG_UART, c);

        while (!LL_USART_IsActiveFlag_TXE(DEBUG_UART)) {}
#endif
    }
}

#endif /* DEBUG */
