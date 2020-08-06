/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONSOLE_H
#define __CONSOLE_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"

/* API ------------------------------------------------------------------*/
int console_init(void);
int console_putc(char c);
int console_putc_nb(char c);
int console_write(char* s, size_t len);
int console_write_str(char *s);
char console_getc(void);
int console_read(void* buffer, size_t len);
void usart_process_data(const void* data, size_t len);
void CONSOLE_UART_IRQ(void);
void CONSOLE_DMA_RX_IRQ(void);
void CONSOLE_DMA_TX_IRQ(void);

#endif /* __CONSOLE_H */
