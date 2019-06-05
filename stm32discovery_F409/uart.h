#ifndef UART_H_
#define UART_H_

#include "cortex_m4_systick.h"
#include "stm32f4xx_usart.h"

void console_init(usart_config_t *config);

/**
 * console_log - log strings in ring buffer
 * @fmt: format string
 */
void console_log(char *fmt, ...);
void console_log_raw(char*data, int l);
/**
 * console_flush - flush the ring buffer to UART
 */
void console_flush(void);
uint8_t consolegetc(void);
void consoleputc(uint8_t);

void init_console_ring_buffer(void);

#endif /* ! DEBUG_H_ */
