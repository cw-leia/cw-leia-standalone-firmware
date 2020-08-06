/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEBUG_H_
#define __DEBUG_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"
#include "config.h"
#include <stdarg.h>

#ifdef DEBUG
/* API ------------------------------------------------------------------*/
void dbg_init();
void dbg_log(const char* format, ...);
void dbg_flush();
#else
#define dbg_init(a)
#define dbg_log(format, ...)
#define dbg_flush(a)
#endif

#endif // __DEBUG_H
