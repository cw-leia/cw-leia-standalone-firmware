#ifndef __FLASHER_PRINT_H__
#define __FLASHER_PRINT_H__

#define FLASHER_DEBUG 1

/* Primitive for debug output */
#if FLASHER_DEBUG
#include "debug.h"
#define log_printf(...) do {		\
    dbg_log(__VA_ARGS__);		\
    dbg_flush();			\
} while(0);
#define log_printf_delay(...) do {	\
    dbg_log(__VA_ARGS__);		\
} while(0);
#define log_printf_flush(...) do {	\
    dbg_flush();			\
} while(0);

#else
#define log_printf(...)
#define log_printf_delay(...)
#define log_printf_flush(...)
#endif

#endif /* _FLASHER_PRINT_H__ */
