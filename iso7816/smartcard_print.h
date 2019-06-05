#ifndef __SMARTCARD_PRINT_H__
#define __SMARTCARD_PRINT_H__

#define SMARTCARD_DEBUG 1

/* Primitive for debug output */
#if SMARTCARD_DEBUG
#include "debug.h"
#define log_printf(...) dbg_log(__VA_ARGS__); dbg_flush();
#else
#define log_printf(...)
#endif

#endif /* _SMARTCARD_PRINT_H__ */
