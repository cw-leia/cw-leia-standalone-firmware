#ifndef _HELPER_H
# define _HELPER_H
#include <stdint.h>

//#include "types.h"
//#include "cortex_m_functions.h"


/*
 * Define time to sleep (for loop)
 */

void* local_memset(void* s, int c, uint32_t n);
void* local_memcpy(void* dest, const void* src, uint32_t n);
int local_memcmp(const void* s1, const void* s2, int n);
uint32_t local_strlen(const char* s);
char* local_strncpy(char* dest, const char* src, uint32_t n);



#endif /* _HELPER_H */
