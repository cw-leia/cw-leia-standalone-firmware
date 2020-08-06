#include "helpers.h"

#if __GNUC__
# pragma GCC push_options
# pragma GCC optimize("O0")
#endif
#if __clang__
# pragma clang optimize off
#endif

void* local_memset(void* s, int c, uint32_t n)
{
    char* bytes = s;

    while (n) {
        *bytes = c;
        bytes++;
        n--;
    }

    return s;
}

void* local_memcpy(void* dest, const void* src, uint32_t n)
{
    char* d_bytes = dest;
    const char* s_bytes = src;

    while (n) {
        *d_bytes = *s_bytes;
        d_bytes++;
        s_bytes++;
        n--;
    }

    return dest;
}

int local_memcmp(const void* s1, const void* s2, int n)
{
    unsigned char u1, u2;

    for ( ; n-- ; s1++, s2++) {
        u1 = * (unsigned char*) s1;
        u2 = * (unsigned char*) s2;

        if ( u1 != u2) {
            return (u1 - u2);
        }
    }

    return 0;
}

uint32_t local_strlen(const char* s)
{
    uint32_t i = 0;

    while (*s) {
        i++;
        s++;
    }

    return i;
}

char* local_strncpy(char* dest, const char* src, uint32_t n)
{
    char* return_value = dest;

    while (n && *src) {
        *dest = *src;
        dest++;
        src++;
        n--;
    }

    while (n) {
        *dest = 0;
        dest++;
        n--;
    }

    return return_value;
}

void sleep_intern(uint8_t length)
{
    int i = 0, j = 0;
    int time_value = (1 << (length * 2));

    for (i = 0; i < time_value; i++) {
        for(j = 0; j < time_value; j++);
    }
}
#if __clang__
# pragma clang optimize on
#endif
#if __GNUG__
# pragma GCC pop_options
#endif
