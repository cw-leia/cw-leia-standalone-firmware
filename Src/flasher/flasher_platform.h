/***************************
* This is the flasher platform implementation
* based on bitbanging.
*/

#ifndef __FLASHER_PLATFORM_H__
#define __FLASHER_PLATFORM_H__

//#include "types.h"
#include <stdint.h>

/* Low level platform related functions */
#define MSB 0
#define LSB 1

uint8_t platform_flasher_is_lost(void);

void platform_flasher_init_contact(void);

void platform_flasher_reinit_contact(void);

uint8_t platform_flasher_smartcard_inserted(void);

void platform_flasher_init_rst(void);
void platform_flasher_set_rst(uint8_t val);

void platform_flasher_init_miso(void);
uint8_t platform_flasher_get_miso(void);

void platform_flasher_init_mosi(void);
void platform_flasher_set_mosi(uint8_t val);

void platform_flasher_init_clk(void);
void platform_flasher_set_clk(uint8_t val);

int platform_flasher_xtal_rounding(uint32_t target_freq, uint32_t* target_freq_rounded);
int platform_flasher_set_xtal_freq(uint32_t* frequency);

int platform_flasher_set_freq(uint32_t *frequency);

void platform_flasher_deinit(void);
int platform_flasher_init(void);

void platform_flasher_delay_nanoseconds(uint32_t nanoseconds_timeout);
void platform_flasher_delay_microseconds(uint32_t microseconds_timeout);
void platform_flasher_delay_milliseconds(uint32_t milliseconds_timeout);

int platform_flasher_xfer_byte(uint8_t in, uint8_t *out, uint8_t msb);

void platform_flasher_contact_handler(void);

#endif /* __FLASHER_PLATFORM_H__ */
