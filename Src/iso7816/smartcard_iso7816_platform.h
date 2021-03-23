/***************************
* This is the ISO7816 platform implementation
* for SmartCard interaction based on USART & GPIO devices.
*
*/

#ifndef __SMARTCARD_ISO7816_PLATFORM_H__
#define __SMARTCARD_ISO7816_PLATFORM_H__

#include <stdint.h>
#include "smartcard_iso7816_platform_usart.h"
#include "smartcard_iso7816_platform_bitbang.h"

/* Low level platform related functions */
enum {
    PLATFORM_USART = 0,
    PLATFORM_BITBANG = 1,
};

/* Low level protocol default is USART */
extern volatile uint8_t platform_low_level_protocol; 

/* The SMARTCARD_CONTACT pin is at state high (pullup to Vcc) when no card is
 * not present, and at state low (linked to GND) when the card is inserted.
 */
static inline uint8_t platform_is_smartcard_inserted(void){
    if(platform_low_level_protocol == PLATFORM_USART){
        return usart_platform_is_smartcard_inserted();
    }
    else{
        return bitbang_platform_is_smartcard_inserted();
    }
}

static inline void platform_set_smartcard_rst(uint8_t val){
    if(platform_low_level_protocol == PLATFORM_USART){
        usart_platform_set_smartcard_rst(val);
    }
    else{
        bitbang_platform_set_smartcard_rst(val);
    }
}

static inline void platform_set_smartcard_vcc(uint8_t val){
    if(platform_low_level_protocol == PLATFORM_USART){
        usart_platform_set_smartcard_vcc(val);
    }
    else{
        bitbang_platform_set_smartcard_vcc(val);
    }
}

static inline int platform_smartcard_init(void){
    if(platform_low_level_protocol == PLATFORM_USART){
        return usart_platform_smartcard_init();
    }
    else{
        return bitbang_platform_smartcard_init();
    }
}

static inline void platform_init_smartcard_contact(void){
    if(platform_low_level_protocol == PLATFORM_USART){
        usart_platform_init_smartcard_contact();
    }
    else{
        bitbang_platform_init_smartcard_contact();
    }
}

/* Adapt clocks depending on what has been received */
static inline int platform_SC_adapt_clocks(uint32_t* etu, uint32_t* frequency){
    if(platform_low_level_protocol == PLATFORM_USART){
        return usart_platform_SC_adapt_clocks(etu, frequency);
    }
    else{
        return bitbang_platform_SC_adapt_clocks(etu, frequency);
    }
}

static inline int platform_smartcard_clock_rounding(uint32_t target_freq, uint32_t* target_freq_rounded){
    if(platform_low_level_protocol == PLATFORM_USART){
        return usart_platform_smartcard_clock_rounding(target_freq, target_freq_rounded);
    }
    else{
        return bitbang_platform_smartcard_clock_rounding(target_freq, target_freq_rounded);
    }
}

/*
 * Low level related functions: we handle the low level USAT/smartcard
 * bytes send and receive stuff here.
 */
static inline int platform_SC_set_unkown_conv(void){
    if(platform_low_level_protocol == PLATFORM_USART){
        return usart_platform_SC_set_unkown_conv();
    }
    else{
        return bitbang_platform_SC_set_unkown_conv();
    }
}

static inline int platform_SC_set_direct_conv(void){
    if(platform_low_level_protocol == PLATFORM_USART){
        return usart_platform_SC_set_direct_conv();
    }
    else{
        return bitbang_platform_SC_set_direct_conv();
    }
}

static inline int platform_SC_set_inverse_conv(void){
    if(platform_low_level_protocol == PLATFORM_USART){
        return usart_platform_SC_set_inverse_conv();
    }
    else{
        return bitbang_platform_SC_set_inverse_conv();
    }
}


/* Smartcard putc and getc handling errors, with timeout in milliseconds */
static inline void platform_SC_flush(void){
    if(platform_low_level_protocol == PLATFORM_USART){
        usart_platform_SC_flush();
    }
    else{
        bitbang_platform_SC_flush();
    }
}

static inline int platform_SC_getc(uint8_t* c, uint32_t timeout, uint8_t reset){
    if(platform_low_level_protocol == PLATFORM_USART){
        return usart_platform_SC_getc(c, timeout, reset);
    }
    else{
        return bitbang_platform_SC_getc(c, timeout, reset);
    }
}

static inline int platform_SC_putc(uint8_t c, uint32_t timeout, uint8_t reset){
    if(platform_low_level_protocol == PLATFORM_USART){
        return usart_platform_SC_putc(c, timeout, reset);
    }
    else{
        return bitbang_platform_SC_putc(c, timeout, reset);
    }
}

/* Get ticks/time in milliseconds */
static inline uint32_t platform_get_microseconds_ticks(void){
    if(platform_low_level_protocol == PLATFORM_USART){
        return usart_platform_get_microseconds_ticks();
    }
    else{
        return bitbang_platform_get_microseconds_ticks();
    }
}

static inline void platform_SC_reinit_iso7816(void){
    if(platform_low_level_protocol == PLATFORM_USART){
        usart_platform_SC_reinit_iso7816();
    }
    else{
        bitbang_platform_SC_reinit_iso7816();
    }
}

static inline void platform_smartcard_reinit(void){
    if(platform_low_level_protocol == PLATFORM_USART){
        usart_platform_smartcard_reinit();
    }
    else{
        bitbang_platform_smartcard_reinit();
    }
}

static inline void platform_SC_reinit_smartcard_contact(void){
    if(platform_low_level_protocol == PLATFORM_USART){
        usart_platform_SC_reinit_smartcard_contact();
    }
    else{
        bitbang_platform_SC_reinit_smartcard_contact();
    }
}

static inline void platform_smartcard_register_smartcard_contact_handler(void (*action)(uint8_t)){
    if(platform_low_level_protocol == PLATFORM_USART){
        usart_platform_smartcard_register_smartcard_contact_handler(action);
    }
    else{
        bitbang_platform_smartcard_register_smartcard_contact_handler(action);
    }
}

static inline void platform_smartcard_register_user_handler_action(void (*action)(void)){
    if(platform_low_level_protocol == PLATFORM_USART){
        usart_platform_smartcard_register_user_handler_action(action);
    }
    else{
        bitbang_platform_smartcard_register_user_handler_action(action);
    }
}

static inline int platform_smartcard_set_1ETU_guardtime(void){
    if(platform_low_level_protocol == PLATFORM_USART){
        return usart_platform_smartcard_set_1ETU_guardtime();
    }
    else{
        return bitbang_platform_smartcard_set_1ETU_guardtime();
    }
}

/* shut the smartcard LED in case of communication error */
static inline void platform_smartcard_lost(void){
    if(platform_low_level_protocol == PLATFORM_USART){
        usart_platform_smartcard_lost();
    }
    else{
        bitbang_platform_smartcard_lost();
    }
}
static inline uint8_t platform_SC_is_lost(void){
    if(platform_low_level_protocol == PLATFORM_USART){
        return usart_platform_SC_is_lost();
    }
    else{
        return bitbang_platform_SC_is_lost();
    }
}

static inline void platform_smartcard_contact_handler(void){
    if(platform_low_level_protocol == PLATFORM_USART){
        usart_platform_smartcard_contact_handler();
    }
    else{
        bitbang_platform_smartcard_contact_handler();
    }
}

static inline void platform_smartcard_deinit(uint8_t mode){
    /* Deinitialize the mode if necessary (do nothing if the
     * mode is the one already configured)
     */
    if(mode != platform_low_level_protocol){
        if(platform_low_level_protocol == PLATFORM_USART){
            usart_platform_smartcard_deinit();
        }
        else{
            bitbang_platform_smartcard_deinit();
        }
    }
}
#endif /* __SMARTCARD_ISO7816_PLATFORM_H__ */
