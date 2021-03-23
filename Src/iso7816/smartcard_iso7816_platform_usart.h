/***************************
* This is the ISO7816 platform implementation
* for SmartCard interaction based on USART & GPIO devices.
*
*/

#ifndef __SMARTCARD_ISO7816_PLATFORM_USART_H__
#define __SMARTCARD_ISO7816_PLATFORM_USART_H__

//#include "types.h"
#include <stdint.h>

/* Low level platform related functions */


/* The SMARTCARD_CONTACT pin is at state high (pullup to Vcc) when no card is
 * not present, and at state low (linked to GND) when the card is inserted.
 */
uint8_t usart_platform_is_smartcard_inserted(void);

void usart_platform_set_smartcard_rst(uint8_t val);

void usart_platform_set_smartcard_vcc(uint8_t val);

int usart_platform_smartcard_init(void);
void usart_platform_init_smartcard_contact(void);

/* Adapt clocks depending on what has been received */
int usart_platform_SC_adapt_clocks(uint32_t* etu, uint32_t* frequency);

int usart_platform_smartcard_clock_rounding(uint32_t target_freq, uint32_t* target_freq_rounded);

/*
 * Low level related functions: we handle the low level USAT/smartcard
 * bytes send and receive stuff here.
 */
int usart_platform_SC_set_unkown_conv(void);

int usart_platform_SC_set_direct_conv(void);

int usart_platform_SC_set_inverse_conv(void);


/* Smartcard putc and getc handling errors, with timeout in milliseconds */
void usart_platform_SC_flush(void);

int usart_platform_SC_getc(uint8_t* c, uint32_t timeout, uint8_t reset);

int usart_platform_SC_putc(uint8_t c, uint32_t timeout, uint8_t reset);

/* Get ticks/time in milliseconds */
uint32_t usart_platform_get_microseconds_ticks(void);

void usart_platform_SC_reinit_smartcard_contact(void);

void usart_platform_SC_reinit_iso7816(void);

void usart_platform_smartcard_reinit(void);

void usart_platform_SC_reinit_smartcard_contact(void);

void usart_platform_smartcard_register_smartcard_contact_handler(void (*action)(uint8_t));

void usart_platform_smartcard_register_user_handler_action(void (*action)(void));

int usart_platform_smartcard_set_1ETU_guardtime(void);

/* shut the smartcard LED in case of communication error */
void usart_platform_smartcard_lost(void);
uint8_t usart_platform_SC_is_lost(void);

void usart_platform_smartcard_register_user_handler_action(void (*action)(void));

uint8_t usart_platform_SC_is_lost(void);

void usart_platform_smartcard_deinit(void);

void usart_platform_smartcard_contact_handler(void);

#endif /* __SMARTCARD_ISO7816_PLATFORM_USART_H__ */
