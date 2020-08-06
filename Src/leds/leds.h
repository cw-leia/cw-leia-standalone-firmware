/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LEDS_H_
#define __LEDS_H_

/* Includes ------------------------------------------------------------------*/
#include "config.h"
/* API ------------------------------------------------------------------*/
void led_init(void);
void led_startup(void);
void led_error_off(void);
void led_error_on(void);
void led_error_toggle(void);
void led_status_off(void);
void led_status_on(void);
void led_status_toggle(void);

#endif // __LEDS_H
