#ifndef __WOOKEY_BITBANG_CONFIG_H_
#define __WOOKEY_BITBANG_CONFIG_H_

/* NOTE: this is here for compatibility reasons, but for now there
 * is no working BITBANG mode on the WooKey board!
 * Use at your own risks!
 */
#define BITBANG_SMARTCARD_IO_PORT                       GPIOA
#define BITBANG_SMARTCARD_IO_PIN                        LL_GPIO_PIN_2


#define BITBANG_SMARTCARD_CLK_USART_PORT                GPIOA
#define BITBANG_SMARTCARD_CLK_USART_PIN                 LL_GPIO_PIN_4

/* Use timer 2 for smartcard clock */
#define BITBANG_TIMER_TO_USE                            TIM2
/* Associated channel */
#define BITBANG_TIMER_CHANNEL                           LL_TIM_CHANNEL_CH4
#define BITBANG_TIMER_IRQ                               TIM2_IRQn

/* The CLK pin is pin PA3, related to Timer2 (channel 4) */
#define BITBANG_SMARTCARD_CLK_PORT                      GPIOA
#define BITBANG_SMARTCARD_CLK_PIN                       LL_GPIO_PIN_3
#define BITBANG_SMARTCARD_CLK_AF                        LL_GPIO_AF_1

#define BITBANG_SMARTCARD_RST_PORT                      GPIOE
#define BITBANG_SMARTCARD_RST_PIN                       LL_GPIO_PIN_3

#define BITBANG_SMARTCARD_VCC_PORT                      GPIOD
#define BITBANG_SMARTCARD_VCC_PIN                       LL_GPIO_PIN_7

#define BITBANG_SMARTCARD_VPP_PORT                      GPIOE
#define BITBANG_SMARTCARD_VPP_PIN                       LL_GPIO_PIN_7

#define BITBANG_SMARTCARD_AUX1_PORT                     GPIOE
#define BITBANG_SMARTCARD_AUX1_PIN                      LL_GPIO_PIN_5

#define BITBANG_SMARTCARD_AUX2_PORT                     GPIOE
#define BITBANG_SMARTCARD_AUX2_PIN                      LL_GPIO_PIN_6

#endif // __WOOKEY_BITBANG_CONFIG_H
