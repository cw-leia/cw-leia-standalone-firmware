#ifndef __WOOKEY_BITBANG_CONFIG_H_
#define __WOOKEY_BITBANG_CONFIG_H_

/* NOTE: this is here for compatibility reasons, but for now there
 * is no working BITBANG mode on the WooKey board!
 */
#define SMARTCARD_IO_PORT                       GPIOA
#define SMARTCARD_IO_PIN                        LL_GPIO_PIN_2


#define SMARTCARD_CLK_USART_PORT                GPIOA
#define SMARTCARD_CLK_USART_PIN                 LL_GPIO_PIN_4

/* Use timer 2 for smartcard clock */
#define TIMER_TO_USE                            TIM2
/* Associated channel */
#define TIMER_CHANNEL                           LL_TIM_CHANNEL_CH4
#define TIMER_IRQ                               TIM2_IRQn

/* The CLK pin is pin PA3, related to Timer2 (channel 4) */
#define SMARTCARD_CLK_PORT                      GPIOA
#define SMARTCARD_CLK_PIN                       LL_GPIO_PIN_3
#define SMARTCARD_CLK_AF                        LL_GPIO_AF_1

#define SMARTCARD_RST_PORT                      GPIOE
#define SMARTCARD_RST_PIN                       LL_GPIO_PIN_3

#define SMARTCARD_VCC_PORT                      GPIOD
#define SMARTCARD_VCC_PIN                       LL_GPIO_PIN_7

#define SMARTCARD_VPP_PORT                      GPIOE
#define SMARTCARD_VPP_PIN                       LL_GPIO_PIN_7

#define SMARTCARD_AUX1_PORT                     GPIOE
#define SMARTCARD_AUX1_PIN                      LL_GPIO_PIN_5

#define SMARTCARD_AUX2_PORT                     GPIOE
#define SMARTCARD_AUX2_PIN                      LL_GPIO_PIN_6

#endif // __WOOKEY_BITBANG_CONFIG_H
