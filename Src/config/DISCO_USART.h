#ifndef __DISCO_USART_CONFIG_H_
#define __DISCO_USART_CONFIG_H_

#ifndef BITBANG


#define SMARTCARD_IO_PORT                       GPIOA
#define SMARTCARD_IO_PIN                        LL_GPIO_PIN_2
#define SMARTCARD_IO_AF                         LL_GPIO_AF_7

#define SMARTCARD_CLK_PORT                      GPIOA
#define SMARTCARD_CLK_PIN                       LL_GPIO_PIN_4
#define SMARTCARD_CLK_AF                        LL_GPIO_AF_7

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

#endif

#endif // __DISCO_USART_CONFIG_H
