#ifndef __DISCO_FLASHER_CONFIG_H_
#define __DISCO_FLASHER_CONFIG_H_

#define FLASHER_MISO_PORT                     GPIOA
#define FLASHER_MISO_PIN                      LL_GPIO_PIN_2

/* Use timer 2 for smartcard clock */
#define FLASHER_TIMER_TO_USE                  TIM2
/* Associated channel */
#define FLASHER_TIMER_CHANNEL                 LL_TIM_CHANNEL_CH4
#define FLASHER_TIMER_IRQ                     TIM2_IRQn

/* The XTAL pin is pin PA3, related to Timer2 (channel 4) */
#define FLASHER_XTAL_PORT                     GPIOA
#define FLASHER_XTAL_PIN                      LL_GPIO_PIN_3
#define FLASHER_XTAL_AF                       LL_GPIO_AF_1

#define FLASHER_RST_PORT                      GPIOC
#define FLASHER_RST_PIN                       LL_GPIO_PIN_12

#define FLASHER_MOSI_PORT                     GPIOC
#define FLASHER_MOSI_PIN                      LL_GPIO_PIN_10

#define FLASHER_CLK_PORT                      GPIOA
#define FLASHER_CLK_PIN                       LL_GPIO_PIN_15

#define FLASHER_IO_DIR_PORT                   GPIOE
#define FLASHER_IO_DIR_PIN                    LL_GPIO_PIN_3


#endif // __DISCO_FLASHER_CONFIG_H
