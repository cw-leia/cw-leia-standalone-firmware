#ifndef __DISCO_CONFIG_H_
#define __DISCO_CONFIG_H_

#ifdef DISCO

#include "DISCO_USART.h"
#include "DISCO_BITBANG.h"
#include "DISCO_FLASHER.h"

/////////////////////////// SMARTCARD CONTACT PIN //////////////////////////////

/* NOTE: the contact port on the DISCO board is dedicated here.
 * If you don't have an ISO7816 contactor with card insert detection, please
 * define SMARTCARD_CONTACT_ACTIVE to 0.
 */
#define SMARTCARD_CONTACT_ACTIVE            	1
#define SMARTCARD_CONTACT_INSERT_HIGH

#if defined(SMARTCARD_CONTACT_ACTIVE)
#define SMARTCARD_CONTACT_PORT                  GPIOE
#define SMARTCARD_CONTACT_PIN                   LL_GPIO_PIN_2
#define SMARTCARD_CONTACT_SYSCFG_EXTI_PORT      LL_SYSCFG_EXTI_PORTE
#define SMARTCARD_CONTACT_SYSCFG_EXTI_LINE      LL_SYSCFG_EXTI_LINE2
#define SMARTCARD_CONTACT_EXTI_LINE             LL_EXTI_LINE_2
#define SMARTCARD_CONTACT_IRQn                  EXTI2_IRQn
#define SMARTCARD_CONTACT_IRQHANDLER            EXTI2_IRQHandler
#endif

/////////////////////////////// DEBUG //////////////////////////////////////////

#define DEBUG_UART                          USART1
#define DEBUG_UART_ENABLE_CLOCK()           LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1)
#define DEBUG_GPIO_ENABLE_CLOCK()           LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB)
#define DEBUG_GPIO_PIN                      LL_GPIO_PIN_6
#define DEBUG_GPIO_PORT                     GPIOB
#define DEBUG_GPIO_AF                       LL_GPIO_AF_7
#define DEBUG_TX_RINGBUFF_SIZE              4096

////////////////////////////// CONSOLE /////////////////////////////////////////

#define CONSOLE_UART                        UART4
#define CONSOLE_UART_IRQ                    UART4_IRQHandler
#define CONSOLE_UART_IRQn                   UART4_IRQn
#define CONSOLE_UART_CLOCK                  LL_APB1_GRP1_PERIPH_UART4
#define CONSOLE_GPIO_CLOCK                  LL_AHB1_GRP1_PERIPH_GPIOC
#define CONSOLE_DMA_CLOCK                   LL_AHB1_GRP1_PERIPH_DMA1
#define CONSOLE_UART_ENABLE_CLOCK()         LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART4)
#define CONSOLE_GPIO_ENABLE_CLOCK()         LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC)
#define CONSOLE_DMA_ENABLE_CLOCK()          LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1)
#define CONSOLE_TX_GPIO_PIN                 LL_GPIO_PIN_10
#define CONSOLE_TX_GPIO_PORT                GPIOC
#define CONSOLE_TX_GPIO_AF                  LL_GPIO_AF_8
#define CONSOLE_RX_GPIO_PIN                 LL_GPIO_PIN_11
#define CONSOLE_RX_GPIO_PORT                GPIOC
#define CONSOLE_RX_GPIO_AF                  LL_GPIO_AF_8
#define CONSOLE_DMA                         DMA1
#define CONSOLE_DMA_CHANNEL                 LL_DMA_CHANNEL_4
#define CONSOLE_DMA_RX_STREAM               LL_DMA_STREAM_2
#define CONSOLE_DMA_TX_STREAM               LL_DMA_STREAM_4
#define CONSOLE_DMA_RX_IRQ                  DMA1_Stream2_IRQHandler
#define CONSOLE_DMA_RX_IRQn                 DMA1_Stream2_IRQn
#define CONSOLE_DMA_TX_IRQ                  DMA1_Stream4_IRQHandler
#define CONSOLE_DMA_TX_IRQn                 DMA1_Stream4_IRQn
#define CONSOLE_TX_RINGBUFF_SIZE            16500
#define CONSOLE_RX_RINGBUFF_SIZE            CONSOLE_TX_RINGBUFF_SIZE
#define CONSOLE_RX_DMA_SIZE                 64

/////////////////////////////// LEDS ///////////////////////////////////////////

#define LEDS_ERROR_GPIO_PORT                GPIOD
#define LEDS_ERROR_GPIO_PIN                 LL_GPIO_PIN_12
#define LEDS_STATUS_GPIO_PORT               GPIOD
#define LEDS_STATUS_GPIO_PIN                LL_GPIO_PIN_15
#define LEDS_GPIO_ENABLE_CLOCK()            LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD)

///////////////////////////// TRIGGER //////////////////////////////////////////

// Our board trigger
#define TRIG_GPIO_PORT                      GPIOA
#define TRIG_GPIO_PIN                       LL_GPIO_PIN_1
#define TRIG_GPIO_ENABLE_CLOCK()            LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA)
// The ChipWhisperer trigger
#define TRIG_CW_GPIO_PORT                   GPIOE
#define TRIG_CW_GPIO_PIN                    LL_GPIO_PIN_10
#define TRIG_CW_GPIO_ENABLE_CLOCK()         LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE)

#endif

#endif // __DISCO_CONFIG_H
