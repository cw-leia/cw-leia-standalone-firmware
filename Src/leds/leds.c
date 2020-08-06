/*
 * This module can be used to init LEIA leds and blink them.
 */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32f4xx_ll_gpio.h"
#include "leds.h"
#include "debug.h"

void
led_init(void)
{

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    LEDS_GPIO_ENABLE_CLOCK();


    /*Configure GPIO pin Output Level */
    led_status_off();
    led_error_off();

    /*Configure GPIO pins */
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;

    /* For error */
    GPIO_InitStruct.Pin = LEDS_ERROR_GPIO_PIN;
    LL_GPIO_Init(LEDS_ERROR_GPIO_PORT, &GPIO_InitStruct);

    /* And status pin */
    GPIO_InitStruct.Pin = LEDS_STATUS_GPIO_PIN;
    LL_GPIO_Init(LEDS_STATUS_GPIO_PORT, &GPIO_InitStruct);

}

void
led_status_on()
{
    LL_GPIO_SetOutputPin(LEDS_STATUS_GPIO_PORT, LEDS_STATUS_GPIO_PIN);
}

void
led_status_off()
{
    LL_GPIO_ResetOutputPin(LEDS_STATUS_GPIO_PORT, LEDS_STATUS_GPIO_PIN);
}

void
led_status_toggle()
{
    LL_GPIO_TogglePin(LEDS_STATUS_GPIO_PORT, LEDS_STATUS_GPIO_PIN);
}

void
led_error_on()
{
    LL_GPIO_SetOutputPin(LEDS_ERROR_GPIO_PORT, LEDS_ERROR_GPIO_PIN);
}

void
led_error_off()
{
    LL_GPIO_ResetOutputPin(LEDS_ERROR_GPIO_PORT, LEDS_ERROR_GPIO_PIN);
}

void
led_error_toggle()
{
    LL_GPIO_TogglePin(LEDS_ERROR_GPIO_PORT, LEDS_ERROR_GPIO_PIN);
}

void
led_startup(void)
{
    uint8_t i;
    for(i = 0; i < 5; i++) {
        led_status_on();
        HAL_Delay(50);
        led_status_off();
        HAL_Delay(50);
    }

}
