/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "debug.h"
#include "console.h"
#include "protocol.h"
#include "leds.h"
#include "smartcard.h"
#include "smartcard_iso7816_platform.h"
/* USER CODE END Includes */

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(uint8_t);

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

extern USBD_HandleTypeDef hUsbDeviceFS;
extern volatile uint32_t dfu_mode_selected;

static inline void GPIO_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOG);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOI);
#ifndef DISCO407
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOJ);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOK);
#endif
    return;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    if(dfu_mode_selected == DFU_MAGIC){
        /* When DFU mode is selected, we have to force HSE */
        SystemClock_Config(1);
    }
    else{
        /* Do not force HSE when DFU mode is not selected */
        SystemClock_Config(0);
    }
  
  
    /* Initialize GPIO clocks */
    GPIO_init();

    /* Initialize debug */
    dbg_init();

    /* Do we have to go to DFU mode? */
    if(dfu_mode_selected == DFU_MAGIC){
        dfu_mode_selected = NO_DFU_MAGIC;

        dbg_log("Going to DFU mode!\r\n");
        dbg_flush();
        HAL_Delay(50);
        /* Goto DFU mode with inline assembly through the BootROM */
        asm volatile ("LDR R0, =0x40023844 \r\n"  /* RCC_APB2ENR */  
                      "LDR R1, =0x00004000 \r\n"  /* ENABLE SYSCFG CLOCK */
                      "STR R1, [R0, #0]\r\n"        
                      "LDR R0, =0x40013800 \r\n"  /* remap ROM at zero */
                      "LDR R1, =0x00000001 \r\n"  /* SYSCFG_MEMRMP */
                      "STR R1, [R0, #0]\r\n"        
                      "LDR R0, =0x1FFF0000 \r\n"  /* ROM BASE */   
                      "LDR SP,[R0, #0] \r\n"      /* SP @ +0 */  
                      "LDR R0,[R0, #4] \r\n"      /* PC @ +4 */  
                      "BX R0\r\n");               /* Jump to Bootloader */
       /* We should not continue execution past this line */
       while(1){};
    }

#ifdef USB
    /* Initialize console and USB */
    MX_USB_DEVICE_Init();
#endif
    console_init();

    /* Initialize all configured peripherals */
    led_init();

    dbg_log("======== LEIA FIRMWARE =========\r\n");
    dbg_log("\tBuilt date\t: %s at %s\r\n", __DATE__, __TIME__);
#if defined(LEIA)
    dbg_log("\tBoard\t\t: LEIA\r\n");
#elif defined(WOOKEY)
    dbg_log("\tBoard\t\t: WooKey\r\n");
#elif defined(DISCO)
    dbg_log("\tBoard\t\t: DISCO\r\n");
#if defined(DISCO407)
    dbg_log("\t\t\t (DISCO407)\r\n");
#else
    dbg_log("\t\t\t (DISCO429)\r\n");
#endif
#else
    #error "Unknown board!!"
#endif
#ifdef ISO7816_BITBANG
#if defined(WOOKEY)
    dbg_log("\tISO7816-3 using Bitbang is *NOT supported* on the WooKey board for now!!\r\n");
    while(1){};
#endif
    dbg_log("\tISO7816-3 using Bitbang\r\n");
#else
    dbg_log("\tISO7816-3 using accelerated USART\r\n");
#endif
    dbg_log("================================\r\n");
    dbg_flush();

    dbg_log("Hello from debug.\r\n");
    dbg_flush();

    led_startup();
    led_startup();

    platform_init_smartcard_contact();

    /* Parse a command */
    protocol_parse_cmd();

    return 0;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(uint8_t force_HSE)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the CPU, AHB and APB busses clocks
    */
#if defined(LEIA)
    /* When using LEIA or DISCO, we always use HSE */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
#ifndef HSE_PLLM_OVERRIDE
    /* On LEIA SOLO, we have a 16 MHz quartz */
    RCC_OscInitStruct.PLL.PLLM = 16;
#else
    RCC_OscInitStruct.PLL.PLLM = HSE_PLLM_OVERRIDE;
#endif
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
#endif
#if defined(DISCO)
    /* When using DISCO, we always use HSE */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
#ifndef HSE_PLLM_OVERRIDE
    /* Disco external quartz is 8 MHz */
    RCC_OscInitStruct.PLL.PLLM = 8;
#else
    RCC_OscInitStruct.PLL.PLLM = HSE_PLLM_OVERRIDE;
#endif
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
#endif
#ifdef WOOKEY
    /* When using WooKey, we use HSI for nominal mode but force
     * HSE when booting in DFU mode (BootROM constraints it)
     */
    if(force_HSE == 1){
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
#ifndef HSE_PLLM_OVERRIDE
        RCC_OscInitStruct.PLL.PLLM = 16;
#else
	RCC_OscInitStruct.PLL.PLLM = HSE_PLLM_OVERRIDE;
#endif
        RCC_OscInitStruct.PLL.PLLN = 336;
        RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
        RCC_OscInitStruct.PLL.PLLQ = 7;
    }
    else{
        RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
        RCC_OscInitStruct.HSIState = RCC_HSI_ON;
        RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
        RCC_OscInitStruct.PLL.PLLM = 8;
        RCC_OscInitStruct.PLL.PLLN = 168;
        RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
        RCC_OscInitStruct.PLL.PLLQ = 7;
    }
#endif

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    dbg_log("Error handler called\r\n");
    dbg_flush();
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
