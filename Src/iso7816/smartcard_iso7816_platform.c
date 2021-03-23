#if !defined(ISO7816_BITBANG)

#include "smartcard_iso7816_platform.h"
#include "smartcard_print.h"
#include "smartcard.h"
#include "helpers.h"

#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_tim.h"

#include "config.h"
#include "leds.h"
#include "triggers.h"

/* The target clock frequency is 3.5MHz for the ATR < max 5MHz.
 * The advantage of this frequency is that it is a perfect divisor of
 * our USARTS core drequencies:
 *  - For USART 1 and 6: 84 MHz / 3.5 MHz = 24
 *  - For USART 2, 3 and 4: 42 MHz / 3.5 MHz = 12
 * NB: you can change this default frequency to 4.2 MHz or 5.25 MHz which are
 * the next divisors when using our USART 2 core frequency. This should work with
 * most of the cards while being faster, but we choose to keep 3.5 MHz mainly as a
 * conservative choice.
 *
 * NB2: this default frequency is a priori only used during the ATR, and a faster communication
 * could/should be established with the card after parsing the ATR and/or negotiating with the
 * smartcard.
 */

/* Smartcard USART configuration: we use USART 2: (TX = I/O = PA2, CLK = PA4)
 * We use the SMARTCARD mode when configuring the GPIOs.
 * The baudrate is set to 9408 bauds (see the explanations in the smartcard_init function).
 * The parameters are set to meet the requirements in the datasheet 24.3.11 section 'Smartcard':
 *  - LINEN bit in the USART_CR2 register cleared.
 *  - HDSEL and IREN bits in the USART_CR3 register cleared.
 *  - Moreover, the CLKEN bit may be set in order to provide a clock to the smartcard.
 *
 * The Smartcard interface is designed to support asynchronous protocol Smartcards as
 * defined in the ISO 7816-3 standard. The USART should be configured as:
 *  - 8 bits plus parity: where M=1 and PCE=1 in the USART_CR1 register
 *  - 1.5 stop bits when transmitting and receiving: where STOP=11 in the USART_CR2 register.
 */

/* Smartcard uses USART 2, i.e. I/O is on PA2 and CLK is on PA4 */
#define SMARTCARD_USART                 USART2

/* The USART we use for smartcard.
 * STM32F4 provides the I/O pin on the TX USART pin, and
 * the CLK pin on the dedicated USART CK pin.
 * The RST (reset) pin uses a dedicated GPIO.
 * The card detect pin uses a dedicated GPIO.
 * The VCC and VPP pins use dedicated GPIOs.
 */



typedef struct {
    USART_TypeDef* instance;
    LL_USART_InitTypeDef config;
    LL_USART_ClockInitTypeDef clock_config;

} PLATFORM_SMARTCARD_TypeDef;

const LL_USART_InitTypeDef default_smartcard_usart_config = {
    /* To be filled later depending on the clock configuration */
    .BaudRate               = 0,
    /* Word length = 9 bits (8 bits + parity) */
    .DataWidth              = LL_USART_DATAWIDTH_9B,
    /* 1 stop bit instead of 1.5 is necessary for some cards that use a very short delay between USART characters ... */
    .StopBits               = LL_USART_STOPBITS_1,
    /* 8 bits plus parity  (parity even) */
    .Parity                 = LL_USART_PARITY_EVEN,
    /* Hardware flow control disabled, */
    .HardwareFlowControl    = LL_USART_HWCONTROL_NONE,
    /* TX and RX are enabled */
    .TransferDirection      = LL_USART_DIRECTION_TX_RX
};

/* Current smartcard USART config: it is not const since it can be changed
 * depending on the detected smartcard.
 */
PLATFORM_SMARTCARD_TypeDef smartcard_usart_config = {
    .instance       = SMARTCARD_USART,
    .config         = {0},
    .clock_config   = {0}
};


void set_usart_config_default(PLATFORM_SMARTCARD_TypeDef* smartcard_usart_config, const LL_USART_InitTypeDef default_config)
{
    smartcard_usart_config->config.BaudRate = default_config.BaudRate;
    smartcard_usart_config->config.DataWidth = default_config.DataWidth;
    smartcard_usart_config->config.StopBits = default_config.StopBits;
    smartcard_usart_config->config.Parity = default_config.Parity;
    smartcard_usart_config->config.HardwareFlowControl = default_config.HardwareFlowControl;
    smartcard_usart_config->config.TransferDirection = default_config.TransferDirection;

}

#ifdef SMARTCARD_IO_DIR_PIN
LL_GPIO_InitTypeDef gpio_smartcard_io_dir = {
    .Pin        = SMARTCARD_IO_DIR_PIN,
    .Mode       = LL_GPIO_MODE_OUTPUT,
    .Pull       = LL_GPIO_PULL_NO,
    .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
};
#endif

LL_GPIO_InitTypeDef gpio_smartcard_io = {
    .Pin        = SMARTCARD_IO_PIN,
    .Mode       = LL_GPIO_MODE_ALTERNATE,
    .Pull       = LL_GPIO_PULL_UP,
    .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_OPENDRAIN,
    .Alternate  = SMARTCARD_IO_AF
};

LL_GPIO_InitTypeDef gpio_smartcard_clk = {
    .Pin        = SMARTCARD_CLK_PIN,
    .Mode       = LL_GPIO_MODE_ALTERNATE,
    .Pull       = LL_GPIO_PULL_UP,
    .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
    .Alternate  = SMARTCARD_CLK_AF
};

LL_GPIO_InitTypeDef gpio_smartcard_rst = {
    .Pin        = SMARTCARD_RST_PIN,
    .Mode       = LL_GPIO_MODE_OUTPUT,
    .Pull       = LL_GPIO_PULL_DOWN,
    .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
};

LL_GPIO_InitTypeDef gpio_smartcard_vcc = {
    .Pin        = SMARTCARD_VCC_PIN,
    .Mode       = LL_GPIO_MODE_OUTPUT,
    .Pull       = LL_GPIO_PULL_DOWN,
    .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
};

LL_GPIO_InitTypeDef gpio_smartcard_vpp = {
    .Pin        = SMARTCARD_VPP_PIN,
    .Mode       = LL_GPIO_MODE_OUTPUT,
    .Pull       = LL_GPIO_PULL_DOWN,
    .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
};

/*** Smartcard auxiliary RFU pins (unused for now) ****/
LL_GPIO_InitTypeDef gpio_smartcard_aux1 = {
    .Pin        = SMARTCARD_AUX1_PIN,
    .Mode       = LL_GPIO_MODE_OUTPUT,
    .Pull       = LL_GPIO_PULL_DOWN,
    .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
};

LL_GPIO_InitTypeDef gpio_smartcard_aux2 = {
    .Pin        = SMARTCARD_AUX2_PIN,
    .Mode       = LL_GPIO_MODE_OUTPUT,
    .Pull       = LL_GPIO_PULL_DOWN,
    .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
};


/* Contact Pin --------------------------------------------------------------*/

#if (SMARTCARD_CONTACT_ACTIVE == 1)
LL_GPIO_InitTypeDef gpio_smartcard_contact = {
    .Pin        = SMARTCARD_CONTACT_PIN,
    .Mode       = LL_GPIO_MODE_INPUT,
    .Pull       = LL_GPIO_PULL_NO,
    .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_OPENDRAIN,
};
#endif

/* Initialize the CONTACT pin */
static volatile uint8_t platform_SC_gpio_smartcard_contact_changed = 0;
static volatile uint8_t platform_SC_is_smartcard_inserted = 0;

uint8_t platform_SC_is_lost(void) {
    return ((~platform_SC_is_smartcard_inserted) & 0x1);
}

#if (SMARTCARD_CONTACT_ACTIVE == 1)
SC_Card* get_current_card();
void SMARTCARD_CONTACT_IRQHANDLER(void)
{
    if (LL_EXTI_IsActiveFlag_0_31(SMARTCARD_CONTACT_EXTI_LINE) != RESET) {
        LL_EXTI_ClearFlag_0_31(SMARTCARD_CONTACT_EXTI_LINE);

#if (SMARTCARD_CONTACT_ACTIVE == 1)
#if defined(SMARTCARD_CONTACT_INSERT_HIGH)
        platform_SC_is_smartcard_inserted = LL_GPIO_IsInputPinSet(SMARTCARD_CONTACT_PORT, SMARTCARD_CONTACT_PIN)  & 1;
#else
        platform_SC_is_smartcard_inserted = (~LL_GPIO_IsInputPinSet(SMARTCARD_CONTACT_PORT, SMARTCARD_CONTACT_PIN))  & 1;
#endif
#else
        platform_SC_is_smartcard_inserted = 1;
#endif
        /* Handle our LED */
        /* Activate LED or not depending on smartcard presence */
        if(platform_SC_is_smartcard_inserted) {
            led_status_on();
        } else {
            led_status_off();
        }
    }

    platform_SC_gpio_smartcard_contact_changed = 1;

    return;
}
#endif

void platform_init_smartcard_contact(void)
{
    platform_SC_gpio_smartcard_contact_changed = 0;

#if (SMARTCARD_CONTACT_ACTIVE == 1)
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_GPIO_Init(SMARTCARD_CONTACT_PORT, &gpio_smartcard_contact);

    LL_SYSCFG_SetEXTISource(SMARTCARD_CONTACT_SYSCFG_EXTI_PORT,
                            SMARTCARD_CONTACT_SYSCFG_EXTI_LINE);
    LL_EXTI_InitTypeDef EXTI_InitStruct;
    EXTI_InitStruct.Line_0_31 = SMARTCARD_CONTACT_EXTI_LINE;
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING_FALLING;
    LL_EXTI_Init(&EXTI_InitStruct);
    /* EXTI interrupt init*/
    NVIC_SetPriority(SMARTCARD_CONTACT_IRQn, 0);
    NVIC_EnableIRQ(SMARTCARD_CONTACT_IRQn);

    platform_SC_gpio_smartcard_contact_changed = 1;

#if defined(SMARTCARD_CONTACT_INSERT_HIGH)
    platform_SC_is_smartcard_inserted = LL_GPIO_IsInputPinSet(SMARTCARD_CONTACT_PORT, SMARTCARD_CONTACT_PIN)  & 1;
#else
    platform_SC_is_smartcard_inserted = (~LL_GPIO_IsInputPinSet(SMARTCARD_CONTACT_PORT, SMARTCARD_CONTACT_PIN))  & 1;
#endif
#else
    platform_SC_is_smartcard_inserted = 1;
#endif
    /* Handle our LED */
    /* Activate LED or not depending on smartcard presence */
    if(platform_SC_is_smartcard_inserted) {
        led_status_on();
    } else {
        led_status_off();
    }

    return;
}

void platform_SC_reinit_smartcard_contact(void)
{
    //platform_SC_is_smartcard_inserted = 0;
    platform_SC_gpio_smartcard_contact_changed = 1;

    return;
}

/* The SMARTCARD_CONTACT pin is at state high (pullup to Vcc) when no card is
 * not present, and at state low (linked to GND) when the card is inserted.
 */
uint8_t platform_is_smartcard_inserted(void)
{
#if (SMARTCARD_CONTACT_ACTIVE == 1)

    if(platform_SC_gpio_smartcard_contact_changed == 1) {
#if defined(SMARTCARD_CONTACT_INSERT_HIGH)
        platform_SC_is_smartcard_inserted = LL_GPIO_IsInputPinSet(SMARTCARD_CONTACT_PORT, SMARTCARD_CONTACT_PIN)  & 1;
#else
        platform_SC_is_smartcard_inserted = (~LL_GPIO_IsInputPinSet(SMARTCARD_CONTACT_PORT, SMARTCARD_CONTACT_PIN))  & 1;
#endif

        platform_SC_gpio_smartcard_contact_changed = 0;
    }

    return platform_SC_is_smartcard_inserted;
#else
    return 1;
#endif
}

/* IO Dir -------------------------------------------------------------------*/

static inline void platform_io_dir_read(void)
{
#ifdef SMARTCARD_IO_DIR_PORT
    LL_GPIO_ResetOutputPin(SMARTCARD_IO_DIR_PORT, SMARTCARD_IO_DIR_PIN);
#endif
}

static inline void platform_io_dir_write(void)
{
#ifdef SMARTCARD_IO_DIR_PORT
    LL_GPIO_SetOutputPin(SMARTCARD_IO_DIR_PORT, SMARTCARD_IO_DIR_PIN);
#endif
}

void platform_init_io_dir(void)
{
#ifdef SMARTCARD_IO_DIR_PORT
    LL_GPIO_Init(SMARTCARD_IO_DIR_PORT, &gpio_smartcard_io_dir);
    platform_io_dir_read();
#endif
}

/* RST ----------------------------------------------------------------------*/

/* Initialize the RST pin */
void platform_init_smartcard_rst(void)
{
    LL_GPIO_Init(SMARTCARD_RST_PORT, &gpio_smartcard_rst);

    /* Maintain the RST at low */
    LL_GPIO_ResetOutputPin(SMARTCARD_RST_PORT, SMARTCARD_RST_PIN);

    return;
}

void platform_set_smartcard_rst(uint8_t val)
{
    if (val == 0) {
        LL_GPIO_ResetOutputPin(SMARTCARD_RST_PORT, SMARTCARD_RST_PIN);

    } else {
        LL_GPIO_SetOutputPin(SMARTCARD_RST_PORT, SMARTCARD_RST_PIN);
    }

    return;
}

/* VCC ----------------------------------------------------------------------*/

/* Initialize the Vcc pin */
void platform_init_smartcard_vcc(void)
{
    LL_GPIO_Init(SMARTCARD_VCC_PORT, &gpio_smartcard_vcc);

    /* For now, maintain the Vcc at zero (low) */
#ifdef LEIA
    LL_GPIO_SetOutputPin(SMARTCARD_VCC_PORT, SMARTCARD_VCC_PIN);
#else
    LL_GPIO_ResetOutputPin(SMARTCARD_VCC_PORT, SMARTCARD_VCC_PIN);
#endif

    return;
}

void platform_set_smartcard_vcc(uint8_t val)
{
#ifdef LEIA
    // Invert
    val = (~val) & 0x1;
#endif

    if (val == 0) {
        LL_GPIO_ResetOutputPin(SMARTCARD_VCC_PORT, SMARTCARD_VCC_PIN);

    } else {
        LL_GPIO_SetOutputPin(SMARTCARD_VCC_PORT, SMARTCARD_VCC_PIN);
    }

    return;
}

/* VPP ----------------------------------------------------------------------*/

/* Initialize the Vpp pin */
void platform_init_smartcard_vpp(void)
{
    LL_GPIO_Init(SMARTCARD_VPP_PORT, &gpio_smartcard_vpp);

    /* Vpp is set low. Not used for now */
#ifdef LEIA
    LL_GPIO_SetOutputPin(SMARTCARD_VPP_PORT, SMARTCARD_VPP_PIN);
#else
    LL_GPIO_ResetOutputPin(SMARTCARD_VPP_PORT, SMARTCARD_VPP_PIN);
#endif
    return;
}

void platform_set_smartcard_vpp(uint8_t val)
{
#ifdef LEIA
    // Invert
    val = (~val) & 0x1;
#endif

    if (val == 0) {
        LL_GPIO_ResetOutputPin(SMARTCARD_VPP_PORT, SMARTCARD_VPP_PIN);

    } else {
        LL_GPIO_SetOutputPin(SMARTCARD_VPP_PORT, SMARTCARD_VPP_PIN);
    }

    return;
}


/* Initialize the IO/CLK pins */
void
platform_init_smartcard_ioclk(void)
{
    LL_GPIO_Init(SMARTCARD_IO_PORT, &gpio_smartcard_io);
    LL_GPIO_Init(SMARTCARD_CLK_PORT, &gpio_smartcard_clk);
}



static int platform_get_bus_clock(USART_TypeDef* USARTx, uint32_t* bus_clock);

int platform_smartcard_clock_rounding(uint32_t target_freq, uint32_t* target_freq_rounded)
{
    uint32_t usart_bus_clk;
    unsigned int i;

    if(target_freq_rounded == NULL) {
        goto err;
    }

    /* Get the usart clock */
    if (platform_get_bus_clock(SMARTCARD_USART, &usart_bus_clk) != 0) {
        goto err;
    }

    log_printf("\t  usart_bus_clk=%d\r\n", usart_bus_clk);

    /* Find the best suitable target frequency <= target frequency with regards to our USART core frequency
     * (i.e. as a divisor of our USART core frequency).
     */
    if(target_freq > usart_bus_clk) {
        /* The target frequency is > USART frequency: there is no need to try ... */
        goto err;
    }

    i = target_freq;

    while(i != 0) {
        if(((usart_bus_clk / i / 2) * i * 2) == usart_bus_clk) {
            break;
        }

        i--;
    }

    log_printf("\t  target_freq_rounded=%d\r\n", i);
    *target_freq_rounded = i;

    return 0;

err:

    return -1;

}

static int platform_get_bus_clock(USART_TypeDef* USARTx, uint32_t* bus_clock)
{

    uint32_t periphclk = LL_RCC_PERIPH_FREQUENCY_NO;
    LL_RCC_ClocksTypeDef rcc_clocks;

    LL_RCC_GetSystemClocksFreq(&rcc_clocks);

    if (USARTx == USART1) {
        periphclk = rcc_clocks.PCLK2_Frequency;

    } else if (USARTx == USART2) {
        periphclk = rcc_clocks.PCLK1_Frequency;
    }

#if defined(USART3)

    else if (USARTx == USART3) {
        periphclk = rcc_clocks.PCLK1_Frequency;
    }

#endif /* USART3 */
#if defined(USART6)

    else if (USARTx == USART6) {
        periphclk = rcc_clocks.PCLK2_Frequency;
    }

#endif /* USART6 */
#if defined(UART4)

    else if (USARTx == UART4) {
        periphclk = rcc_clocks.PCLK1_Frequency;
    }

#endif /* UART4 */
#if defined(UART5)

    else if (USARTx == UART5) {
        periphclk = rcc_clocks.PCLK1_Frequency;
    }

#endif /* UART5 */
#if defined(UART7)

    else if (USARTx == UART7) {
        periphclk = rcc_clocks.PCLK1_Frequency;
    }

#endif /* UART7 */
#if defined(UART8)

    else if (USARTx == UART8) {
        periphclk = rcc_clocks.PCLK1_Frequency;
    }

#endif /* UART8 */
#if defined(UART9)

    else if (USARTx == UART9) {
        periphclk = rcc_clocks.PCLK2_Frequency;
    }

#endif /* UART9 */
#if defined(UART10)

    else if (USARTx == UART10) {
        periphclk = rcc_clocks.PCLK2_Frequency;
    }

#endif

    if (periphclk == LL_RCC_PERIPH_FREQUENCY_NO) {
        goto err;
    }

    *bus_clock = periphclk;

    return 0;

err:
    log_printf("Error in platform_get_bus_clock\r\n");
    return -1;
}

static volatile uint8_t platform_SC_pending_receive_byte = 0;
static volatile uint8_t platform_SC_pending_send_byte = 0;
static volatile uint8_t platform_SC_byte = 0;

typedef enum {
    GPIOS_EN  = 0,
    USART_EN  = 1,
    ALL_EN    = 2,
    USART_DIS = 3,
} smartcard_init_type;



static inline int platform_get_usart_number(USART_TypeDef* USARTx)
{
#if defined(USART1)

    if (USARTx == USART1) {
        return 1;
    }

#endif

#if defined(USART2)

    if (USARTx == USART2) {
        return 2;
    }

#endif

#if defined(USART3)

    if (USARTx == USART3) {
        return 3;
    }

#endif

#if defined(USART6)

    if (USARTx == USART6) {
        return 6;
    }

#endif

    log_printf("Error in platform_get_usart_number\r\n");

    return 0;
}

static void DWT_Init(void);
static void setup_microsec_timer(void);
static volatile uint8_t setup_microsec_timer_done = 0;

static int platform_smartcard_hw_init(smartcard_init_type type)
{
    if((type == GPIOS_EN) || (type == ALL_EN)) {
        /* Initialize the GPIOs */
        platform_init_smartcard_contact();
        platform_init_smartcard_vcc();
        platform_init_smartcard_vpp();
        platform_init_smartcard_rst();
        platform_init_smartcard_ioclk();
        platform_init_io_dir();
    }

    if((type == USART_EN) || (type == ALL_EN)) {
        /* Initialize the USART in smartcard mode */
        LL_USART_Disable(SMARTCARD_USART);
        log_printf("\t==> Init USART%d in smartcard mode!\r\n", platform_get_usart_number(SMARTCARD_USART));
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
        LL_USART_Init(SMARTCARD_USART, &smartcard_usart_config.config);
        LL_USART_ClockInit(SMARTCARD_USART, &smartcard_usart_config.clock_config);
        LL_USART_Enable(SMARTCARD_USART);
        LL_USART_ConfigSyncMode(SMARTCARD_USART);
        LL_USART_EnableSmartcard(SMARTCARD_USART);
        LL_USART_EnableSmartcardNACK(SMARTCARD_USART);
        LL_USART_EnableSCLKOutput(SMARTCARD_USART);
        LL_USART_SetLastClkPulseOutput(SMARTCARD_USART, LL_USART_LASTCLKPULSE_OUTPUT);
        LL_USART_SetClockPolarity(SMARTCARD_USART, LL_USART_POLARITY_LOW);
        LL_USART_SetClockPhase(SMARTCARD_USART, LL_USART_PHASE_1EDGE);

        LL_USART_EnableIT_PE(SMARTCARD_USART);
        LL_USART_EnableIT_RXNE(SMARTCARD_USART);
        LL_USART_EnableIT_ERROR(SMARTCARD_USART);
        LL_USART_EnableIT_TC(SMARTCARD_USART);
        LL_USART_Enable(SMARTCARD_USART);
        LL_USART_ClearFlag_TC(SMARTCARD_USART);
        LL_USART_ClearFlag_RXNE(SMARTCARD_USART);

        /* USART interrupt */
        NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
        NVIC_EnableIRQ(USART2_IRQn);

        log_printf("\t==> Enable OK\r\n");
    }

    if(type == USART_DIS) {
        /* Disable the smartcard USART */
        LL_USART_Disable(SMARTCARD_USART);
    }
    /* Initialize time measurement */
    if(setup_microsec_timer_done == 0){
        DWT_Init();
        setup_microsec_timer();
        setup_microsec_timer_done = 1;
    }

    return 0;
}

int platform_smartcard_init(void)
{

    /* Reinitialize global variables */
    platform_SC_pending_receive_byte = 0;
    platform_SC_pending_send_byte = 0;
    platform_SC_byte = 0;

    /* Reset our smartcard USART configuration */

    set_usart_config_default(&smartcard_usart_config, default_smartcard_usart_config);

    /* Disable USART block to reinitialize its state and flush the
     * buffers.
     */
    platform_smartcard_hw_init(USART_DIS);

    return platform_smartcard_hw_init(ALL_EN);
}

/* Adapt clocks and guard time depending on what has been received */
int platform_SC_adapt_clocks(uint32_t* etu, uint32_t* frequency)
{
    uint32_t guard_time, usart_bus_clk;
    uint32_t prescaler;

    if((etu == NULL) || (frequency == NULL)) {
        goto err;
    }

    if(platform_smartcard_clock_rounding(*frequency, frequency) != 0) {
        goto err;
    }

    log_printf("\tRounding target freguency to %d\r\n", *frequency);

    /* First, get the usart clock */
    if (platform_get_bus_clock(SMARTCARD_USART, &usart_bus_clk) != 0) {
        goto err;
    }

    /* Then, compute the baudrate depending on the target frequency */
    /* Baudrate is the clock frequency divided by one ETU (372 ticks by default, possibly negotiated).
     * For example, a frequency of 3.5MHz gives 3.5MHz / 372 ~= 9408 bauds for a 372 ticks ETU. */
    smartcard_usart_config.config.BaudRate = (*frequency) / (*etu);

    /* Finally, adapt the CLK clock pin frequency to the target frequency using the prescaler.
     * Also, adapt the guard time (expressed in bauds).
     * Frequency is = (APB_clock / PRESCALER) = (42MHz / 12) = 3.5MHz. The value of the prescaler field is x2 (cf. datasheet).
     */
    prescaler = (usart_bus_clk / (*frequency));
    prescaler /= 2;
    guard_time = 1;
    //config->guard_time_prescaler = ((prescaler / 2) << USART_GTPR_PSC_Pos) | (target_guard_time << USART_GTPR_GT_Pos);

    /* Adapt the configuration at the USART level */
    LL_USART_SetSmartcardPrescaler(SMARTCARD_USART, prescaler);
    LL_USART_SetSmartcardGuardTime(SMARTCARD_USART, guard_time);
    LL_USART_SetBaudRate(SMARTCARD_USART, usart_bus_clk, smartcard_usart_config.config.OverSampling, smartcard_usart_config.config.BaudRate);
    //usart_init(&smartcard_usart_config);

    return 0;

err:
    return -1;
}

static inline uint8_t platform_SC_char_pop(void);
static inline void platform_SC_char_push(uint8_t c);

/*
 * Low level related functions: we handle the low level USAT/smartcard
 * bytes send and receive stuff here.
 */
static volatile uint8_t dummy_usart_read = 0;


void USART2_IRQHandler(void)
{
    if(LL_USART_IsActiveFlag_PE(SMARTCARD_USART) && (platform_SC_pending_send_byte != 0)) {
        /* Parity error, program a resend */
        platform_SC_pending_send_byte = 3;
        /* Dummy read of the DR register to ACK the interrupt */
        LL_USART_ClearFlag_PE(SMARTCARD_USART);
        return;
    }

    /* Check if we have a framing error */
    if(LL_USART_IsActiveFlag_FE(SMARTCARD_USART) && (platform_SC_pending_send_byte != 0)) {
        /* Frame error, program a resend */
        platform_SC_pending_send_byte = 4;
        /* Dummy read of the DR register to ACK the interrupt */
        LL_USART_ClearFlag_FE(SMARTCARD_USART);
        return;
    }

    /* We have sent our byte */
    if(LL_USART_IsActiveFlag_TC(SMARTCARD_USART) && (platform_SC_pending_send_byte != 0)) {
        /* Signal that the byte has been sent */
        platform_SC_pending_send_byte = 2;
        /* Clear TC */
        LL_USART_ClearFlag_TC(SMARTCARD_USART);
        /* Declare the event to the trig management */
        trig(TRIG_IRQ_PUTC);
        return;
    }

    /* We can actually read data */
    if(LL_USART_IsActiveFlag_RXNE(SMARTCARD_USART)) {
        platform_SC_byte = platform_SC_char_pop();

        if(platform_SC_pending_send_byte == 0) {
            trig(TRIG_IRQ_GETC);
            platform_SC_pending_receive_byte = 1;
        }

        return;
    }

    return;
}

/* Set the unkown convention at low level */
int platform_SC_set_unkown_conv(void)
{

    return 0;
}

/* Set the direct convention at low level */
int platform_SC_set_direct_conv(void)
{

    return 0;
}

/* Set the inverse convention at low level */
int platform_SC_set_inverse_conv(void)
{
    /* ACK the pending parity errors */ 
    LL_USART_ClearFlag_PE(SMARTCARD_USART);
    /* Adapt the configuration at the USART level */
    smartcard_usart_config.config.Parity = LL_USART_PARITY_ODD;
    LL_USART_Disable(SMARTCARD_USART);
    LL_USART_Enable(SMARTCARD_USART);
    LL_USART_SetParity(SMARTCARD_USART, smartcard_usart_config.config.Parity); 
 
    return 0;
}

/* Low level flush of our receive/send state, in order
 * for the higher level to be sure that everything is clean
 */
void platform_SC_flush(void)
{
    /* Flushing the receive/send state is only a matter of cleaning
     * our ring buffer!
     */
    platform_SC_pending_receive_byte = platform_SC_pending_send_byte = 0;
}

/* Low level char PUSH/POP functions */
static inline uint8_t platform_SC_char_pop(void)
{
    return LL_USART_ReceiveData8(SMARTCARD_USART);
}
static inline void platform_SC_char_push(uint8_t c)
{
    LL_USART_TransmitData8(SMARTCARD_USART, c);

    return;
}



/* Smartcard putc and getc handling errors:
 * The getc function is non blocking */
int platform_SC_getc(uint8_t* c, __attribute__((unused)) uint32_t timeout, __attribute__((unused)) uint8_t reset)
{

    if(c == NULL) {
        return -1;
    }

    if(platform_SC_pending_receive_byte != 1) {
        return -1;
    }

    /* Data is ready, go ahead */
    *c = platform_SC_byte;
    platform_SC_pending_receive_byte = 0;

    return 0;
}

/* The putc function is non-blocking and checks
 * for errors. In the case of errors, try to send the byte again.
 */
int platform_SC_putc(uint8_t c, __attribute__((unused)) uint32_t timeout, __attribute__((unused)) uint8_t reset)
{

    if(reset) {
        platform_SC_pending_send_byte = 0;
        return 0;
    }

    if((platform_SC_pending_send_byte == 0) || (platform_SC_pending_send_byte >= 3)) {
        platform_io_dir_write();
        platform_SC_pending_send_byte = 1;
        platform_SC_char_push(c);
        return -1;
    }

    if(platform_SC_pending_send_byte == 2) {
        platform_io_dir_read();
        /* The byte has been sent */
        platform_SC_pending_send_byte = 0;
        return 0;
    }

    return -1;
}


void platform_smartcard_reinit(void)
{
    /* Reset our smartcard USART configuration */
    set_usart_config_default(&smartcard_usart_config, default_smartcard_usart_config);

    log_printf("==> Reinit USART%d\r\n", platform_get_usart_number(SMARTCARD_USART));
    LL_USART_Disable(SMARTCARD_USART);
    LL_USART_Enable(SMARTCARD_USART);

    return;
}

/**** Time measurement ***/
__attribute__((unused)) static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

__attribute__((unused)) static unsigned int get_cortex_m4_cycles(void)
{
    return DWT->CYCCNT;
}

/* Setup a microsecond timer */
static void setup_microsec_timer(void)
{
    /* We use TIM5 for high resolution 32-bit microsecond time */
    LL_TIM_InitTypeDef timer_config;
    LL_TIM_StructInit(&timer_config);
    __HAL_RCC_TIM5_CLK_ENABLE();
    /* Set prescaler to get a 1 microsecond resolution */
    timer_config.Prescaler = 84 - 1;
    timer_config.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    timer_config.CounterMode = LL_TIM_COUNTERMODE_UP;    
    LL_TIM_Init(TIM5, &timer_config);
    LL_TIM_SetCounter(TIM5, 0);
    LL_TIM_EnableCounter(TIM5);

    return;
}

/* Get ticks/time in microseconds
 */
uint32_t platform_get_microseconds_ticks(void)
{
    return LL_TIM_GetCounter(TIM5);
}

void platform_smartcard_lost(void)
{
    return;
}

static void (*volatile user_handler_action)(void) = NULL;
void platform_smartcard_register_user_handler_action(void (*action)(void))
{
    if(action == NULL) {
        return;
    }

    user_handler_action = action;

    return;
}

void platform_SC_reinit_iso7816(void)
{
    platform_SC_pending_receive_byte = 0;
    platform_SC_pending_send_byte = 0;
    platform_SC_byte = 0;
    dummy_usart_read = 0;

#if (SMARTCARD_CONTACT_ACTIVE == 1)
#if defined(SMARTCARD_CONTACT_INSERT_HIGH)
    platform_SC_is_smartcard_inserted = LL_GPIO_IsInputPinSet(SMARTCARD_CONTACT_PORT, SMARTCARD_CONTACT_PIN)  & 1;
#else
    platform_SC_is_smartcard_inserted = (~LL_GPIO_IsInputPinSet(SMARTCARD_CONTACT_PORT, SMARTCARD_CONTACT_PIN))  & 1;
#endif
#else
    platform_SC_is_smartcard_inserted = 1;
#endif

    platform_SC_gpio_smartcard_contact_changed = 0;

    /* Handle our LED */
    /* Activate LED or not depending on smartcard presence */
    if(platform_SC_is_smartcard_inserted) {
        led_status_on();
    } else {
        led_status_off();
    }

    return;
}

int platform_smartcard_set_1ETU_guardtime(void)
{
    /* We are already at 1 ETU guard time, so return OK */
    return 0;
}

#endif
