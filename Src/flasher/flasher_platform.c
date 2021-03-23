#include "flasher_platform.h"
#include "flasher_print.h"

#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_tim.h"

#include "config.h"
#include "leds.h"


/**** Flasher GPIOs ****/

/* SPI clock */
static LL_GPIO_InitTypeDef gpio_flasher_clk = {
    .Pin = FLASHER_CLK_PIN,
    .Mode = LL_GPIO_MODE_OUTPUT,
    .Pull = LL_GPIO_PULL_NO,
    .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
};

/* SPI MISO */
static LL_GPIO_InitTypeDef gpio_flasher_miso = {
    .Pin = FLASHER_MISO_PIN,
    .Mode = LL_GPIO_MODE_INPUT,
    .Pull = LL_GPIO_PULL_NO,
    .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_OPENDRAIN,
};

/* SPI MOSI */
static LL_GPIO_InitTypeDef gpio_flasher_mosi = {
    .Pin = FLASHER_MOSI_PIN,
    .Mode = LL_GPIO_MODE_OUTPUT,
    .Pull = LL_GPIO_PULL_NO,
    .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
};

/* Target reset */
static LL_GPIO_InitTypeDef gpio_flasher_rst = {
    .Pin    = FLASHER_RST_PIN,
    .Mode       = LL_GPIO_MODE_OUTPUT,
    .Pull       = LL_GPIO_PULL_DOWN,
    .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
};

/* Target Xtal (uses a timer) */
static LL_GPIO_InitTypeDef gpio_flasher_xtal = {
    .Pin    = FLASHER_XTAL_PIN,
    .Mode       = LL_GPIO_MODE_ALTERNATE,
    .Pull       = LL_GPIO_PULL_DOWN,
    .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
    .Alternate  = FLASHER_XTAL_AF,
};

/* I/O DIR direction pin */
#ifdef FLASHER_IO_DIR_PIN
static LL_GPIO_InitTypeDef gpio_flasher_io_dir = {
    .Pin = FLASHER_IO_DIR_PIN,
    .Mode = LL_GPIO_MODE_OUTPUT,
    .Pull = LL_GPIO_PULL_NO,
    .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
};
#endif


/* Contact Pin --------------------------------------------------------------*/

#if (SMARTCARD_CONTACT_ACTIVE == 1)
LL_GPIO_InitTypeDef gpio_flasher_contact = {
    .Pin = SMARTCARD_CONTACT_PIN,
    .Mode = LL_GPIO_MODE_INPUT,
    .Pull = LL_GPIO_PULL_NO,
    .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_OPENDRAIN,
};
#endif

/* Initialize the CONTACT pin */
static volatile uint8_t platform_flasher_gpio_smartcard_contact_changed = 0;
static volatile uint8_t platform_flasher_is_smartcard_inserted = 0;

uint8_t platform_flasher_is_lost(void) {
    return ((~platform_flasher_is_smartcard_inserted) & 0x1);
}

#if (SMARTCARD_CONTACT_ACTIVE == 1)
void platform_flasher_contact_handler(void)
{
    if (LL_EXTI_IsActiveFlag_0_31(SMARTCARD_CONTACT_EXTI_LINE) != RESET) {
    LL_EXTI_ClearFlag_0_31(SMARTCARD_CONTACT_EXTI_LINE);

#if (SMARTCARD_CONTACT_ACTIVE == 1)
#if defined(SMARTCARD_CONTACT_INSERT_HIGH)
    platform_flasher_is_smartcard_inserted = LL_GPIO_IsInputPinSet(SMARTCARD_CONTACT_PORT, SMARTCARD_CONTACT_PIN)  & 1;
#else
    platform_flasher_is_smartcard_inserted = (~LL_GPIO_IsInputPinSet(SMARTCARD_CONTACT_PORT, SMARTCARD_CONTACT_PIN))  & 1;
#endif
#else
    platform_flasher_is_smartcard_inserted = 1;
#endif
    /* Handle our LED */
    /* Activate LED or not depending on smartcard presence */
    if(platform_flasher_is_smartcard_inserted) {
        led_status_on();
    } else {
        led_status_off();
    }
    }

    platform_flasher_gpio_smartcard_contact_changed = 1;

    return;
}
#endif

void platform_flasher_init_contact(void)
{
    platform_flasher_gpio_smartcard_contact_changed = 0;

#if (SMARTCARD_CONTACT_ACTIVE == 1)
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_GPIO_Init(SMARTCARD_CONTACT_PORT, &gpio_flasher_contact);

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

    platform_flasher_gpio_smartcard_contact_changed = 1;

#if defined(SMARTCARD_CONTACT_INSERT_HIGH)
    platform_flasher_is_smartcard_inserted = LL_GPIO_IsInputPinSet(SMARTCARD_CONTACT_PORT, SMARTCARD_CONTACT_PIN)  & 1;
#else
    platform_flasher_is_smartcard_inserted = (~LL_GPIO_IsInputPinSet(SMARTCARD_CONTACT_PORT, SMARTCARD_CONTACT_PIN))  & 1;
#endif
#else
    platform_flasher_is_smartcard_inserted = 1;
#endif
    /* Handle our LED */
    /* Activate LED or not depending on flasher presence */
    if(platform_flasher_is_smartcard_inserted) {
    led_status_on();
    } else {
    led_status_off();
    }

    return;
}

void platform_flasher_reinit_contact(void)
{
    //platform_flasher_is_smartcard_inserted = 0;
    platform_flasher_gpio_smartcard_contact_changed = 1;

    return;
}

/* The SMARTCARD_CONTACT pin is at state high (pullup to Vcc) when no card is
 * not present, and at state low (linked to GND) when the card is inserted.
 */
uint8_t platform_flasher_smartcard_inserted(void)
{
#if (SMARTCARD_CONTACT_ACTIVE == 1)
#if defined(SMARTCARD_CONTACT_INSERT_HIGH)
    platform_flasher_is_smartcard_inserted = LL_GPIO_IsInputPinSet(SMARTCARD_CONTACT_PORT, SMARTCARD_CONTACT_PIN)  & 1;
#else
    platform_flasher_is_smartcard_inserted = (~LL_GPIO_IsInputPinSet(SMARTCARD_CONTACT_PORT, SMARTCARD_CONTACT_PIN))  & 1;
#endif

    return platform_flasher_is_smartcard_inserted;
#else
    return 1;
#endif
}

/* RST ----------------------------------------------------------------------*/

/* Initialize the RST pin */
void platform_flasher_init_rst(void)
{
    LL_GPIO_Init(FLASHER_RST_PORT, &gpio_flasher_rst);

    /* Maintain the RST at low */
    LL_GPIO_ResetOutputPin(FLASHER_RST_PORT, FLASHER_RST_PIN);

    return;
}

void platform_flasher_set_rst(uint8_t val)
{
    if (val == 0) {
    LL_GPIO_ResetOutputPin(FLASHER_RST_PORT, FLASHER_RST_PIN);

    } else {
    LL_GPIO_SetOutputPin(FLASHER_RST_PORT, FLASHER_RST_PIN);
    }

    return;
}

/* MISO  ----------------------------------------------------------------------*/

/* Initialize the MISO pin */
void platform_flasher_init_miso(void)
{
    LL_GPIO_Init(FLASHER_MISO_PORT, &gpio_flasher_miso);

    return;
}

uint8_t platform_flasher_get_miso(void)
{
    return LL_GPIO_IsInputPinSet(FLASHER_MISO_PORT, FLASHER_MISO_PIN);
}


/* MOSI ----------------------------------------------------------------------*/

/* Initialize the MOSI pin */
void platform_flasher_init_mosi(void)
{
    LL_GPIO_Init(FLASHER_MOSI_PORT, &gpio_flasher_mosi);

    return;
}

void platform_flasher_set_mosi(uint8_t val)
{
    if (val == 0) {
    LL_GPIO_ResetOutputPin(FLASHER_MOSI_PORT, FLASHER_MOSI_PIN);

    } else {
    LL_GPIO_SetOutputPin(FLASHER_MOSI_PORT, FLASHER_MOSI_PIN);
    }

    return;
}



/* CLK ----------------------------------------------------------------------*/
/* Initialize the CLK pin */
void platform_flasher_init_clk(void) {
    LL_GPIO_Init(FLASHER_CLK_PORT, &gpio_flasher_clk);

    return;
}

void platform_flasher_set_clk(uint8_t val)
{
    if (val == 0) {
    LL_GPIO_ResetOutputPin(FLASHER_CLK_PORT, FLASHER_CLK_PIN);

    } else {
    LL_GPIO_SetOutputPin(FLASHER_CLK_PORT, FLASHER_CLK_PIN);
    }

    return;
}

/* XTAL ----------------------------------------------------------------------*/
/* Initialize the XTAL pin */
static void platform_flasher_init_xtal(void) {
    LL_GPIO_Init(FLASHER_XTAL_PORT, &gpio_flasher_xtal);

    return;
}

/* IO Dir -------------------------------------------------------------------*/

static inline void platform_flasher_io_dir_read(void)
{
#ifdef FLASHER_IO_DIR_PORT
    LL_GPIO_ResetOutputPin(FLASHER_IO_DIR_PORT, FLASHER_IO_DIR_PIN);
#endif
}

static inline void platform_flasher_io_dir_write(void)
{
#ifdef FLASHER_IO_DIR_PORT
    LL_GPIO_SetOutputPin(FLASHER_IO_DIR_PORT, FLASHER_IO_DIR_PIN);
#endif
}

void platform_flasher_init_io_dir(void)
{
#ifdef FLASHER_IO_DIR_PORT
    LL_GPIO_Init(FLASHER_IO_DIR_PORT, &gpio_flasher_io_dir);
#endif
}


#define APB_BASE_CLOCK 42000000

int platform_flasher_xtal_rounding(uint32_t target_freq, uint32_t* target_freq_rounded)
{
    if(target_freq_rounded == NULL) {
    goto err;
    }

    /* Find a suitable frequency *under* our asked frequency (in order to avoid going beyond the card max supported one) */

    uint32_t try = target_freq;

    while((APB_BASE_CLOCK / (APB_BASE_CLOCK / try)) > target_freq) {
    try = try - 1000;
    }

    *target_freq_rounded = APB_BASE_CLOCK / (APB_BASE_CLOCK / (try));

    return 0;
err:

    return -1;

}


static volatile uint8_t clock_initialized = 0;
int platform_flasher_set_xtal_freq(uint32_t* frequency)
{
    platform_flasher_xtal_rounding(*frequency, frequency);

    LL_TIM_InitTypeDef timer_config;
    LL_TIM_StructInit(&timer_config);

    /* Initialize our GPIO and timer if they have not been already initialized */
    if(clock_initialized == 0) {
        platform_flasher_init_xtal();
        clock_initialized = 1;
        /* Enable timer */

        __HAL_RCC_TIM2_CLK_ENABLE();
        LL_TIM_Init         (FLASHER_TIMER_TO_USE, &timer_config);
        LL_TIM_EnableARRPreload     (FLASHER_TIMER_TO_USE);
        LL_TIM_SetCounterMode       (FLASHER_TIMER_TO_USE, LL_TIM_COUNTERMODE_DOWN);
        LL_TIM_SetOnePulseMode      (FLASHER_TIMER_TO_USE, LL_TIM_ONEPULSEMODE_REPETITIVE); // To check
        LL_TIM_SetUpdateSource      (FLASHER_TIMER_TO_USE, LL_TIM_UPDATESOURCE_COUNTER);
        LL_TIM_EnableUpdateEvent   (FLASHER_TIMER_TO_USE);
        LL_TIM_SetCounter(FLASHER_TIMER_TO_USE, 0);
    }

    /* Compute the necessary autoreload and prescaler for our frequency */
    LL_TIM_SetPrescaler(FLASHER_TIMER_TO_USE, 0);
    LL_TIM_SetAutoReload(FLASHER_TIMER_TO_USE, (APB_BASE_CLOCK / (*frequency)) - 1);
    log_printf("==> Asked for %d, rounded to %d\r\n", *frequency, APB_BASE_CLOCK / (APB_BASE_CLOCK / (*frequency)));
    *frequency = APB_BASE_CLOCK / (APB_BASE_CLOCK / (*frequency));

    /* We activate the output compare mode with the toggling option
     * in order to generate our clock
     */
    LL_TIM_OC_SetCompareCH4(FLASHER_TIMER_TO_USE, 0); // TODO make generic channel usage
    LL_TIM_OC_SetMode(FLASHER_TIMER_TO_USE, FLASHER_TIMER_CHANNEL, LL_TIM_OCMODE_TOGGLE);
    LL_TIM_CC_EnableChannel(FLASHER_TIMER_TO_USE, FLASHER_TIMER_CHANNEL);
    LL_TIM_EnableCounter(FLASHER_TIMER_TO_USE);

    return 0;
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
static uint32_t platform_flasher_get_microseconds_ticks(void)
{
    return LL_TIM_GetCounter(TIM5);
}



static volatile uint32_t platform_flasher_clock_freq;
static volatile uint32_t platform_flasher_clock_delay_ns;

int platform_flasher_set_freq(uint32_t *frequency)
{
    if(frequency == NULL){
        goto err;
    }
    platform_flasher_clock_freq = *frequency;
    /* Set delay depending on frequency */
    platform_flasher_clock_delay_ns = 500 * (1000000 / (*frequency));
err:
    return 0;
}


static void DWT_Init(void);
static void setup_microsec_timer(void);
static volatile uint8_t setup_microsec_timer_done = 0;

#define XTAL_FREQUENCY 2000000

static int platform_flasher_hw_init(void)
{
    log_printf("[Low level] FLASHER mode init\r\n");
    /* Initialize the GPIOs */
    platform_flasher_init_mosi();
    platform_flasher_set_mosi(0);
    platform_flasher_init_miso();
    platform_flasher_init_clk();
    platform_flasher_set_clk(0);
    platform_flasher_init_rst();
    platform_flasher_init_xtal();
    /* IO DIR is put in read mode (so that MISO can be read) */
    platform_flasher_init_io_dir();
    platform_flasher_io_dir_read();
    /* Initialize time measurement */
    if(setup_microsec_timer_done == 0){
        DWT_Init();
        setup_microsec_timer();
        setup_microsec_timer_done = 1;
    }

    /* Set XTAL frequency */
    uint32_t xtal_freq = XTAL_FREQUENCY;
    platform_flasher_set_xtal_freq(&xtal_freq);

    /* Init contact GPIO */
    platform_flasher_init_contact();

    return 0;
}

void platform_flasher_deinit(void)
{
    log_printf("[Low level] FLASHER mode deinit\r\n");
    clock_initialized = 0;
    setup_microsec_timer_done = 0;
    /* Mainly stop our timers */
    LL_TIM_DeInit(BITBANG_TIMER_TO_USE);
    __HAL_RCC_TIM2_CLK_DISABLE();
    LL_TIM_DeInit(TIM5);
    __HAL_RCC_TIM5_CLK_DISABLE();
}

int platform_flasher_init(void)
{
    return platform_flasher_hw_init();
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

/***************************************************/
/* Fixed delay of a given number of nanoseconds */
void platform_flasher_delay_nanoseconds(uint32_t nanoseconds_timeout){
    uint32_t start, end;
    start = (1000 / 168) * get_cortex_m4_cycles();
    end = start;
    while((end - start) <= nanoseconds_timeout){
        end = (1000 / 168) * get_cortex_m4_cycles();
    }
    return;
}

/* Fixed delay of a given number of microseconds */
void platform_flasher_delay_microseconds(uint32_t microseconds_timeout){
    uint32_t start_tick, curr_tick;

    if(microseconds_timeout == 0){
        return;
    }
    start_tick = platform_flasher_get_microseconds_ticks();
    /* Now wait */
    curr_tick = start_tick;
    while((curr_tick - start_tick) <= microseconds_timeout){
        curr_tick = platform_flasher_get_microseconds_ticks();
    }

    return;
}

void platform_flasher_delay_milliseconds(uint32_t milliseconds_timeout){
    platform_flasher_delay_microseconds(milliseconds_timeout * 1000ULL);
    return;
}


/* In order to read, push a byte on MOSI and get back a byte
 * on MISO
 */
int platform_flasher_xfer_byte(uint8_t in, uint8_t *out, uint8_t msb)
{
    if(out == NULL){
    return -1;
    }
    unsigned int read_bits = 0;
    uint32_t mask;
    if(msb == MSB){
    mask = (0x1 << 7);
    }
    else{
    mask = 0x1;
    }
    *out = 0;
    uint8_t c = 0;
    while(read_bits < 8){
    /* Set MOSI */
    if(in & mask){
        platform_flasher_set_mosi(1);
    }
    else{
        platform_flasher_set_mosi(0);
    }
    if(msb == MSB){
        in = in << 1;
    }
    else{
        in = in >> 1;
    }
    /* Set Clock high */
    platform_flasher_set_clk(1);
    /* Wait a bit */
    platform_flasher_delay_nanoseconds(platform_flasher_clock_delay_ns);
    /* Sample MISO */
    if(platform_flasher_get_miso() & 0x1){
        if(msb == MSB){
        c = c | (0x1 << (7-read_bits));
        }
        else{
        c = c | (0x1 << read_bits);
        }
    }
    /* Set cloc low */
    platform_flasher_set_clk(0);
    read_bits++;
    /* Wait a bit */
    platform_flasher_delay_nanoseconds(platform_flasher_clock_delay_ns);
    }
    *out = c;

    /* Set MOSI to 0 */
    platform_flasher_set_mosi(0);
    /* Wait a bit */
    platform_flasher_delay_nanoseconds(4*platform_flasher_clock_delay_ns);

    return 0;
}
