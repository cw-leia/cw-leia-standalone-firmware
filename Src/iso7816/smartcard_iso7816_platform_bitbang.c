#include "smartcard_iso7816_platform_bitbang.h"
#include "smartcard_print.h"
#include "smartcard.h"

#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_tim.h"

#include "config.h"
#include "leds.h"
#include "triggers.h"


/**** Bit Banging GPIOs ****/

static volatile uint8_t One_ETU_guardtime = 0;

static LL_GPIO_InitTypeDef gpio_smartcard_io = {
    .Pin = BITBANG_SMARTCARD_IO_PIN,
    .Mode = LL_GPIO_MODE_INPUT,
    .Pull = LL_GPIO_PULL_UP,
    .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_OPENDRAIN,
};


/* Assure it will not interfere with actual clock as it is connected on PCB */
static LL_GPIO_InitTypeDef gpio_smartcard_clk_usart = {
    .Pin = BITBANG_SMARTCARD_CLK_USART_PIN,
    .Mode = LL_GPIO_MODE_INPUT,
    .Pull = LL_GPIO_PULL_NO,
    .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_OPENDRAIN,
};

#ifdef BITBANG_SMARTCARD_IO_DIR_PIN
static LL_GPIO_InitTypeDef gpio_smartcard_io_dir = {
    .Pin = BITBANG_SMARTCARD_IO_DIR_PIN,
    .Mode = LL_GPIO_MODE_OUTPUT,
    .Pull = LL_GPIO_PULL_NO,
    .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
};
#endif

static LL_GPIO_InitTypeDef gpio_smartcard_clk = {
    .Pin        = BITBANG_SMARTCARD_CLK_PIN,
    .Mode       = LL_GPIO_MODE_ALTERNATE,
    .Pull       = LL_GPIO_PULL_DOWN,
    .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
    .Alternate  = BITBANG_SMARTCARD_CLK_AF
};

static LL_GPIO_InitTypeDef gpio_smartcard_rst = {
    .Pin        = BITBANG_SMARTCARD_RST_PIN,
    .Mode       = LL_GPIO_MODE_OUTPUT,
    .Pull       = LL_GPIO_PULL_DOWN,
    .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
};

static LL_GPIO_InitTypeDef gpio_smartcard_vcc = {
    .Pin        = BITBANG_SMARTCARD_VCC_PIN,
    .Mode       = LL_GPIO_MODE_OUTPUT,
    .Pull       = LL_GPIO_PULL_DOWN,
    .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
};

static LL_GPIO_InitTypeDef gpio_smartcard_vpp = {
    .Pin        = BITBANG_SMARTCARD_VPP_PIN,
    .Mode       = LL_GPIO_MODE_OUTPUT,
    .Pull       = LL_GPIO_PULL_DOWN,
    .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
};

/*** Smartcard auxiliary RFU pins (unused for now) ****/
__attribute__((unused)) static LL_GPIO_InitTypeDef gpio_smartcard_aux1 = {
    .Pin        = BITBANG_SMARTCARD_AUX1_PIN,
    .Mode       = LL_GPIO_MODE_OUTPUT,
    .Pull       = LL_GPIO_PULL_DOWN,
    .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
};

__attribute__((unused)) static LL_GPIO_InitTypeDef gpio_smartcard_aux2 = {
    .Pin        = BITBANG_SMARTCARD_AUX2_PIN,
    .Mode       = LL_GPIO_MODE_OUTPUT,
    .Pull       = LL_GPIO_PULL_DOWN,
    .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
};


/* Contact Pin --------------------------------------------------------------*/

#if (SMARTCARD_CONTACT_ACTIVE == 1)
LL_GPIO_InitTypeDef gpio_smartcard_contact = {
    .Pin = SMARTCARD_CONTACT_PIN,
    .Mode = LL_GPIO_MODE_INPUT,
    .Pull = LL_GPIO_PULL_NO,
    .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_OPENDRAIN,
};
#endif

/* Initialize the CONTACT pin */
static volatile uint8_t bitbang_platform_SC_gpio_smartcard_contact_changed = 0;
static volatile uint8_t bitbang_platform_SC_is_smartcard_inserted = 0;

uint8_t bitbang_platform_SC_is_lost(void) {
    return ((~bitbang_platform_SC_is_smartcard_inserted) & 0x1);
}

#if (SMARTCARD_CONTACT_ACTIVE == 1)
SC_Card* get_current_card();
void bitbang_platform_smartcard_contact_handler(void)
{
    if (LL_EXTI_IsActiveFlag_0_31(SMARTCARD_CONTACT_EXTI_LINE) != RESET) {
        LL_EXTI_ClearFlag_0_31(SMARTCARD_CONTACT_EXTI_LINE);

#if (SMARTCARD_CONTACT_ACTIVE == 1)
#if defined(SMARTCARD_CONTACT_INSERT_HIGH)
        bitbang_platform_SC_is_smartcard_inserted = LL_GPIO_IsInputPinSet(SMARTCARD_CONTACT_PORT, SMARTCARD_CONTACT_PIN)  & 1;
#else
        bitbang_platform_SC_is_smartcard_inserted = (~LL_GPIO_IsInputPinSet(SMARTCARD_CONTACT_PORT, SMARTCARD_CONTACT_PIN))  & 1;
#endif
#else
        bitbang_platform_SC_is_smartcard_inserted = 1;
#endif
        /* Handle our LED */
        /* Activate LED or not depending on smartcard presence */
        if(bitbang_platform_SC_is_smartcard_inserted) {
            led_status_on();
        } else {
            led_status_off();
        }
    }

    bitbang_platform_SC_gpio_smartcard_contact_changed = 1;

    return;
}
#endif

void bitbang_platform_init_smartcard_contact(void)
{
    bitbang_platform_SC_gpio_smartcard_contact_changed = 0;

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

    bitbang_platform_SC_gpio_smartcard_contact_changed = 1;

#if defined(SMARTCARD_CONTACT_INSERT_HIGH)
    bitbang_platform_SC_is_smartcard_inserted = LL_GPIO_IsInputPinSet(SMARTCARD_CONTACT_PORT, SMARTCARD_CONTACT_PIN)  & 1;
#else
    bitbang_platform_SC_is_smartcard_inserted = (~LL_GPIO_IsInputPinSet(SMARTCARD_CONTACT_PORT, SMARTCARD_CONTACT_PIN))  & 1;
#endif
#else
    bitbang_platform_SC_is_smartcard_inserted = 1;
#endif
    /* Handle our LED */
    /* Activate LED or not depending on smartcard presence */
    if(bitbang_platform_SC_is_smartcard_inserted) {
        led_status_on();
    } else {
        led_status_off();
    }

    return;
}

void bitbang_platform_SC_reinit_smartcard_contact(void)
{
    //bitbang_platform_SC_is_smartcard_inserted = 0;
    bitbang_platform_SC_gpio_smartcard_contact_changed = 1;

    return;
}

/* The BITBANG_SMARTCARD_CONTACT pin is at state high (pullup to Vcc) when no card is
 * not present, and at state low (linked to GND) when the card is inserted.
 */
uint8_t bitbang_platform_is_smartcard_inserted(void)
{
#if (SMARTCARD_CONTACT_ACTIVE == 1)
#if defined(SMARTCARD_CONTACT_INSERT_HIGH)
    bitbang_platform_SC_is_smartcard_inserted = LL_GPIO_IsInputPinSet(SMARTCARD_CONTACT_PORT, SMARTCARD_CONTACT_PIN)  & 1;
#else
    bitbang_platform_SC_is_smartcard_inserted = (~LL_GPIO_IsInputPinSet(SMARTCARD_CONTACT_PORT, SMARTCARD_CONTACT_PIN))  & 1;
#endif

    return bitbang_platform_SC_is_smartcard_inserted;
#else
    return 1;
#endif
}

/* IO Dir -------------------------------------------------------------------*/

static inline void bitbang_platform_io_dir_read(void)
{
#ifdef BITBANG_SMARTCARD_IO_DIR_PORT
    LL_GPIO_ResetOutputPin(BITBANG_SMARTCARD_IO_DIR_PORT, BITBANG_SMARTCARD_IO_DIR_PIN);
#endif
}

static inline void bitbang_platform_io_dir_write(void)
{
#ifdef BITBANG_SMARTCARD_IO_DIR_PORT
    LL_GPIO_SetOutputPin(BITBANG_SMARTCARD_IO_DIR_PORT, BITBANG_SMARTCARD_IO_DIR_PIN);
#endif
}

void bitbang_platform_init_io_dir(void)
{
#ifdef BITBANG_SMARTCARD_IO_DIR_PORT
    LL_GPIO_Init(BITBANG_SMARTCARD_IO_DIR_PORT, &gpio_smartcard_io_dir);

#endif
}

/* RST ----------------------------------------------------------------------*/

/* Initialize the RST pin */
void bitbang_platform_init_smartcard_rst(void)
{
    LL_GPIO_Init(BITBANG_SMARTCARD_RST_PORT, &gpio_smartcard_rst);

    /* Maintain the RST at low */
    LL_GPIO_ResetOutputPin(BITBANG_SMARTCARD_RST_PORT, BITBANG_SMARTCARD_RST_PIN);

    return;
}

void bitbang_platform_set_smartcard_rst(uint8_t val)
{
    if (val == 0) {
        LL_GPIO_ResetOutputPin(BITBANG_SMARTCARD_RST_PORT, BITBANG_SMARTCARD_RST_PIN);

    } else {
        LL_GPIO_SetOutputPin(BITBANG_SMARTCARD_RST_PORT, BITBANG_SMARTCARD_RST_PIN);
    }

    return;
}

/* VCC ----------------------------------------------------------------------*/

/* Initialize the Vcc pin */
void bitbang_platform_init_smartcard_vcc(void)
{
    LL_GPIO_Init(BITBANG_SMARTCARD_VCC_PORT, &gpio_smartcard_vcc);

    /* For now, maintain the Vcc at zero (low) */
#ifdef LEIA
    LL_GPIO_SetOutputPin(BITBANG_SMARTCARD_VCC_PORT, BITBANG_SMARTCARD_VCC_PIN);
#else
    LL_GPIO_ResetOutputPin(BITBANG_SMARTCARD_VCC_PORT, BITBANG_SMARTCARD_VCC_PIN);
#endif

    return;
}

void bitbang_platform_set_smartcard_vcc(uint8_t val)
{
#ifdef LEIA
    // Invert
    val = (~val) & 0x1;
#endif

    if (val == 0) {
        LL_GPIO_ResetOutputPin(BITBANG_SMARTCARD_VCC_PORT, BITBANG_SMARTCARD_VCC_PIN);

    } else {
        LL_GPIO_SetOutputPin(BITBANG_SMARTCARD_VCC_PORT, BITBANG_SMARTCARD_VCC_PIN);
    }

    return;
}

/* VPP ----------------------------------------------------------------------*/

/* Initialize the Vpp pin */
void bitbang_platform_init_smartcard_vpp(void)
{
    LL_GPIO_Init(BITBANG_SMARTCARD_VPP_PORT, &gpio_smartcard_vpp);

    /* Vpp is set low. Not used for now */
#ifdef LEIA
    LL_GPIO_SetOutputPin(BITBANG_SMARTCARD_VPP_PORT, BITBANG_SMARTCARD_VPP_PIN);
#else
    LL_GPIO_ResetOutputPin(BITBANG_SMARTCARD_VPP_PORT, BITBANG_SMARTCARD_VPP_PIN);
#endif
    return;
}

void bitbang_platform_set_smartcard_vpp(uint8_t val)
{
#ifdef LEIA
    // Invert
    val = (~val) & 0x1;
#endif

    if (val == 0) {
        LL_GPIO_ResetOutputPin(BITBANG_SMARTCARD_VPP_PORT, BITBANG_SMARTCARD_VPP_PIN);

    } else {
        LL_GPIO_SetOutputPin(BITBANG_SMARTCARD_VPP_PORT, BITBANG_SMARTCARD_VPP_PIN);
    }

    return;
}


static void bitbang_platform_init_smartcard_ioclk(void)
{
    LL_GPIO_Init(BITBANG_SMARTCARD_IO_PORT, &gpio_smartcard_io);
    LL_GPIO_Init(BITBANG_SMARTCARD_CLK_USART_PORT, &gpio_smartcard_clk_usart);

    return;
}

static void bitbang_platform_init_smartcard_clock(void) {
    LL_GPIO_Init(BITBANG_SMARTCARD_CLK_PORT, &gpio_smartcard_clk);

    return;
}

/* Initialize the I/O pin */
static void bitbang_platform_set_smartcard_IO_mode(uint32_t mode)
{
    LL_GPIO_SetPinMode(BITBANG_SMARTCARD_IO_PORT, BITBANG_SMARTCARD_IO_PIN, mode);
    return;
}


void bitbang_platform_set_smartcard_IO(uint8_t val)
{
    if (val == 0) {
        LL_GPIO_ResetOutputPin(BITBANG_SMARTCARD_IO_PORT, BITBANG_SMARTCARD_IO_PIN);

    } else {
        LL_GPIO_SetOutputPin(BITBANG_SMARTCARD_IO_PORT, BITBANG_SMARTCARD_IO_PIN);
    }

    return;
}

uint8_t bitbang_platform_get_smartcard_IO(void)
{
    return LL_GPIO_IsInputPinSet(BITBANG_SMARTCARD_IO_PORT, BITBANG_SMARTCARD_IO_PIN);
}


/* Fixed delay of a given number of microseconds */
static void SC_delay_microseconds(uint32_t microseconds_timeout)
{
    uint32_t start_tick, curr_tick;

    if(microseconds_timeout == 0) {
        return;
    }

    start_tick = bitbang_platform_get_microseconds_ticks();
    /* Now wait */
    curr_tick = start_tick;

    while((curr_tick - start_tick) <= microseconds_timeout) {
        curr_tick = bitbang_platform_get_microseconds_ticks();
    }

    return;
}


#define APB_BASE_CLOCK 42000000

int bitbang_platform_smartcard_clock_rounding(uint32_t target_freq, uint32_t* target_freq_rounded)
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
//TODO
int bitbang_platform_smartcard_set_freq(uint32_t* frequency)
{
    bitbang_platform_smartcard_clock_rounding(*frequency, frequency);

    LL_TIM_InitTypeDef timer_config;
    LL_TIM_StructInit(&timer_config);

    /* Initialize our GPIO and timer if they have not been already initialized */
    if(clock_initialized == 0) {
        bitbang_platform_init_smartcard_clock();
        clock_initialized = 1;
        /* Enable timer */

        __HAL_RCC_TIM2_CLK_ENABLE();
        LL_TIM_Init                 (BITBANG_TIMER_TO_USE, &timer_config);
        LL_TIM_EnableARRPreload     (BITBANG_TIMER_TO_USE);
        LL_TIM_SetCounterMode       (BITBANG_TIMER_TO_USE, LL_TIM_COUNTERMODE_DOWN);
        LL_TIM_SetOnePulseMode      (BITBANG_TIMER_TO_USE, LL_TIM_ONEPULSEMODE_REPETITIVE); // To check
        LL_TIM_SetUpdateSource      (BITBANG_TIMER_TO_USE, LL_TIM_UPDATESOURCE_COUNTER);
        LL_TIM_EnableUpdateEvent   (BITBANG_TIMER_TO_USE);
        LL_TIM_SetCounter(BITBANG_TIMER_TO_USE, 0);
    }

    /* Compute the necessary autoreload and prescaler for our frequency */
    LL_TIM_SetPrescaler(BITBANG_TIMER_TO_USE, 0);
    LL_TIM_SetAutoReload(BITBANG_TIMER_TO_USE, (APB_BASE_CLOCK / (*frequency)) - 1);
    log_printf("==> Asked for %d, rounded to %d\r\n", *frequency, APB_BASE_CLOCK / (APB_BASE_CLOCK / (*frequency)));
    *frequency = APB_BASE_CLOCK / (APB_BASE_CLOCK / (*frequency));

    /* We activate the output compare mode with the toggling option
     * in order to generate our clock
     */
    LL_TIM_OC_SetCompareCH4(BITBANG_TIMER_TO_USE, 0); // TODO make generic channel usage
    LL_TIM_OC_SetMode(BITBANG_TIMER_TO_USE, BITBANG_TIMER_CHANNEL, LL_TIM_OCMODE_TOGGLE);
    LL_TIM_CC_EnableChannel(BITBANG_TIMER_TO_USE, BITBANG_TIMER_CHANNEL);
    LL_TIM_EnableCounter(BITBANG_TIMER_TO_USE);

    return 0;
}

/*
 * Adapt our clocks and ETU
 */
static volatile uint32_t bitbang_platform_SC_current_sc_etu;
static volatile uint32_t bitbang_platform_SC_sc_clock_freq;
static int bitbang_platform_smartcard_clocks_init(uint32_t* target_freq, uint8_t target_guard_time, uint32_t* etu)
{
    if((etu == NULL) || (target_freq == NULL)) {
        goto err;
    }

    if(bitbang_platform_smartcard_set_freq(target_freq)) {
        goto err;
    }

    bitbang_platform_SC_current_sc_etu = *etu;
    bitbang_platform_SC_sc_clock_freq = *target_freq;

    return 0;

err:

    return -1;
}

static void DWT_Init(void);
static void setup_microsec_timer(void);
static volatile uint8_t setup_microsec_timer_done = 0;

static int bitbang_platform_smartcard_hw_init(void)
{
    log_printf("[Low level SC] BITBANG mode init\r\n");
    /* Initialize the GPIOs */
    bitbang_platform_init_smartcard_contact();
    bitbang_platform_init_smartcard_vcc();
    bitbang_platform_init_smartcard_vpp();
    bitbang_platform_init_smartcard_rst();
    bitbang_platform_init_smartcard_ioclk();
    bitbang_platform_init_smartcard_clock();
    bitbang_platform_init_io_dir();
    /* Initialize time measurement */
    if(setup_microsec_timer_done == 0){
        DWT_Init();
        setup_microsec_timer();
        setup_microsec_timer_done = 1;
    }

    return 0;
}

void bitbang_platform_smartcard_deinit(void)
{
    log_printf("[Low level SC] BITBANG mode deinit\r\n");
    clock_initialized = 0;
    setup_microsec_timer_done = 0;
    /* Mainly stop our timers */
    LL_TIM_DeInit(BITBANG_TIMER_TO_USE);
    __HAL_RCC_TIM2_CLK_DISABLE();
    LL_TIM_DeInit(TIM5);
    __HAL_RCC_TIM5_CLK_DISABLE();
}

int bitbang_platform_smartcard_init(void)
{
    /* Reinitialize global variables */

    return bitbang_platform_smartcard_hw_init();
}

/* Adapt clocks and guard time depending on what has been received */
int bitbang_platform_SC_adapt_clocks(uint32_t* etu, uint32_t* frequency)
{
    if((etu == NULL) || (frequency == NULL)) {
        goto err;
    }

    /* Adapt the clocks configuration in our structure */
    if(bitbang_platform_smartcard_clocks_init(frequency, 1, etu)) {
        goto err;
    }

    return 0;

err:
    return -1;
}

enum {
    PLATFORM_SC_TS_DIRECT_CONVENTION = 0,
    PLATFORM_SC_TS_INVERSE_CONVENTION = 1,
    PLATFORM_SC_TS_UNKOWN_CONVENTION = 2,
};

static volatile uint8_t bitbang_platform_SC_current_conv = PLATFORM_SC_TS_UNKOWN_CONVENTION;
/* Set the unkown convention at low level */
int bitbang_platform_SC_set_unkown_conv(void)
{
    bitbang_platform_SC_current_conv = PLATFORM_SC_TS_UNKOWN_CONVENTION;
    return 0;
}

/* Set the direct convention at low level */
int bitbang_platform_SC_set_direct_conv(void)
{
    bitbang_platform_SC_current_conv = PLATFORM_SC_TS_DIRECT_CONVENTION;
    return 0;
}

/* Set the inverse convention at low level */
int bitbang_platform_SC_set_inverse_conv(void)
{
    bitbang_platform_SC_current_conv = PLATFORM_SC_TS_INVERSE_CONVENTION;
    return 0;
}

/* Get the inverse convention at low level */
static int bitbang_platform_SC_is_inverse_conv(void)
{
    if(bitbang_platform_SC_current_conv == PLATFORM_SC_TS_INVERSE_CONVENTION) {
        return 1;
    }

    return 0;
}

/* Low level flush of our receive/send state, in order
 * for the higher level to be sure that everything is clean
 */
void bitbang_platform_SC_flush(void)
{
    /* Since we are stateless, flushing is nothing */
    return;
}

static uint32_t bitbang_platform_SC_get_sc_etu(void)
{
    return bitbang_platform_SC_current_sc_etu;
}

static uint32_t bitbang_platform_SC_get_sc_clock_freq(void)
{
    return bitbang_platform_SC_sc_clock_freq;
}

/* Fixed delay of a given number of clock cycles */
static void bitbang_platform_SC_delay_sc_clock_cycles(uint32_t sc_clock_cycles_timeout)
{
    uint64_t t;
    uint32_t start_tick, curr_tick;

    if(sc_clock_cycles_timeout == 0) {
        return;
    }

    /* The timeout is in smartcard clock cycles, which can be converted to MCU clock time
     * using a simple conversion. The clock time is expressed in microseconds.
     */
    t = (sc_clock_cycles_timeout * 1000000ULL) / bitbang_platform_SC_get_sc_clock_freq();
    start_tick = bitbang_platform_get_microseconds_ticks();
    /* Now wait */
    curr_tick = start_tick;

    while((curr_tick - start_tick) <= (uint32_t)t) {
        curr_tick = bitbang_platform_get_microseconds_ticks();
    }

    return;
}


/* The getc function. */
int bitbang_platform_SC_getc(uint8_t* c, uint32_t timeout, uint8_t reset)
{
    bitbang_platform_io_dir_read();

    if(c == NULL) {
        return -1;
    }

    if(reset) {
        return 0;
    }

    /* The timeout is in ETU clocks */

    /* Wait until the IO level is low (start bit begins) */
    if((bitbang_platform_get_smartcard_IO() & 0x1) == 1) {
        goto err;
    }

    /* Loop over the received bytes */
    uint8_t byte = 0;
    uint8_t parity_bit = 0, computed_parity = 0;
    uint8_t i;

    for(i = 0; i < 9; i++) {
        /* Wait one ETU */
        bitbang_platform_SC_delay_sc_clock_cycles(bitbang_platform_SC_get_sc_etu());

        if(i < 8) {
            /* Fill the received byte bits */
            uint8_t bit = bitbang_platform_get_smartcard_IO() & 0x1;
            byte |= (bit << i);
            computed_parity ^= bit;

        } else {
            /* Handle the parity bit */
            parity_bit = bitbang_platform_get_smartcard_IO() & 0x1;
        }
    }

    if(bitbang_platform_SC_is_inverse_conv()) {
        parity_bit = (~parity_bit) & 0x1;
    }

    /* If we still do not know our convention (i.e. this is the 
     * first byte of the ATR), the parity bit might be wrong. We
     * hence ignore it only in this case!
     */
    if(bitbang_platform_SC_current_conv != PLATFORM_SC_TS_UNKOWN_CONVENTION){
        /* Check the parity bit */
        if(parity_bit != computed_parity) {
            /* TODO: set the I/O line down to report the error to the sender */
            goto err;
        }
    }

    bitbang_platform_SC_delay_sc_clock_cycles(bitbang_platform_SC_get_sc_etu());

    /* Return the byte */
    *c = byte;
    /* Be ready to listen another byte as soon as possible
     * after the stop bit */
    bitbang_platform_SC_delay_sc_clock_cycles(bitbang_platform_SC_get_sc_etu());

    if(One_ETU_guardtime == 0) {
        /* Compute the number of microseconds for ETU/2
         * NB: we do this because of the precision
         */
        uint64_t etu_microseconds = (bitbang_platform_SC_get_sc_etu() * 1000000ULL) / bitbang_platform_SC_get_sc_clock_freq();
        SC_delay_microseconds(etu_microseconds / 2);
    }

    return 0;
err:
    return -1;
}

/* The putc function. */
int bitbang_platform_SC_putc(uint8_t c, uint32_t timeout, uint8_t reset)
{
    bitbang_platform_io_dir_write();

    if(reset) {
        return 0;
    }

    /* The timeout is in ETU clocks */
    /* Set the I/O to output mode */
    bitbang_platform_set_smartcard_IO_mode(LL_GPIO_MODE_OUTPUT);

    /* Send the start bit */
    bitbang_platform_set_smartcard_IO(0);
    bitbang_platform_SC_delay_sc_clock_cycles(bitbang_platform_SC_get_sc_etu());

    /* Send the byte */
    uint8_t byte = c;
    uint8_t parity = 0;
    uint8_t i;

    for(i = 0; i < 8; i++) {
        uint8_t bit;
        bit = (byte >> i) & 0x01;
        parity ^= bit;
        bitbang_platform_set_smartcard_IO(bit);
        /* Wait one ETU */
        bitbang_platform_SC_delay_sc_clock_cycles(bitbang_platform_SC_get_sc_etu());
    }

    /* The the parity bit */
    if(bitbang_platform_SC_is_inverse_conv()) {
        parity = (~parity) & 0x1;
    }

    bitbang_platform_set_smartcard_IO(parity);
    /* Wait one ETU */
    bitbang_platform_SC_delay_sc_clock_cycles(bitbang_platform_SC_get_sc_etu());

    /* Send the stop byte */
    bitbang_platform_set_smartcard_IO(1);
    /* Wait one ETU and a half to emit the stop byte, except if the card asked for 1 ETU guard time */
    bitbang_platform_SC_delay_sc_clock_cycles(bitbang_platform_SC_get_sc_etu());

    if(One_ETU_guardtime == 0) {
        /* Compute the number of microseconds for ETU/2
         * NB: we do this because of the precision
         */
        uint64_t etu_microseconds = (bitbang_platform_SC_get_sc_etu() * 1000000ULL) / bitbang_platform_SC_get_sc_clock_freq();
        SC_delay_microseconds(etu_microseconds / 2);
    }

    /* TODO: check for an error from the receiver! */

    /* Set the I/O back to input mode (stop bit) */
    bitbang_platform_set_smartcard_IO(1);
    bitbang_platform_set_smartcard_IO_mode(LL_GPIO_MODE_INPUT);

    return 0;
}


void bitbang_platform_smartcard_reinit(void)
{
    log_printf("==> Reinit ISO7816 Driver\r\n");

    One_ETU_guardtime = 0;
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
uint32_t bitbang_platform_get_microseconds_ticks(void)
{
    return LL_TIM_GetCounter(TIM5);
}

void bitbang_platform_smartcard_lost(void)
{
    return;
}

static void (*volatile user_handler_action)(void) = NULL;
void bitbang_platform_smartcard_register_user_handler_action(void (*action)(void))
{
    if(action == NULL) {
        return;
    }

    user_handler_action = action;

    return;
}

void bitbang_platform_SC_reinit_iso7816(void)
{
#if (SMARTCARD_CONTACT_ACTIVE == 1)
#if defined(SMARTCARD_CONTACT_INSERT_HIGH)
    bitbang_platform_SC_is_smartcard_inserted = LL_GPIO_IsInputPinSet(SMARTCARD_CONTACT_PORT, SMARTCARD_CONTACT_PIN)  & 1;
#else
    bitbang_platform_SC_is_smartcard_inserted = (~LL_GPIO_IsInputPinSet(SMARTCARD_CONTACT_PORT, SMARTCARD_CONTACT_PIN))  & 1;
#endif
#else
    bitbang_platform_SC_is_smartcard_inserted = 1;
#endif

    bitbang_platform_SC_gpio_smartcard_contact_changed = 0;

    /* Handle our LED */
    /* Activate LED or not depending on smartcard presence */
    if(bitbang_platform_SC_is_smartcard_inserted) {
        led_status_on();
    } else {
        led_status_off();
    }

    return;
}

int bitbang_platform_smartcard_set_1ETU_guardtime(void)
{
    One_ETU_guardtime = 1;
    return 0;
}
