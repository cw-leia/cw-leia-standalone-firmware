#include "smartcard_iso7816_platform.h"
#include "smartcard_print.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "triggers.h"

/* The target clock frequency is 3.5MHz for the ATR < max 5MHz.
 * The advantage of this frequency is that it is a perfect divisor of
 * our USARTS core drequencies:
 *   - For USART 1 and 6: 84 MHz / 3.5 MHz =  24
 *   - For USART 2, 3 and 4: 42 MHz / 3.5 MHz = 12
 * NB: you can change this default frequency to 4.2 MHz or 5.25 MHz which are
 * the next divisors when using our USART 2 core frequency. This should work with
 * most of the cards while being faster, but we choose to keep 3.5 MHz mainly as a
 * conservative choice.
 * NB2: this default frequency is a priori only used during the ATR, and a faster communication
 * could/should be established with the card after parsing the ATR and/or negotiating with the
 * smartcard.
 */

/* Smartcard USART configuration: we use USART 2: (TX = I/O = PA2, CLK = PA4)
 * We use the SMARTCARD mode when configuring the GPIOs.
 * The baudrate is set to 9408 bauds (see the explanations in the smartcard_init function). 
 * The parameters are set to meet the requirements in the datasheet 24.3.11 section 'Smartcard':
 *      - LINEN bit in the USART_CR2 register cleared. 
 *      - HDSEL and IREN bits in the USART_CR3 register cleared.
 *      - Moreover, the CLKEN bit may be set in order to provide a clock to the smartcard.
 * The Smartcard interface is designed to support asynchronous protocol Smartcards as
 * defined in the ISO 7816-3 standard. The USART should be configured as:
 *      - 8 bits plus parity: where M=1 and PCE=1 in the USART_CR1 register
 *      - 1.5 stop bits when transmitting and receiving: where STOP=11 in the USART_CR2 register.
 */
/* Smartcard uses USART 2, i.e. I/O is on PA2 and CLK is on PA4 */
#define SMARTCARD_USART         2

/* The USART we use for smartcard.
 * STM32F4 provides the I/O pin on the TX USART pin, and
 * the CLK pin on the dedicated USART CK pin.
 * The RST (reset) pin uses a dedicated GPIO.
 * The card detect pin uses a dedicated GPIO.
 * The VCC and VPP pins use dedicated GPIOs.
 */
cb_usart_getc_t platform_SC_usart_getc = NULL;
cb_usart_putc_t platform_SC_usart_putc = NULL;
static void platform_smartcard_irq(void);

const usart_config_t default_smartcard_usart_config = {
        .set_mask = USART_SET_ALL,
        .mode = SMARTCARD,
        .usart = SMARTCARD_USART,
        /* To be filled later depending on the clock configuration */
        .baudrate = 0,
        /* Word length = 9 bits (8 bits + parity) */
        .word_length = USART_CR1_M_9,
        /* 1 stop bit instead of 1.5 is necessary for some cards that use a very short delay between USART characters ... */
        .stop_bits = USART_CR2_STOP_1BIT,
        /* 8 bits plus parity  (parity even) */
        .parity = USART_CR1_PCE_EN | USART_CR1_PS_EVEN,
        /* Hardware flow control disabled, smartcard mode enabled, smartard NACK enabled, HDSEL and IREN disabled, error interrupts enabled (for framing error) */
        .hw_flow_control = USART_CR3_CTSE_CTS_DIS | USART_CR3_RTSE_RTS_DIS | USART_CR3_SCEN_EN | USART_CR3_NACK_EN | USART_CR3_HDSEL_DIS | USART_CR3_IREN_DIS | USART_CR3_EIE_EN,
        /* TX and RX are enabled, parity error interrupt enabled */
        .options_cr1 = USART_CR1_TE_EN | USART_CR1_RE_EN | USART_CR1_PEIE_EN | USART_CR1_RXNEIE_EN | USART_CR1_TCIE_EN,
        /* LINEN disabled, USART clock enabled, CPOL low, CPHA 1st edge, last bit clock pulse enabled */
        .options_cr2 = USART_CR2_LINEN_DIS | USART_CR2_CLKEN_EN | USART_CR2_CPOL_DIS | USART_CR2_CPHA_DIS | USART_CR2_LBCL_EN,
        /* To be filled later depending on the clock configuration */
        .guard_time_prescaler = 0,
        /* Reception callback */
        .callback_irq_handler = platform_smartcard_irq,
        /* Receive and send function pointers using polling (not needed per se since we are interrupt driven) */
        .callback_usart_getc_ptr = &platform_SC_usart_getc,
        .callback_usart_putc_ptr = &platform_SC_usart_putc,
};

/* Current smartcard USART config: it is not const since it can be changed
 * depending on the detected smartcard.
 */
usart_config_t smartcard_usart_config;


#define SMARTCARD_IO_DIR_PORT		GPIOE_BASE
#define SMARTCARD_IO_DIR_PIN            3
const gpio_config_t gpio_smartcard_io_dir = {
	.port = SMARTCARD_IO_DIR_PORT,
	.pin  = SMARTCARD_IO_DIR_PIN,
        .set_mask = GPIO_SET_MODE | GPIO_SET_PUPD | GPIO_SET_SPEED | GPIO_SET_TYPE,
        .mode = PIN_OUTPUT_MODE,
        .pupd = GPIO_PULLUP,
        .speed = PIN_VERY_HIGH_SPEED,
        .type = PIN_OTYPER_PP,
};

/* Is the contact pin active? */
#define SMARTCARD_CONTACT_ACTIVE	0

/* Smartcard contact detect is PC11 */
#define SMARTCARD_CONTACT_PORT          GPIOC_BASE
#define SMARTCARD_CONTACT_PIN           11
const gpio_config_t gpio_smartcard_contact = {
        .port = SMARTCARD_CONTACT_PORT,
        .pin = SMARTCARD_CONTACT_PIN,
        .set_mask = GPIO_SET_MODE | GPIO_SET_PUPD | GPIO_SET_SPEED | GPIO_SET_TYPE,
        .mode = PIN_INPUT_MODE,
        .pupd = GPIO_NOPULL,
        .speed = PIN_VERY_HIGH_SPEED,
        .type = PIN_OTYPER_OD,
};
/* Smartcard RST is PE3 */
#define SMARTCARD_RST_PORT              GPIOC_BASE
#define SMARTCARD_RST_PIN               12
const gpio_config_t gpio_smartcard_rst = {
        .port = SMARTCARD_RST_PORT,
        .pin = SMARTCARD_RST_PIN,
        .set_mask = GPIO_SET_MODE | GPIO_SET_PUPD | GPIO_SET_SPEED | GPIO_SET_TYPE,
        .mode = PIN_OUTPUT_MODE,
        .pupd = GPIO_PULLDOWN,
        .speed = PIN_VERY_HIGH_SPEED,
        .type = PIN_OTYPER_PP,
};

/* Smartcard Vcc is PD7 */
#define SMARTCARD_VCC_PORT              GPIOD_BASE
#define SMARTCARD_VCC_PIN               7
const gpio_config_t gpio_smartcard_vcc = {
        .port = SMARTCARD_VCC_PORT,
        .pin = SMARTCARD_VCC_PIN,
        .set_mask = GPIO_SET_MODE | GPIO_SET_PUPD | GPIO_SET_SPEED | GPIO_SET_TYPE,
        .mode = PIN_OUTPUT_MODE,
        .pupd = GPIO_PULLDOWN,
        .speed = PIN_VERY_HIGH_SPEED,
        .type = PIN_OTYPER_PP,
};
/* Smartcard Vpp is PE7
 */
#define SMARTCARD_VPP_PORT              GPIOE_BASE
#define SMARTCARD_VPP_PIN               7
const gpio_config_t gpio_smartcard_vpp = {
        .port = SMARTCARD_VPP_PORT,
        .pin = SMARTCARD_VPP_PIN,
        .set_mask = GPIO_SET_MODE | GPIO_SET_PUPD | GPIO_SET_SPEED | GPIO_SET_TYPE,
        .mode = PIN_OUTPUT_MODE,
        .pupd = GPIO_PULLDOWN,
        .speed = PIN_VERY_HIGH_SPEED,
        .type = PIN_OTYPER_PP,
};

/*** Smartcard auxiliary RFU pins (unused for now) ****/
/* Smartcard AUX1 is PE5 */
#define SMARTCARD_AUX1_PORT             GPIOE_BASE
#define SMARTCARD_AUX1_PIN              5
const gpio_config_t gpio_smartcard_aux1 = {
        .port = SMARTCARD_AUX1_PORT,
        .pin = SMARTCARD_AUX1_PIN,
        .set_mask = GPIO_SET_MODE | GPIO_SET_PUPD | GPIO_SET_SPEED | GPIO_SET_TYPE,
        .mode = PIN_OUTPUT_MODE,
        .pupd = GPIO_PULLDOWN,
        .speed = PIN_VERY_HIGH_SPEED,
        .type = PIN_OTYPER_PP,
};
/* Smartcard AUX2 is PE6 */
#define SMARTCARD_AUX2_PORT             GPIOE_BASE
#define SMARTCARD_AUX2_PIN              6
const gpio_config_t gpio_smartcard_aux2 = {
        .port = SMARTCARD_AUX2_PORT,
        .pin = SMARTCARD_AUX2_PIN,
        .set_mask = GPIO_SET_MODE | GPIO_SET_PUPD | GPIO_SET_SPEED | GPIO_SET_TYPE,
        .mode = PIN_OUTPUT_MODE,
        .pupd = GPIO_PULLDOWN,
        .speed = PIN_VERY_HIGH_SPEED,
        .type = PIN_OTYPER_PP,
};

/* Initialize the CONTACT pin */
static volatile uint8_t platform_SC_gpio_smartcard_contact_changed = 0;

void gpio_smartcard_contact_IRQhandler(void){
	/* Clear the EXTI corresponding to the line */
	set_reg_bits(EXTI_PR, (0x1 << gpio_smartcard_contact.pin));
	platform_SC_gpio_smartcard_contact_changed = 1;
	
	return;
}

void platform_init_smartcard_contact(void)
{
    gpio_set_config(&gpio_smartcard_contact);

	platform_SC_gpio_smartcard_contact_changed = 1;

	/* Initialize the handler */
	gpio_register_exti_handler(&gpio_smartcard_contact, GPIO_EXTI_RISING_FALLING, gpio_smartcard_contact_IRQhandler);

        return; 
}

void platform_init_io_dir(void){
        gpio_set_config(&gpio_smartcard_io_dir);
}

static inline void platform_io_dir_read(void){
    gpio_clear(&gpio_smartcard_io_dir);
}

static inline void platform_io_dir_write(void){
    gpio_set(&gpio_smartcard_io_dir);
}

/* Initialize the RST pin */
void platform_init_smartcard_rst(void)
{
        gpio_set_config(&gpio_smartcard_rst);

        /* Maintain the RST at low */
        gpio_clear(&gpio_smartcard_rst);

        return;
}

/* Initialize the Vcc pin */
void platform_init_smartcard_vcc(void)
{
        gpio_set_config(&gpio_smartcard_vcc);

        /* For now, maintain the Vcc at zero (low) */   
	gpio_set(&gpio_smartcard_vcc);

        return;
}

/* Initialize the Vpp pin */
void platform_init_smartcard_vpp(void)
{
        gpio_set_config(&gpio_smartcard_vpp);

        /* Vpp is set low. Not used for now */
        gpio_set(&gpio_smartcard_vpp);

        return;
}

void platform_set_smartcard_rst(uint8_t val)
{
        gpio_set_value(&gpio_smartcard_rst, val);

        return;
}

void platform_set_smartcard_vcc(uint8_t val)
{
        gpio_set_value(&gpio_smartcard_vcc, (~val) & 0x01);

        return;
}

void platform_set_smartcard_vpp(uint8_t val)
{
        gpio_set_value(&gpio_smartcard_vpp, (~val) & 0x01);

        return;
}


/* Fixed delay of a given number of microseconds */
static void SC_delay_microseconds(uint64_t microseconds_timeout){
        uint64_t start_tick, curr_tick;

        if(microseconds_timeout == 0){
                return;
        }
        start_tick = platform_get_microseconds_ticks();
        /* Now wait */
        curr_tick = start_tick;
        while((curr_tick - start_tick) <= microseconds_timeout){
                curr_tick = platform_get_microseconds_ticks();
        }

        return;
}

static void SC_delay_milliseconds(uint64_t milliseconds_timeout){
	SC_delay_microseconds(milliseconds_timeout * 1000ULL);
	return;
}

/* The SMARTCARD_CONTACT pin is at state high (pullup to Vcc) when no card is 
 * not present, and at state low (linked to GND) when the card is inserted.
 */
static volatile uint8_t platform_SC_is_smartcard_inserted = 0;

static uint8_t _platform_is_smartcard_inserted(void){
#if (SMARTCARD_CONTACT_ACTIVE == 1)
	if(platform_SC_gpio_smartcard_contact_changed == 1){
		/* Wait for a stable state and sample it */
		/* FIXME: this delay should be preferably non blocking */
		SC_delay_milliseconds(100);
		platform_SC_is_smartcard_inserted = ((~gpio_get(&gpio_smartcard_contact)) & 0x1);
		platform_SC_gpio_smartcard_contact_changed = 0;
	}
	return platform_SC_is_smartcard_inserted;
#else
	return 1;
#endif
}

uint8_t platform_is_smartcard_inserted(void)
{
	/* NOTE: we only do this for GoodUSB because we do not have
	 * insertion switch on the discovery.
 	*/
	return _platform_is_smartcard_inserted();
}

int platform_smartcard_clock_rounding(uint32_t target_freq, uint32_t *target_freq_rounded)
{
	uint32_t usart_bus_clk;
	unsigned int i;

	if(target_freq_rounded == NULL) {
		goto err;
	}

	/* Get the usart clock */
	usart_config_t *config = &smartcard_usart_config;
	usart_bus_clk = usart_get_bus_clock(config);

	/* Find the best suitable target frequency <= target frequency with regards to our USART core frequency
	 * (i.e. as a divisor of our USART core frequency).
	 */
	if(target_freq > usart_bus_clk) {
		/* The target frequency is > USART frequency: there is no need to try ... */
		goto err;
	}

	i = target_freq;
	while(i != 0) {
		if(((usart_bus_clk / i / 2) * i * 2) == usart_bus_clk){
    			break;
		}
		i--;
	}

	*target_freq_rounded = i;

	return 0;

err:

	return -1;

}

/* Initialize the USART in smartcard mode as
 * described in the datasheet, as well as smartcard
 * associated GPIOs.
 */
static int platform_smartcard_clocks_init(usart_config_t *config, uint32_t *target_freq, uint8_t target_guard_time, uint32_t *etu)
{
	uint32_t usart_bus_clk, prescaler;
	if(platform_smartcard_clock_rounding(*target_freq, target_freq) != 0) {
    		goto err;
	}
	log_printf("Rounding target freguency to %d\n", *target_freq);
        
	/* First, get the usart clock */
	usart_bus_clk = usart_get_bus_clock(config);

	/* Then, compute the baudrate depending on the target frequency */
	/* Baudrate is the clock frequency divided by one ETU (372 ticks by default, possibly negotiated). 
	 * For example, a frequency of 3.5MHz gives 3.5MHz / 372 ~= 9408 bauds for a 372 ticks ETU. */
	config->baudrate = (*target_freq) / (*etu);

	/* Finally, adapt the CLK clock pin frequency to the target frequency using the prescaler.
	 * Also, adapt the guard time (expressed in bauds).
	 * Frequency is = (APB_clock / PRESCALER) = (42MHz / 12) = 3.5MHz. The value of the prescaler field is x2 (cf. datasheet).
	 */
	prescaler = (usart_bus_clk / (*target_freq));
	config->guard_time_prescaler = ((prescaler / 2) << USART_GTPR_PSC_Pos) | (target_guard_time << USART_GTPR_GT_Pos);

	return 0;

err:

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

static int platform_smartcard_hw_init(smartcard_init_type type)
{	
	if((type == GPIOS_EN) || (type == ALL_EN)){
		/* Initialize the GPIOs */
		platform_init_smartcard_contact();
		platform_init_smartcard_vcc();
		platform_init_smartcard_vpp();
		platform_init_smartcard_rst();
		platform_init_io_dir();
	}

	if((type == USART_EN) || (type == ALL_EN)){
		/* Initialize the USART in smartcard mode */
		log_printf("==> Enable USART%d in smartcard mode!\n", smartcard_usart_config.usart);
		usart_init(&smartcard_usart_config);
		log_printf("==> Enable OK\n");
	}
	
	if(type == USART_DIS){
		/* Disable the smartcard USART */
		usart_disable(&smartcard_usart_config);
	}

	return 0;
}

int platform_smartcard_init(void){
	/* Reinitialize global variables */
	platform_SC_pending_receive_byte = 0;
	platform_SC_pending_send_byte = 0;
	platform_SC_byte = 0;

	/* Reset our smartcard USART configuration */
	smartcard_usart_config = default_smartcard_usart_config;

	/* Disable USART block to reinitialize its state and flush the
	 * buffers.
	 */
	platform_smartcard_hw_init(USART_DIS);

	return platform_smartcard_hw_init(ALL_EN);
}

/* Adapt clocks and guard time depending on what has been received */
int platform_SC_adapt_clocks(uint32_t *etu, uint32_t *frequency){
	uint32_t old_mask;
	usart_config_t *config = &smartcard_usart_config;

	if((etu == NULL) || (frequency == NULL)){
		goto err;
	}

	if(config->mode != SMARTCARD){
		goto err;
	}
	old_mask = config->set_mask;
	/* Adapt the clocks configuration in our structure */
	if(platform_smartcard_clocks_init(config, frequency, 1, etu)){
		goto err;
	}
	config->set_mask = USART_SET_BAUDRATE | USART_SET_GUARD_TIME_PS;
	/* Adapt the configuration at the USART level */
	usart_init(&smartcard_usart_config);
	config->set_mask = old_mask;

	return 0;
err:
	return -1;
}

/*
 * Low level related functions: we handle the low level USAT/smartcard
 * bytes send and receive stuff here.
 */
static volatile uint8_t dummy_usart_read = 0;
static void platform_smartcard_irq(void){
	/* Check if we have a parity error */
	if((get_reg(r_CORTEX_M_USART_SR(smartcard_usart_config.usart), USART_SR_PE)) && (platform_SC_pending_send_byte != 0)){
		/* Parity error, program a resend */
		platform_SC_pending_send_byte = 3;
		/* Dummy read of the DR register to ACK the interrupt */
		dummy_usart_read = (*r_CORTEX_M_USART_DR(SMARTCARD_USART)) & 0xff;
		return;
	}
	/* Check if we have a framing error */
	if((get_reg(r_CORTEX_M_USART_SR(smartcard_usart_config.usart), USART_SR_FE)) && (platform_SC_pending_send_byte != 0)){
		/* Frame error, program a resend */
		platform_SC_pending_send_byte = 4;
		/* Dummy read of the DR register to ACK the interrupt */
		dummy_usart_read = (*r_CORTEX_M_USART_DR(SMARTCARD_USART)) & 0xff;
		return;
	}
	/* We have sent our byte */
	if((get_reg(r_CORTEX_M_USART_SR(smartcard_usart_config.usart), USART_SR_TC)) && (platform_SC_pending_send_byte != 0)){
        trig(TRIG_IRQ_PUTC);
		/* Clear TC */
	        clear_reg_bits(r_CORTEX_M_USART_SR(smartcard_usart_config.usart), USART_SR_TC_Msk);
		/* Signal that the byte has been sent */
		platform_SC_pending_send_byte = 2;
		return;
	}	
	/* We can actually read data */
	if(get_reg(r_CORTEX_M_USART_SR(smartcard_usart_config.usart), USART_SR_RXNE)){
		platform_SC_byte = (*r_CORTEX_M_USART_DR(SMARTCARD_USART)) & 0xff;
		if(platform_SC_pending_send_byte == 0){
            trig(TRIG_IRQ_GETC);
			platform_SC_pending_receive_byte = 1;
		}
		return;
	}

	return;
}

/* Set the direct convention at low level */
int platform_SC_set_direct_conv(void){
	return 0;
}

/* Set the inverse convention at low level */
int platform_SC_set_inverse_conv(void){
	uint32_t old_mask;
	usart_config_t *config = &smartcard_usart_config;
	uint64_t t, start_tick, curr_tick;

	/* Flush the pending received byte from the USART block */
	dummy_usart_read = (*r_CORTEX_M_USART_DR(SMARTCARD_USART)) & 0xff;	
	/* ACK the pending parity errors */
	dummy_usart_read = get_reg(r_CORTEX_M_USART_SR(smartcard_usart_config.usart), USART_SR_PE);

	/* Reconfigure the usart with an ODD parity */
	if(config->mode != SMARTCARD){
		goto err;
	}
	old_mask = config->set_mask;
	/* Adapt the configuration at the USART level */
	config->set_mask = USART_SET_PARITY;
	config->parity = USART_CR1_PCE_EN | USART_CR1_PS_ODD,
	usart_init(&smartcard_usart_config);
	config->set_mask = old_mask;

	/* Get the pending byte again (with 9600 ETU at 372 timeout) to send the proper
	 * parity ACK to the card and continue to the next bytes ...
	 */
	t = ((uint64_t)9600 * 372 * 1000) / 3500000;
        start_tick = platform_get_microseconds_ticks();
        curr_tick = start_tick;
	while(platform_SC_getc((uint8_t*)&dummy_usart_read, 0, 0)){
		if((curr_tick - start_tick) > t){
			goto err;
		}
		curr_tick = platform_get_microseconds_ticks();
	}

	return 0;

err:

	return -1;
}


/* Low level flush of our receive/send state, in order
 * for the higher level to be sure that everything is clean
 */
void platform_SC_flush(void){
        /* Flushing the receive/send state is only a matter of cleaning
         * our ring buffer!
         */
        platform_SC_pending_receive_byte = platform_SC_pending_send_byte = 0;
}

/* Low level char PUSH/POP functions */
static inline uint8_t platform_SC_char_pop(void){
	return platform_SC_byte;
}
static inline void platform_SC_char_push(uint8_t c){
	(*r_CORTEX_M_USART_DR(SMARTCARD_USART)) = c;
	return;
}

/* Smartcard putc and getc handling errors: 
 * The getc function is non blocking */
int platform_SC_getc(uint8_t *c, uint32_t timeout, uint8_t reset){
	platform_io_dir_read();
	if(c == NULL){
		return -1;
	}
	if(platform_SC_pending_receive_byte != 1){
		return -1;
	}
	/* Data is ready, go ahead */
	*c = platform_SC_char_pop();
	platform_SC_pending_receive_byte = 0;

	return 0;
}

/* The putc function is non-blocking and checks
 * for errors. In the case of errors, try to send the byte again.
 */
int platform_SC_putc(uint8_t c, uint32_t timeout, uint8_t reset){
	platform_io_dir_write();
	if(reset){
		platform_SC_pending_send_byte = 0;
		return 0;
	}
	if((platform_SC_pending_send_byte == 0) || (platform_SC_pending_send_byte >= 3)){
		platform_SC_pending_send_byte = 1;
		platform_SC_char_push(c);
		return -1;
	}
	if(platform_SC_pending_send_byte == 2){
		/* The byte has been sent */
		platform_SC_pending_send_byte = 0;
		return 0;
	}

	return -1;
}

/**** Cycles counter for good precision ***********/
#define DEMCR_TRCENA    0x01000000
/* Core Debug registers */
#define SCB_DEMCR       (*((volatile unsigned int *)0xE000EDFC))
#define DWT_CTRL        (*(volatile unsigned int *)0xE0001000)
#define CYCCNTENA       (1<<0)
#define DWT_CYCCNT      ((volatile unsigned int *)0xE0001004)

static void cortex_m4_cycles_reset(void)
{
    /* Enable DWT */
    SCB_DEMCR |= DEMCR_TRCENA;
    *DWT_CYCCNT = 0;
    /* Enable CPU cycle counter */
    DWT_CTRL |= CYCCNTENA;
}

static unsigned int get_cortex_m4_cycles(void){
        return *((volatile unsigned int *)DWT_CYCCNT);
}


void platform_smartcard_reinit(void){
        /* Reset our smartcard USART configuration */
        smartcard_usart_config = default_smartcard_usart_config;

        log_printf("==> Reinit USART%d\n", smartcard_usart_config.usart);
        usart_disable(&smartcard_usart_config);
        usart_enable(&smartcard_usart_config);
        cortex_m4_cycles_reset();

        return;
}


/* Get ticks/time in microseconds */
uint64_t platform_get_microseconds_ticks(void){
        /* In order to get microseconds precision, we use the cycle counter */
        return (get_cortex_m4_cycles() / (PROD_CORE_FREQUENCY / 1000UL));
}


void platform_SC_reinit_smartcard_contact(void){
	platform_SC_is_smartcard_inserted = 0;
        return;
}

void platform_smartcard_lost(void)
{
	return;
}

static void (*volatile user_handler_action)(void) = NULL;
void platform_smartcard_register_user_handler_action(void (*action)(void))
{
        if(action == NULL){
                return;
        }
        user_handler_action = action;
        return;
}

void platform_SC_reinit_iso7816(void){
	platform_SC_pending_receive_byte = 0;
	platform_SC_pending_send_byte = 0;
	platform_SC_byte = 0;
	dummy_usart_read = 0;
	platform_SC_is_smartcard_inserted = ((~gpio_get(&gpio_smartcard_contact)) & 0x1);
	platform_SC_gpio_smartcard_contact_changed = 0;

        return;
}

int platform_smartcard_set_1ETU_guardtime(void){
       /* We are already at 1 ETU guard time, so return OK */
       return 0;
}

