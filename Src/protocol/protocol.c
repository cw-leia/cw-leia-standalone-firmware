#include "config.h"
#include "console.h"
#include "helpers.h"
#include "protocol.h"
#include "smartcard.h"
#include "smartcard_iso7816_platform.h"
#include "triggers.h"
#include "leds.h"
//#include "timers.h" TODO

/* Local card in global variable */
static SC_Card card;
SC_Card* get_current_card(void)
{
    return &card;
}

/* Magic handling DFU. We put this in a dedicated section that will not
 * be initialized by our startup procedure.
 */
__attribute__((section(".noninit"))) volatile uint32_t dfu_mode_selected;
static uint8_t protocol_goto_dfu(__attribute__((unused)) SC_Card* card, __attribute__((unused)) command_cb_args_t* args);

#define PROTOCOL_AVAILABLE_COMMANDS_N 9
static command_t available_commands[PROTOCOL_AVAILABLE_COMMANDS_N] = {
    /* name, byte command,                              max_size,                    callback       Blink LED?   */
    {"send_APDU",     'a',               COMMANDS_BUFFER_MAX_LEN,             protocol_send_APDU,  1},
    {"get_ATR",       't',                                     0,               protocol_get_ATR,  1},
    {"reset",         'r',                                     0,            protocol_reset_card,  1},
    {"configure",     'c',         sizeof(protocol_config_pts_t),         protocol_configure_pts,  1},
    {"set_triggers",  'O', sizeof(protocol_config_trigger_set_t),  protocol_trigger_set_strategy,  1},
    {"get_triggers",  'o',                       sizeof(uint8_t),  protocol_trigger_get_strategy,  1},
    {"card_inserted", '?',                                     0,      protocol_is_card_inserted,  0},
    {"get_timers",    'm',                                     0,            protocol_get_timers,  1},
    {"goto_dfu",      'u',                                     0,              protocol_goto_dfu,  1},
};

/* 0x00 Status OK
 * 0x01 Card not Inserted
 * 0xFF Unknown error  */
#define PLATFORM_STATUS_OK              0x00
#define PLATFORM_ERR_CARD_NOT_INSERTED  0x01
#define PLATFORM_ERR_UNKNOWN_ERROR      0xFF

/* Timers for the current command */
static volatile uint32_t delta_t = 0, delta_t_answer = 0;

uint8_t protocol_get_timers(__attribute__((unused)) SC_Card* card, command_cb_args_t* args)
{

    *((uint32_t*)&args->response) = delta_t;
    *(((uint32_t*)&args->response) + 1) = delta_t_answer;

    args->response_size = 2 * sizeof(uint32_t);

    return PLATFORM_STATUS_OK;
}

static uint8_t protocol_goto_dfu(__attribute__((unused)) SC_Card* card, __attribute__((unused)) command_cb_args_t* args) {

    dfu_mode_selected = DFU_MAGIC;

    return PLATFORM_STATUS_OK;
}

uint8_t protocol_send_APDU(SC_Card* card, command_cb_args_t* args)
{

    SC_APDU_cmd* apduc = (SC_APDU_cmd*)&args->buffer;
    SC_APDU_resp* resp = (SC_APDU_resp*)&args->response;

    SC_print_APDU(apduc);

    if(platform_is_smartcard_inserted() == 0) {
        return PLATFORM_ERR_CARD_NOT_INSERTED;
    }

    delta_t = platform_get_microseconds_ticks();
    SC_send_APDU(apduc, resp, card);
    delta_t = platform_get_microseconds_ticks() - delta_t;
    delta_t_answer = resp->delta_t_answer;
    resp->delta_t = delta_t;
    SC_print_RESP(resp);

    args->response_size = 6 + 8 + resp->le;

    return PLATFORM_STATUS_OK;
}

uint8_t protocol_configure_pts(SC_Card* card, command_cb_args_t* args)
{

    protocol_config_pts_t* config;

    if(platform_is_smartcard_inserted() == 0) {
        return PLATFORM_ERR_CARD_NOT_INSERTED;
    }

    config = (protocol_config_pts_t*) &args->buffer;

    args->response_size = 0;

    delta_t = platform_get_microseconds_ticks();
    delta_t_answer = 0;
    if(SC_fsm_init(card, config->negotiate_pts, config->negotiate_baudrate, config->protocol, config->etu, config->freq, 0) != 0) {
        goto err;
    }
    delta_t = platform_get_microseconds_ticks() - delta_t;

    // Print some debugs
    SC_print_Card(card);

    return PLATFORM_STATUS_OK;

err:
    delta_t = platform_get_microseconds_ticks() - delta_t;
    return PLATFORM_ERR_UNKNOWN_ERROR;
}

uint8_t protocol_trigger_set_strategy(__attribute__((unused)) SC_Card* card, command_cb_args_t* args)
{

    protocol_config_trigger_set_t* config;
    config = (protocol_config_trigger_set_t*) &args->buffer;

    delta_t = delta_t_answer = 0;

    if(trigger_set_strategy(config->index, &config->strategy)) {
        goto err;
    }

    return PLATFORM_STATUS_OK;

err:
    return PLATFORM_ERR_UNKNOWN_ERROR;
}

uint8_t protocol_trigger_get_strategy(__attribute__((unused)) SC_Card* card, command_cb_args_t* args)
{
    uint8_t strategy_index;

    strategy_index = (uint8_t) args->buffer[0];

    delta_t = delta_t_answer = 0;

    if(trigger_get_strategy(strategy_index, (trigger_strategy_t*) &args->response)) {
        goto err;
    }

    args->response_size = sizeof(trigger_strategy_t);

    return PLATFORM_STATUS_OK;

err:
    return PLATFORM_ERR_UNKNOWN_ERROR;
}

uint8_t protocol_is_card_inserted(__attribute__((unused)) SC_Card* card, command_cb_args_t* args)
{
    args->response_size = 1;

    delta_t = delta_t_answer = 0;

    if(platform_is_smartcard_inserted()) {
        args->response[0] = 1;

    } else {
        args->response[0] = 0;
    }

    return PLATFORM_STATUS_OK;

}

uint8_t protocol_reset_card(SC_Card* card, command_cb_args_t* args)
{


    delta_t_answer = 0;
    delta_t = platform_get_microseconds_ticks();
    if(SC_fsm_init(card, 0, 0, 0, 0, 0, 0)) {
        goto err;
    }
    delta_t = platform_get_microseconds_ticks() - delta_t;

    args->response_size = 0;

    return PLATFORM_STATUS_OK;

err:
    delta_t = platform_get_microseconds_ticks() - delta_t;
    return PLATFORM_ERR_UNKNOWN_ERROR;
}


uint8_t protocol_get_ATR(SC_Card* card, command_cb_args_t* args)
{

    delta_t = delta_t_answer = 0;

    if(platform_is_smartcard_inserted() == 0) {
        return PLATFORM_ERR_CARD_NOT_INSERTED;
    }

    /* Print the ATR */
    SC_print_Card(card);

    args->response_size = sizeof(card->info.atr);
    local_memcpy(args->response, &(card->info.atr), sizeof(card->info.atr));

    return PLATFORM_STATUS_OK;
}

static int __protocol_read_char_uart(uint8_t* command)
{
    *command = console_getc();
    return 1;
}

/* Flags telling that we have a command in progress and handling it
 * at the console interrupt level.
 */
static const uint8_t command_in_progress_char = 'w'; /* wait extension flag */
void __protocol_send_wait_extension(uint8_t do_blink)
{
    if(do_blink == 1){
        led_error_on();
        HAL_Delay(1); /* NOTE: we can't blink for too long, we are blocking here! */
        led_error_off();
    }
    console_putc_nb(command_in_progress_char);
}

void protocol_parse_cmd(void)
{
    command_cb_args_t args;
    local_memset(&args, 0, sizeof(command_cb_args_t));
    uint8_t command;
    uint32_t size;
    uint8_t i; // Index used to go through commands[]
    uint32_t j; // Index used to count the bytes of the payload
    uint8_t err;
    uint8_t tmp;

    while (1) {
idle:
        /* Do we have to reset? */
        if(dfu_mode_selected == DFU_MAGIC){
            /* Reset and go to DFU */
            NVIC_SystemReset();
        }

        if (platform_SC_is_lost()) {
            dbg_log("[X] Smart card lost\r\n");
            dbg_flush();
            SC_smartcard_lost(get_current_card());
        }
        dbg_log("------------------------------\r\n");
        dbg_log("Waiting for command\r\n");
        dbg_flush();

        /* We are waiting for a CMD */
        console_putc('W');

        while(__protocol_read_char_uart(&command) == -1);

        dbg_log("Received command : '%c'\r\n", command);
        dbg_flush();

        if(command == ' ' || command == 0) {
            goto idle;
        }

        /* We are looking for a matching CMD */
        for (i = 0; i < PROTOCOL_AVAILABLE_COMMANDS_N ; i++) {
            if (available_commands[i].o_command == command) {
                goto found;
            }
        }

        /* No command found */
        goto unk;
found:
	/* Blink our activity LEDs if asked to */
	if(available_commands[i].blink_led == 1){
	        led_error_on();
        	HAL_Delay(25);
	        led_error_off();
	}

        /* Get the size of the command payload */
        while(__protocol_read_char_uart(&tmp) == -1);

        size = tmp << 24;

        while(__protocol_read_char_uart(&tmp) == -1);

        size += tmp << 16;

        while(__protocol_read_char_uart(&tmp) == -1);

        size += tmp << 8;

        while(__protocol_read_char_uart(&tmp) == -1);

        size += tmp;

        dbg_log("[Protocol] payload size = %d\r\n", size);
        dbg_flush();

        /* If the size of the payload is bigger than the definition, go error */
        if (size > available_commands[i].max_size) {
            goto err;
        }

        for (j = 0; j < size; j++) {
            while(__protocol_read_char_uart(& (args.buffer[j])) == -1);
        }

        args.buffer_size = size;

        dbg_log("[Protocol] Calling function %s...\r\n", available_commands[i].name);
        dbg_flush();

        err = available_commands[i].callback(get_current_card(), &args);

        dbg_log("[Protocol] Response Header : S%02x", err);
        console_putc('S');
        console_putc((char) err);

        dbg_log("R%08x\r\n", args.response_size);
        dbg_flush();

        console_putc('R');
        console_putc((char) ((args.response_size & 0x000000FF)));
        console_putc((char) ((args.response_size & 0x0000FF00) >>  8));
        console_putc((char) ((args.response_size & 0x00FF0000) >> 16));
        console_putc((char) ((args.response_size & 0xFF000000) >> 24));

        if(args.response_size != 0) {
            console_write((char*)args.response, args.response_size);
        }

	/* Blink our activity LEDs if asked to */
	if(available_commands[i].blink_led == 1){
	        led_error_on();
        	HAL_Delay(25);
	        led_error_off();
	}

        goto idle;
unk:
        dbg_log("[Protocol] Unknown command '%c' (%d)\r\n", command, command);
        dbg_flush();
        console_putc('U');
        goto idle;
err:
        dbg_log("[Protocol] Error\r\n");
        dbg_flush();
        console_putc('E');
        goto idle;
    }
}
