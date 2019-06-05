#include "protocol.h"
#include "ring_buffer.h"
#include "debug.h"
#include "helpers.h"
#include "init.h"
#include "smartcard.h"
#include "stm32f4xx_usart.h"
#include "uart.h"
#include "triggers.h"
#include "smartcard_iso7816_platform.h"


#define PROTOCOL_AVAILABLE_COMMANDS_N 7
static command_t available_commands[PROTOCOL_AVAILABLE_COMMANDS_N] = {
      /* name, byte command,                          max_size,              callback*/
      {"send_APDU",     'a',                                   600,             protocol_send_APDU},
      {"get_ATR",       't',                                     0,               protocol_get_ATR},
      {"reset",         'r',                                     0,            protocol_reset_card},
      {"configure",     'c',         sizeof(protocol_config_pts_t),         protocol_configure_pts},
      {"set_triggers",  'O', sizeof(protocol_config_trigger_set_t),  protocol_trigger_set_strategy},
      {"get_triggers",  'o',                       sizeof(uint8_t),  protocol_trigger_get_strategy},
      {"card_inserted", '?',                                     0,      protocol_is_card_inserted},
};

/* 0x00 Status OK
 * 0x01 Card not Inserted
 * 0xFF Unknown error  */
#define PLATFORM_STATUS_OK             0x00
#define PLATFORM_ERR_CARD_NOT_INSERTED 0x01
#define PLATFORM_ERR_UNKNOWN_ERROR     0xFF


uint8_t protocol_send_APDU(SC_Card *card, command_cb_args_t *args) {

  SC_APDU_cmd *apduc = (SC_APDU_cmd *)&args->buffer;
  SC_APDU_resp *resp = (SC_APDU_resp *)&args->response;
  uint32_t i;

  SC_print_APDU(apduc);
  SC_send_APDU(apduc, resp, card);
  SC_print_RESP(resp);

  args->response_size = 6 + resp->le;

  return PLATFORM_STATUS_OK;

err:
  return PLATFORM_ERR_UNKNOWN_ERROR;
}

uint8_t protocol_configure_pts(SC_Card *card, command_cb_args_t *args){
  protocol_config_pts_t* config;

  if(platform_is_smartcard_inserted() == 0){
    return PLATFORM_ERR_CARD_NOT_INSERTED;
  }

  config = (protocol_config_pts_t*) &args->buffer;

  args->response_size = 0;

  if(SC_fsm_init(card, config->negotiate_pts, config->negotiate_baudrate, config->protocol, config->etu, config->freq) != 0){
    goto err;
  }

  return PLATFORM_STATUS_OK;

err:
  return PLATFORM_ERR_UNKNOWN_ERROR;
}

uint8_t protocol_trigger_set_strategy(SC_Card *card, command_cb_args_t *args){
  protocol_config_trigger_set_t* config;

  config = (protocol_config_trigger_set_t*) &args->buffer;

  if(trigger_set_strategy(config->index, &config->strategy)){
    goto err;
  }

  return PLATFORM_STATUS_OK;

err:
  return PLATFORM_ERR_UNKNOWN_ERROR;
}

uint8_t protocol_trigger_get_strategy(SC_Card *card, command_cb_args_t *args){
  uint8_t strategy_index;

  strategy_index = (uint8_t) args->buffer[0];


  if(trigger_get_strategy(strategy_index, (trigger_strategy_t*) &args->response)){
    goto err;
  }

  args->response_size = sizeof(trigger_strategy_t);

  return PLATFORM_STATUS_OK;

err:
  return PLATFORM_ERR_UNKNOWN_ERROR;
}

uint8_t protocol_is_card_inserted(SC_Card *card, command_cb_args_t *args){
  args->response_size = 1;

  if(platform_is_smartcard_inserted()){
    args->response[0] = 1;
  }
  else{
    args->response[0] = 0;
  }

  return PLATFORM_STATUS_OK;

}

uint8_t protocol_reset_card(SC_Card *card, command_cb_args_t *args){

    SC_fsm_init(card, 0, 0, 0, 0, 0);

    args->response_size = 0;

    return PLATFORM_STATUS_OK;
}


uint8_t protocol_get_ATR(SC_Card *card, command_cb_args_t *args) {

  if(platform_is_smartcard_inserted() == 0){
    return PLATFORM_ERR_CARD_NOT_INSERTED;
  }

  /* Print the ATR */
  SC_print_Card(card);

  args->response_size = sizeof(card->info.atr);
  memcpy(args->response, &(card->info.atr), sizeof(card->info.atr));


  return PLATFORM_STATUS_OK;

err:
  return PLATFORM_ERR_UNKNOWN_ERROR;
}

int protocol_read_char_uart(volatile s_ring_t* ring_buffer, char* command){
    usart_mute_rxneie(PROD_CONSOLE_USART);
    int ret = ring_buffer_read_char(ring_buffer, command);
    usart_unmute_rxneie(PROD_CONSOLE_USART);

    return ret;
}

void protocol_parse_cmd(volatile s_ring_t* ring_buffer, SC_Card *card) {
  command_cb_args_t args = {0, {}, {}, 0}; // (buffer_size, buffer, response, response_size)
  char command;
  uint32_t size;
  uint8_t i; // Index used to go through commands[]
  uint32_t j; // Index used to count the bytes of the payload
  uint8_t err;
  uint8_t tmp;

  while (1) {
  idle:
    dbg_log("------------------------------\n");
    dbg_log("Waiting for command\n");

    consoleputc('W');         // We say we are waiting for a CMD
    while(protocol_read_char_uart(ring_buffer, &command) == -1);

    dbg_log("Received command : '%c'\n", command);


    if(command == ' ' || command == 0){
        goto idle;
    }

    for (i = 0; i < PROTOCOL_AVAILABLE_COMMANDS_N ; i++) { // Looking for a corresponding CMD
      if (available_commands[i].o_command == command)
        goto found;
    }

    goto unk; // No CMD found with the 'command' keyword
  found:
    // Get the size of the command data

    dbg_log("[Protocol] payload size = ");
    while(protocol_read_char_uart(ring_buffer, &tmp) == -1);
    size = tmp << 24;
    while(protocol_read_char_uart(ring_buffer, &tmp) == -1);
    size += tmp << 16;
    while(protocol_read_char_uart(ring_buffer, &tmp) == -1);
    size += tmp << 8;
    while(protocol_read_char_uart(ring_buffer, &tmp) == -1);
    size += tmp;

    dbg_log("%d\n", size);

    // if the size of the payload is bigger than the definition, go error
    if (size > available_commands[i].max_size)
      goto err;

    for (j = 0; j < size; j++) {
      while(protocol_read_char_uart(ring_buffer, &(args.buffer[j])) == -1);
    }
    args.buffer_size = size;

    dbg_log("[Protocol] Calling function %s...\n", available_commands[i].name);

    err = available_commands[i].callback(card, &args);


    dbg_log("[Protocol] Response Header : S%d", err);
    consoleputc('S');
    consoleputc((char) err);

    dbg_log("R%d\n", args.response_size);
    dbg_flush();

    consoleputc('R');
    consoleputc((char) ((args.response_size&0x000000FF)));
    consoleputc((char) ((args.response_size&0x0000FF00) >>  8));
    consoleputc((char) ((args.response_size&0x00FF0000) >> 16));
    consoleputc((char) ((args.response_size&0xFF000000) >> 24));

    console_flush();
    if(args.response_size!= 0){
      for(j=0; j<args.response_size;j++){
          consoleputc((char) args.response[j]);
      }
      //console_log_raw((char *)args.response, args.response_size); // Utilise le ring buffer foireux.
    }
    console_flush();

    goto idle;
  unk:
    dbg_log("[Protocol] Unknown command '%c' (%d)\n", command, command);
    dbg_flush();
    consoleputc('U');
    goto idle;
  err:
    dbg_log("[Protocol] Error\n");
    dbg_flush();
    consoleputc('E');
    goto idle;
  }
}
