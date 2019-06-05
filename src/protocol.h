#ifndef __PROTOCOL_H_
#define __PROTOCOL_H_

#include "helpers.h"
#include "smartcard.h"
#include "ring_buffer.h"
#include "triggers.h"

#define COMMANDS_BUFFER_MAX_LEN 600

typedef struct {
  uint32_t buffer_size;
  uint8_t buffer[COMMANDS_BUFFER_MAX_LEN];
  uint8_t response[COMMANDS_BUFFER_MAX_LEN];
  uint32_t response_size;
} command_cb_args_t;

typedef uint8_t (*cb_command)(SC_Card *, command_cb_args_t *);

typedef struct {
  char name[30];       // Command name
  uint8_t o_command;   // Char that identify the command
  uint32_t max_size;   // Data max size <= COMMANDS_BUFFER_MAX_LEN
  cb_command callback; // Callback to the function

} command_t;

typedef struct {
  uint8_t protocol;            // Actual protocol to use.
                               //     0 if no protocol is forced,
                               //     1 for T=0, 2 for T=1.
  uint32_t etu;                // ETU value.
  uint32_t freq;               // Actual frequency to use
                               //      0 for default one
                               //      x for choosen value
  uint8_t negotiate_pts;       // Do the PTS negotiation with the card.
  uint8_t negotiate_baudrate;  // Do the baudrate negotiation with the card.
} protocol_config_pts_t;

typedef struct {
  uint8_t index;
  trigger_strategy_t strategy;
} protocol_config_trigger_set_t;

void protocol_parse_cmd(volatile s_ring_t*, SC_Card *);

uint8_t protocol_send_APDU(SC_Card *card, command_cb_args_t *args);
uint8_t protocol_get_ATR(SC_Card *card, command_cb_args_t *args);
uint8_t protocol_reset_card(SC_Card *card, command_cb_args_t *args);
uint8_t protocol_configure_pts(SC_Card *card, command_cb_args_t *args);
uint8_t protocol_trigger_get_strategy(SC_Card *card, command_cb_args_t *args);
uint8_t protocol_trigger_set_strategy(SC_Card *card, command_cb_args_t *args);
uint8_t protocol_is_card_inserted(SC_Card *card, command_cb_args_t *args);

#endif // __PROTOCOL_H
