#ifndef __PROTOCOL_H_
#define __PROTOCOL_H_

#include "smartcard.h"
#include "triggers.h"

#define max(a, b) (((a) > (b)) ? (a) : (b))
#define min(a, b) (((a) < (b)) ? (a) : (b))
#define COMMANDS_BUFFER_MAX_LEN (max(max(sizeof(SC_APDU_cmd), sizeof(SC_APDU_resp)), sizeof(SC_Card)))

typedef struct __attribute__((packed)) {
    uint32_t buffer_size;
    uint8_t buffer[COMMANDS_BUFFER_MAX_LEN];
    uint8_t response[COMMANDS_BUFFER_MAX_LEN];
    uint32_t response_size;
} command_cb_args_t;

typedef uint8_t (*cb_command)(SC_Card*, command_cb_args_t*);

typedef struct __attribute__((packed)) {
    char name[30];               // Command name
    uint8_t o_command;           // Char that identify the command
    uint32_t max_size;           // Data max size <= COMMANDS_BUFFER_MAX_LEN
    cb_command callback;         // Callback to the function
    uint8_t blink_led;           // Do we want the activity LED to blink?
} command_t;

typedef struct __attribute__((packed)) {
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

typedef struct __attribute__((packed)) {
    uint8_t index;
    trigger_strategy_t strategy;
} protocol_config_trigger_set_t;

void smartreader_protocol_parse_cmd(void);

#endif // __PROTOCOL_H
