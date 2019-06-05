#ifndef __TRIGGER_H_
#define __TRIGGER_H_

#include "helpers.h"

#define TRIGGER_DEPTH 10
#define STRATEGY_MAX 4
///////////////////////////////////////////
#define TRIG_GET_ATR_PRE                  1
#define TRIG_GET_ATR_POST                 2

#define TRIG_SEND_APDU_FRAGMENTED_T0_PRE  3
#define TRIG_SEND_APDU_SIMPLE_T0_PRE      4
#define TRIG_GET_RESP_FRAGMENTED_T0_PRE   5
#define TRIG_GET_RESP_SIMPLE_T0_PRE       6

#define TRIG_IRQ_PUTC                     7
#define TRIG_IRQ_GETC                     8

#define TRIG_PRE_RESP_T0                  9
///////////////////////////////////////////


typedef struct {
    uint8_t size; // from 1 to TRIGGER_DEPTH; 0 means not enabled
    uint32_t delay;
    uint32_t delay_cnt;
    uint8_t list[TRIGGER_DEPTH];
} trigger_strategy_t;

int trigger_tick_handler();

int trigger_init_IO();
int trig_IO();

int trig(uint8_t number);

int trigger_set_strategy(uint8_t index, const trigger_strategy_t* strategy);
int trigger_get_strategy(uint8_t index, trigger_strategy_t* strategy);

#endif // __TRIGGER_H_

