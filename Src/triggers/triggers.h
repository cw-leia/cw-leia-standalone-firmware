#ifndef __TRIGGER_H_
#define __TRIGGER_H_

#include <stdint.h>

#define TRIGGER_DEPTH   10
#define STRATEGY_MAX     4
///////////////////////////////////////////
#define TRIG_GET_ATR_PRE                  ((uint32_t)1<<0)
#define TRIG_GET_ATR_POST                 ((uint32_t)1<<1)
#define TRIG_PRE_SEND_APDU_SHORT_T0       ((uint32_t)1<<2)
#define TRIG_PRE_SEND_APDU_FRAGMENTED_T0  ((uint32_t)1<<3)
#define TRIG_PRE_SEND_APDU_T1             ((uint32_t)1<<4)
#define TRIG_POST_RESP_T0                 ((uint32_t)1<<6)
#define TRIG_POST_RESP_T1                 ((uint32_t)1<<7)
#define TRIG_IRQ_PUTC                     ((uint32_t)1<<8)
#define TRIG_IRQ_GETC                     ((uint32_t)1<<9)
///////////////////////////////////////////


typedef struct  __attribute__((packed)) {
    /* From 1 to TRIGGER_DEPTH;
     * 0 means not enabled */
    uint8_t size;
    uint32_t delay; /* delay in milliseconds: delay added to strategy triggers */
    uint8_t single; /* Activate the single mode */
    uint32_t list[TRIGGER_DEPTH];
    uint32_t list_trigged[TRIGGER_DEPTH];
    uint32_t cnt_trigged[TRIGGER_DEPTH];
    uint32_t event_time[TRIGGER_DEPTH];
    uint32_t apply_delay[TRIGGER_DEPTH];
} trigger_strategy_t;

int trigger_init_IO();
int trig_IO();

int trig(uint32_t number);

int trigger_set_strategy(uint8_t index, const trigger_strategy_t* strategy);
int trigger_get_strategy(uint8_t index, trigger_strategy_t* strategy);

#endif // __TRIGGER_H_

