#include "stm32f4xx_ll_gpio.h"
#include "config.h"
#include "helpers.h"
#include "triggers.h"


static volatile uint8_t trigger_hist[TRIGGER_DEPTH] = {0};
static volatile uint8_t trigger_hist_pt = 0;

static trigger_strategy_t strategies[STRATEGY_MAX];


#if defined(TRIG_GPIO_PORT)
static LL_GPIO_InitTypeDef gpio_trigger = {
    .Pin        = TRIG_GPIO_PIN,
    .Mode       = LL_GPIO_MODE_ALTERNATE,
    .Pull       = LL_GPIO_PULL_NO,
    .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
};
#endif

inline int trig_IO()
{
#if defined(TRIG_GPIO_PORT)
    LL_GPIO_SetOutputPin(TRIG_GPIO_PORT, TRIG_GPIO_PIN);
    LL_GPIO_ResetOutputPin(TRIG_GPIO_PORT, TRIG_GPIO_PIN);
#endif

    return 0;
}


int trigger_init_IO()
{
#if defined(TRIG_GPIO_PORT)
    LL_GPIO_Init(TRIG_GPIO_PORT, &gpio_trigger);
#endif
    return 0;
}


int trigger_tick_handler()
{
    int i;

    for(i = 0; i < STRATEGY_MAX; i++) {
        if(strategies[i].delay_cnt) {
            strategies[i].delay_cnt--;

            if(strategies[i].delay_cnt == 0) {
                trig_IO();
            }
        }
    }

    return 0;
}


int get_trigger_params(uint8_t* depth, uint8_t* stg_max)
{

    *depth = TRIGGER_DEPTH;
    *stg_max = STRATEGY_MAX;

    return 0;
}


int trigger_set_strategy(uint8_t index, const trigger_strategy_t* strategy)
{
    local_memcpy(&strategies[index], strategy, sizeof(trigger_strategy_t));

    return 0;
}


int trigger_get_strategy(uint8_t index, trigger_strategy_t* strategy)
{
    local_memcpy(strategy, &strategies[index], sizeof(trigger_strategy_t));

    return 0;
}


static inline int cmp(trigger_strategy_t* s)
{
    int i, j;

    /* If the strategy is empty, it means it is not enabled. */
    if(s->size == 0) {
        return 1;
    }

    /* Traverse strategy list in inverse order. */
    for(j = 0; j < s->size; j++) {
        i = ((trigger_hist_pt - j - 1 + TRIGGER_DEPTH) % TRIGGER_DEPTH);

        /* If not enough history. */
        if(trigger_hist[i] == 0) {
            return 1;
        }

        if(s->list[s->size - j - 1] == 0) {
            continue;
        }

        /* If it is not a match, exit. */
        if((s->list[s->size - j - 1] & trigger_hist[i]) == 0) {
            return 1;
        }

    }

    return 0;
}


int trig(uint32_t trign)
{
    int i;

    /* Add trigger ID(trign) in the buffer. */
    trigger_hist[trigger_hist_pt++] = trign;
    trigger_hist_pt %= TRIGGER_DEPTH;

    /* Look for a strategy that should be trigged.
     * If yes, set the delay. */
    for(i = 0; i < STRATEGY_MAX; i++) {
        if(cmp(&strategies[i]) == 0) {
            if(strategies[i].delay) {
                strategies[i].delay_cnt = strategies[i].delay;

            } else {
                trig_IO();
            }
        }
    }

    return 0;
}
