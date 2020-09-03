#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "config.h"
#include "helpers.h"
#include "triggers.h"
/* For time measurement */
#include "smartcard_iso7816_platform.h"
/* For systick handlers */
#include "stm32f4xx_it.h"

static volatile trigger_strategy_t strategies[STRATEGY_MAX];


/* Internal board trigger */
#if defined(TRIG_GPIO_PORT)
static LL_GPIO_InitTypeDef gpio_trigger = {
    .Pin        = TRIG_GPIO_PIN,
    .Mode       = LL_GPIO_MODE_OUTPUT,
    .Pull       = LL_GPIO_PULL_NO,
    .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
};
#endif

/* ChipWhisperer related trigger */
#if defined(TRIG_CW_GPIO_PORT)
static LL_GPIO_InitTypeDef gpio_trigger_cw = {
    .Pin        = TRIG_CW_GPIO_PIN,
    .Mode       = LL_GPIO_MODE_OUTPUT,
    .Pull       = LL_GPIO_PULL_NO,
    .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
};
#endif

/* NOTE: optimize the loop with -O0 to have a consistent
 * behaviour.
 */
#if __GNUC__
# pragma GCC push_options
# pragma GCC optimize("O0")
#endif
#if __clang__
# pragma clang optimize off
#endif
volatile unsigned int volatile_cnt = 0;
static void trig_sleep(void)
{
    for(volatile_cnt = 0; volatile_cnt < 16; volatile_cnt++);
    return;
}
#if __clang__
# pragma clang optimize on
#endif
#if __GNUG__
# pragma GCC pop_options
#endif

inline int trig_IO()
{
    /* Internal board trigger */
#if defined(TRIG_GPIO_PORT)
    LL_GPIO_SetOutputPin(TRIG_GPIO_PORT, TRIG_GPIO_PIN);
    trig_sleep(); /* Small wait for minimal size trigger */
    LL_GPIO_ResetOutputPin(TRIG_GPIO_PORT, TRIG_GPIO_PIN);
#endif

    /* ChipWhisperer related trigger */
#if defined(TRIG_CW_GPIO_PORT)
    LL_GPIO_SetOutputPin(TRIG_CW_GPIO_PORT, TRIG_CW_GPIO_PIN);
    trig_sleep(); /* Small wait for minimal size trigger */
    LL_GPIO_ResetOutputPin(TRIG_CW_GPIO_PORT, TRIG_CW_GPIO_PIN);
#endif

    return 0;
}

/* Handler to handle delayed triggers, this handler runs asynchronously */
void delayed_triggers_handler(void)
{
    unsigned int j, k;
    uint32_t microssec_time = platform_get_microseconds_ticks();

    for(j = 0; j < STRATEGY_MAX; j++){
        for(k = 0; k < strategies[j].size; k++){
            if(strategies[j].apply_delay[k] != 0){
                  if((microssec_time / 1000) >= strategies[j].apply_delay[k]){
                      strategies[j].list_trigged[k] = strategies[j].list[k];
                      strategies[j].cnt_trigged[k]++;
                      strategies[j].event_time[k] = microssec_time;
                      strategies[j].apply_delay[k] = 0;
                  }
            }
        }
    }

    return;
}


int trigger_init_IO()
{
    /* Internal board trigger */
#if defined(TRIG_GPIO_PORT)
    TRIG_GPIO_ENABLE_CLOCK();
    LL_GPIO_Init(TRIG_GPIO_PORT, &gpio_trigger);
    LL_GPIO_ResetOutputPin(TRIG_GPIO_PORT, TRIG_GPIO_PIN);
#endif

    /* ChipWhisperer related trigger */
#if defined(TRIG_CW_GPIO_PORT)
    TRIG_CW_GPIO_ENABLE_CLOCK();
    LL_GPIO_Init(TRIG_CW_GPIO_PORT, &gpio_trigger_cw);
    LL_GPIO_ResetOutputPin(TRIG_CW_GPIO_PORT, TRIG_CW_GPIO_PIN);
#endif
    /* Regiter our callback */
    if(register_systick_user_callback(delayed_triggers_handler)){
        return 1;
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
    local_memcpy((trigger_strategy_t*)&strategies[index], strategy, sizeof(trigger_strategy_t));
    /* Zeroize write zone */
    local_memset(((trigger_strategy_t*)&strategies[index])->list_trigged, 0, sizeof(strategies[index].list_trigged)); 
    local_memset(((trigger_strategy_t*)&strategies[index])->cnt_trigged, 0, sizeof(strategies[index].cnt_trigged)); 
    local_memset(((trigger_strategy_t*)&strategies[index])->event_time, 0, sizeof(strategies[index].event_time)); 
    local_memset(((trigger_strategy_t*)&strategies[index])->apply_delay, 0, sizeof(strategies[index].apply_delay)); 

    return 0;
}


int trigger_get_strategy(uint8_t index, trigger_strategy_t* strategy)
{
    local_memcpy(strategy, (trigger_strategy_t*)&strategies[index], sizeof(trigger_strategy_t));

    return 0;
}

static inline int cmp(trigger_strategy_t* s, uint32_t trign, uint32_t delay)
{
    int i;

    /* If the strategy is empty, it means it is not enabled. */
    if(s->size == 0) {
        return 1;
    }

    /* Traverse strategy list. */
    for(i = 0; i < s->size; i++) {
        if((s->list[i] & trign) != 0) {
            /* If we are in single mode, no need to trig again if we have already trigged this specific event */
            if((s->single == 1) && ((s->list_trigged[i] & trign) != 0)){
                return 1;
            }
            if(delay == 0){
                /* We have found a matching trigger and we are above our mask */
                s->list_trigged[i] |= trign; /* Tell that we have been trigged */
                s->cnt_trigged[i]++; /* Increment triggers counters */
                s->event_time[i] = platform_get_microseconds_ticks(); /* Save the time where the last event happened */
                return 0;
            }
            else{
                /* the delay in milliseconds to apply for this specific trigger */
                s->apply_delay[i] = (platform_get_microseconds_ticks() / 1000) + delay;
                return 1;
            }
        }
    }

    return 1;
}

int trig(uint32_t trign)
{
    int i;

    /* Look for a strategy that should be trigged.
     * If yes, trig with possible delay */
    for(i = 0; i < STRATEGY_MAX; i++) {
        if(cmp((trigger_strategy_t*)&strategies[i], trign, strategies[i].delay) == 0) {
            /* Trig now, or possibly later */
            trig_IO();
        }
    }

    return 0;
}
