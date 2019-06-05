#ifndef STM32F4XX_TIMER_H
#define STM32F4XX_TIMER_H

#define TIM25_TIE (1)
#define TIMER25_CLOCK_DIV1 0
#define TIMER25_AUTO_RELOAD 1
#define TIMER25_CMS 0x0
#define TIMER25_DIR_UP 0
#define TIMER25_ONEPULSE_NO 0
#define TIMER25_URS_ONLY_ON_OVERFLOW 1
#define TIMER25_UPDATE_DISABLE_NO 0


#define TIMER25_PRESCALER_DIV1 0


void timer2_init_master(int ckd, char auto_reload, char cms, char dir, char opm,
		char urs, char udis);
void timer2_enable();
void timer2_disable();
void timer2_enable_interrupt();
void timer2_disable_interrupt();
void timer2_set_counter(uint32_t counter);
void timer2_set_prescaler(uint16_t psc);
void timer2_set_autoreload(uint32_t arr);
void timer2_set_ccr1(uint32_t ccr1);
void timer2_set_ccr2(uint32_t ccr2);
void timer2_set_ccr3(uint32_t ccr3);
void timer2_set_ccr4(uint32_t ccr4);


#endif // STM32F4XX_TIMER_H
