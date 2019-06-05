#include "debug.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_nvic.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_timer_regs.h"
#include "stm32f4xx_timer.h"

/*
*
*Only General Timer 2 & 5
*
*/

void timer2_init_master(int ckd, char auto_reload, char cms, char dir, char opm,
		char urs, char udis)
{
	/* Enable the SYSCFG and clock TIMER2 */
	set_reg_bits(r_CORTEX_M_RCC_APB2ENR, RCC_APB2ENR_SYSCFGEN);
	set_reg_bits(r_CORTEX_M_RCC_APB1ENR, RCC_APB1ENR_TIM2EN);
	/* Set the TIM2CR1 according to the parameters */
	write_reg_value(r_CORTEX_M_TIM2CR1,(ckd&3)<<8|((auto_reload&1)<<7)|
			((cms&7)<<5)|((dir&1)<<4)|((opm&1)<<3)|
			((urs&1)<<2)|((udis&1)<<1));

}

void timer2_enable(void)
{
	set_reg_bits(r_CORTEX_M_TIM2CR1,1);
}

void timer2_disable(void)
{
	clear_reg_bits(r_CORTEX_M_TIM2CR1,1);
}
void timer2_enable_interrupt(void)
{
	set_reg_bits(r_CORTEX_M_TIM2DIER,TIM25_TIE);
}
void timer2_disable_interrupt(void)
{
	clear_reg_bits(r_CORTEX_M_TIM2DIER,TIM25_TIE);
}

void timer2_set_counter(uint32_t counter)
{
  write_reg_value(r_CORTEX_M_TIM2CNT,counter);
}

void timer2_set_prescaler(uint16_t psc)
{
  write_reg_value(r_CORTEX_M_TIM2PSC,psc);
}

void timer2_set_autoreload(uint32_t arr)
{
  write_reg_value(r_CORTEX_M_TIM2ARR,arr);
}

void timer2_set_ccr1(uint32_t ccr1)
{
  write_reg_value(r_CORTEX_M_TIM2CCR1,ccr1);
}
void timer2_set_ccr2(uint32_t ccr2)
{
  write_reg_value(r_CORTEX_M_TIM2CCR2,ccr2);
}
void timer2_set_ccr3(uint32_t ccr3)
{
  write_reg_value(r_CORTEX_M_TIM2CCR3,ccr3);
}
void timer2_set_ccr4(uint32_t ccr4)
{
  write_reg_value(r_CORTEX_M_TIM2CCR4,ccr4);
}


