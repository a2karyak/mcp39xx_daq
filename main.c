#include <stdint.h>

#include <stm32_platform.h>
#include <cm3_vector.h>
#include <stm32_vector.h>
#include <clock.h>
#include <clk_enable.h>
#include <gpio.h>
#include <systick.h>
#include <timer.h>
#include <board/board.h>

static const struct RCC_CLK_CONFIG clk_cfg =
{
	.clk_src = RCC_SYSCLK_SRC_CRYSTAL,
#ifdef STM32F10X_MD // 72 MHz
	.pll_mul = 9,
	.pll_hse_div = RCC_PLL_HSE_DIV_1,
#endif
	.ahb_pre = RCC_AHB_PRE_1,
	.apb1_pre = RCC_APB_PRE_2,
	.apb2_pre = RCC_APB_PRE_1
};

#define CPU_FREQ (SYSCLK_FREQ(&BOARD_CLOCK_CONFIG, &clk_cfg))

#include "delay.h"

extern void exti3_irq(void);
extern void exti5_9_irq(void);
extern void input_setup(void);
extern int start_measurement(void);

__attribute__ ((section(".isr_vector_core")))
const struct CoreInterruptVector g_pfnVectors_Core =
{
	.estack = &_estack,
	.Reset_Handler = Reset_Handler,
};

__attribute__ ((section(".isr_vector_ext")))
const struct ExtInterruptVector g_pfnVectors_Ext =
{
	.EXTI3_IRQHandler = exti3_irq,
	.EXTI9_5_IRQHandler = exti5_9_irq,
};

void new_sample(int value)
{
	LED_PORT->ODR = value << LED_PIN1;
}

int main()
{
	rcc_config(&BOARD_CLOCK_CONFIG, &clk_cfg);

	SysTick->LOAD = SysTick_LOAD_RELOAD_Msk;
	SysTick->VAL = 0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

	clk_enable(LED_PORT);
	gpio_configure_out(LED_PORT, LED_PIN1, GPIO_OUT_PP, GPIO_OUT_SPEED_2MHz);

	input_setup();
	start_measurement();

	uint32_t clock = SysTick->VAL;
	uint32_t led_time = 0;
	while (1)
	{
//		uint32_t elapsed = systick_time_interval(&clock);
//		if (periodic_timer(&led_time, CPU_FREQ / 2, elapsed))
//			LED_PORT->ODR ^= 1 << LED_PIN1;
	}
}
