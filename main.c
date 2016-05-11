#include <stdint.h>

#include <stm32_platform.h>
#include <cm3_vector.h>
#include <clock.h>
#include <clk_enable.h>
#include <gpio.h>
#include <systick.h>
#include <timer.h>
#include <spi.h>
#include <board/board.h>
#include "mcp3914.h"

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

__attribute__ ((section(".isr_vector_core")))
const struct CoreInterruptVector g_pfnVectors_Core =
{
	&_estack,
	.Reset_Handler = Reset_Handler,
};

static const struct MCP3914_PORT_CFG mcp_port0_cfg =
{
		SPI1,
		{ GPIOA, 5 }, // SCK
		{ GPIOA, 7 }, // MISO
		{ GPIOA, 6 }, // MOSI
		{ GPIOA, 4 }, // CS
		{ GPIOB, 0 }, // RESET
		{ GPIOA, 3 }, // DR
		{ DMA1_Channel2 }, // RX DMA
		{ DMA1_Channel3 }, // TX DMA
};

static struct MCP3914_PORT mcp_port0;

int main()
{
	rcc_config(&BOARD_CLOCK_CONFIG, &clk_cfg);

	SysTick->LOAD = SysTick_LOAD_RELOAD_Msk;
	SysTick->VAL = 0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

	clk_enable(LED_PORT);
	gpio_configure_out(LED_PORT, LED_PIN1, GPIO_OUT_PP, GPIO_OUT_SPEED_2MHz);

	clk_enable(GPIOA);
	clk_enable(GPIOB);
	mcp3914_port_configure(&mcp_port0_cfg, GPIO_OUT_SPEED_HIGH, SPI_CR1_BR_DIV_64);
	mcp3914_port_init(&mcp_port0_cfg, &mcp_port0);

	uint32_t security = mcp3914_read_reg(&mcp_port0, MCP3914_REG_SECURITY);
	uint32_t config1 = mcp3914_read_reg(&mcp_port0, MCP3914_REG_CONFIG1);
	config1 = (config1 & ~MCP3914_CONFIG1_CLK_MASK) | MCP3914_CONFIG1_CLK_INT;
	mcp3914_write_reg(&mcp_port0_cfg, MCP3914_REG_CONFIG1, config1);

	uint32_t clock = SysTick->VAL;
	uint32_t led_time = 0;
	while (1)
	{
		uint32_t elapsed = systick_time_interval(&clock);
		if (periodic_timer(&led_time, CPU_FREQ / 2, elapsed))
			LED_PORT->ODR ^= 1 << LED_PIN1;
	}
}
