#include <assert.h>
#include "mcp3914.h"
#include <clk_enable.h>

#ifdef INPUT_SIMULATION

static volatile uint32_t active_mask;

static uint32_t registers[MCP3914_REG_NUM];
uint8_t channel[2];
TIM_TypeDef *t = TIM2;

extern void enable_interrupt(unsigned irq);
extern void new_sample(int value);

void stop_measurement(void)
{
	active_mask = 0;
}

void timer_interrupt(void)
{
	static int i = 0;
	if (active_mask)
	{
		new_sample(i ^= 1);
	}
	t->SR = ~TIM_SR_UIF;
}

void input_setup(void)
{
	enable_interrupt(TIM2_IRQn);
	clk_enable(t);
	t->PSC = 720 - 1;
	t->ARR = 10000 - 1;
	t->DIER |= TIM_DIER_UIE;
	t->CR1 |= TIM_CR1_CEN;

	unsigned i;
	for (i = 0; i != MCP3914_REG_NUM; ++i)
		registers[i] = i;
}

int start_measurement(void)
{
	if (active_mask == 0)
	{
		active_mask = 1;
		return 0;
	}
	return 1;
}

void input_write_register(unsigned reg, uint32_t val)
{
	unsigned reg_number = reg % MCP3914_REG_NUM;
	if (reg_number >= MCP3914_NUM_CHANNELS)
		registers[reg_number] = val;
}

uint32_t input_read_register(unsigned reg)
{
	unsigned reg_number = reg % MCP3914_REG_NUM;
	if (reg_number < MCP3914_NUM_CHANNELS)
		return 0x5a5a;
	else
		return registers[reg_number];
}

#endif
