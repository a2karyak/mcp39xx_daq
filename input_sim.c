#ifdef INPUT_SIMULATION

#include <assert.h>
#include <string.h> // memcpy
#include <stm32_platform.h>
#include "mcp3914.h"
#include <clk_enable.h>
#include <systick.h>
#include "buffer.h"

enum { NUM_DEV = 2 };

uint8_t channel_no[2] = { 0, 1 };

static uint32_t registers[NUM_DEV][MCP3914_REG_NUM];
static TIM_TypeDef *t = TIM2;
static uint32_t start_sample = 0;
static uint32_t dropped_buffers = 0;
static uint64_t dropped_samples = 0;

unsigned long g_stat_irq_clk = 0;
unsigned g_stat_irq_cnt = 0;

extern void enable_interrupt(unsigned irq);
extern void new_sample(int value);

void stop_measurement(void)
{
	if (t->CR1 & TIM_CR1_CEN)
	{
		t->CR1 &= ~TIM_CR1_CEN;
		prod_discard();
	}
}

void timer_interrupt(void)
{
	if ((t->CR1 & TIM_CR1_CEN) == 0)
	{
		t->SR = ~TIM_SR_UIF;
		return;
	}

	uint32_t t0 = SysTick->VAL;
	g_stat_irq_cnt++;

	// simulate data collection
	struct Buffer *b = &buffer[buf_tail_head];
	unsigned i;
	for (i = 0; i != MCP3914_NUM_CHANNELS * NUM_DEV; ++i)
	{
		unsigned x = b->hdr.start_sample * MCP3914_NUM_CHANNELS * NUM_DEV + buf_tail_ptr / 3 + i;
		memcpy(b->data + buf_tail_ptr + i * 3, &x, 3);
	}
	// complete the buffer, if needed
	if (buf_tail_ptr == 0)
	{
		uint32_t next_completed_buffer = next_buffer(buf_tail_head);
		if (next_completed_buffer != buf_tail_tail)
		{
			struct Buffer *b = &buffer[buf_tail_head];
			b->hdr.num_bytes = BUF_DATA_SZ;
			buf_tail_head = next_completed_buffer;
		}
	}
	struct Buffer *new_buffer = &buffer[buf_tail_head];
	buf_tail_ptr += MCP3914_NUM_CHANNELS * NUM_DEV * 3;
	++new_buffer->hdr.num_samples;
	if (buf_tail_ptr > BUF_DATA_SZ - MCP3914_NUM_CHANNELS * NUM_DEV * 3)
	{
		new_sample(1);
		start_sample += new_buffer->hdr.num_samples;
		if (buf_tail_tail == buf_head_head)
		{
			dropped_samples += new_buffer->hdr.num_samples;
			dropped_buffers += 1;
		}
		else
		{
			new_buffer = &buffer[buf_tail_tail];
			buf_tail_tail = next_buffer(buf_tail_tail);
		}
		new_buffer->hdr.start_sample = start_sample;
		new_buffer->hdr.num_samples = 0;
		buf_tail_ptr = 0;
		new_sample(1);
	}
	t->SR = ~TIM_SR_UIF;
	g_stat_irq_clk += systick_time_interval(&t0);
}

void input_setup(void)
{
	unsigned i, j;
	for (j = 0; j != NUM_DEV; ++j)
		for (i = 0; i != MCP3914_REG_NUM; ++i)
			registers[j][i] = j << 8 + i;

	enable_interrupt(TIM2_IRQn);
	clk_enable(t);
	t->PSC = 9 - 1;
	t->DIER |= TIM_DIER_UIE;
}

int start_measurement(void)
{
	if ((t->CR1 & TIM_CR1_CEN) == 0)
	{
		start_sample = 0;
		buf_tail_tail = next_buffer(buf_tail_tail);
		struct Buffer *b = &buffer[buf_tail_head];
		b->hdr.start_sample = start_sample;
		b->hdr.num_samples = 0;
		buf_tail_ptr = 0;

		unsigned osc = (registers[0][MCP3914_REG_CONFIG0] >> 13) & 7;
		t->ARR = ((8000 << (osc + 5)) + 2500 / 2) / 2500 - 1;
		t->CR1 |= TIM_CR1_CEN;
		return 0;
	}
	return 1;
}

void input_write_register(unsigned reg, uint32_t val)
{
	unsigned port = reg / MCP3914_REG_NUM;
	unsigned reg_number = reg % MCP3914_REG_NUM;
	if (reg_number >= MCP3914_NUM_CHANNELS)
		registers[port][reg_number] = val;
}

uint32_t input_read_register(unsigned reg)
{
	unsigned port = reg / MCP3914_REG_NUM;
	unsigned reg_number = reg % MCP3914_REG_NUM;
	if (reg_number < MCP3914_NUM_CHANNELS)
		return 0x5a5a;
	else
		return registers[port][reg_number];
}

#endif
