#ifdef INPUT_SIMULATION

#include <assert.h>
#include <string.h> // memcpy
#include "mcp3914.h"
#include <stm32_platform.h>
#include <clk_enable.h>
#include "buffer.h"

uint8_t channel_no[2] = { 0, 1 };

static volatile uint32_t active_mask;
static uint32_t registers[MCP3914_REG_NUM];
static TIM_TypeDef *t = TIM2;
static uint32_t start_sample = 0;
static uint32_t dropped_buffers = 0;
static uint64_t dropped_samples = 0;

extern void enable_interrupt(unsigned irq);
extern void new_sample(int value);

void stop_measurement(void)
{
	active_mask = 0;
	prod_discard();
}

void timer_interrupt(void)
{
	if (active_mask)
	{
		// simulate data collection
		struct Buffer *b = &buffer[buf_tail_head];
		unsigned i;
		for (i = 0; i != MCP3914_NUM_CHANNELS; ++i)
		{
			unsigned x = b->hdr.start_sample * MCP3914_NUM_CHANNELS + buf_tail_ptr / 3 + i;
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
		buf_tail_ptr += MCP3914_NUM_CHANNELS * 3;
		++new_buffer->hdr.num_samples;
		if (buf_tail_ptr > BUF_DATA_SZ - MCP3914_MAX_SAMPLE_SIZE)
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
	}
	t->SR = ~TIM_SR_UIF;
}

void input_setup(void)
{
	enable_interrupt(TIM2_IRQn);
	clk_enable(t);
	t->PSC = 72 - 1;
	t->ARR = 1000 * 4096 / 2500 - 1;
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
		start_sample = 0;
		buf_tail_tail = next_buffer(buf_tail_tail);
		struct Buffer *b = &buffer[buf_tail_head];
		b->hdr.start_sample = start_sample;
		b->hdr.num_samples = 0;
		buf_tail_ptr = 0;
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
