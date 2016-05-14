#include "buffer.h"
#include <assert.h>

struct Buffer buffer[NUM_BUF];

volatile uint8_t buf_head_head = 0, buf_tail_head = 0;
volatile uint8_t buf_head_tail = 0, buf_tail_tail = 0;
uint32_t buf_head_len = 0;
uint32_t buf_tail_ptr;

void prod_discard(void)
{
	assert(buf_head_head == buf_head_tail);
	// should not overlap with producer activities
	buf_tail_head = buf_tail_tail = buf_head_head;
	buf_tail_ptr = 0;
}

