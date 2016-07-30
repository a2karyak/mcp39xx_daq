#ifndef BUFFER_H_
#define BUFFER_H_

#include <stdint.h>

struct BufferHdr
{
	uint32_t start_sample;
	uint16_t num_samples;
	uint16_t num_bytes;
} __attribute__((packed));

enum
{
	BUF_SZ = 1024,
	BUF_DATA_SZ = BUF_SZ - sizeof(struct BufferHdr),
#ifdef STM32F10X_MD
#ifdef _DEBUG
	NUM_BUF = 8
#else
	NUM_BUF = 16
#endif
#endif
};

struct Buffer
{
	struct BufferHdr hdr;
	uint8_t data[BUF_DATA_SZ];
} __attribute__((packed));

extern struct Buffer buffer[NUM_BUF];

extern volatile uint8_t buf_head_head, buf_tail_head;
extern volatile uint8_t buf_head_tail, buf_tail_tail;
extern uint32_t buf_head_len;
extern uint32_t buf_tail_ptr;

void prod_discard(void);

static __inline uint32_t next_buffer(uint32_t b)
{
	return (b + 1) % NUM_BUF;
}

static __inline int cons_buffer_open(void)
{
	return buf_head_head != buf_head_tail;
}

static __inline int cons_buffer_available(void)
{
	return buf_head_head != buf_tail_head;
}

static __inline struct Buffer *cons_get_buffer(void)
{
	struct Buffer *b = &buffer[buf_head_head];
	if (buf_head_head == buf_head_tail)
	{
		buf_head_tail = next_buffer(buf_head_tail);
		buf_head_len = 0;
	}
	return b;
}

static __inline void cons_close_buffer(void)
{
	buf_head_head = buf_head_tail;
}

#endif
