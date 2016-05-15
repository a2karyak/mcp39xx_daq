#ifdef INPUT_MCP3914

#include <assert.h>
#include <spi.h>
#include <clk_enable.h>
#include "mcp3914.h"
#include "buffer.h"

enum { NUM_DEV = 2 };

static const struct MCP3914_PORT_CFG mcp_cfg[NUM_DEV] =
{
	{
		SPI1,
		{ GPIOB, 0 }, // RESET
		{ GPIOA, 5 }, // SCK
		{ GPIOA, 7 }, // MOSI
		{ GPIOA, 6 }, // MISO
		{ GPIOA, 4 }, // CS
		{ GPIOA, 3 }, // DR
		{ DMA1_Channel2, 1 * 4 }, // RX DMA
		{ DMA1_Channel3, 2 * 4 }, // TX DMA
	},
	{
		SPI2,
		{ GPIOB, 12 }, // RESET
		{ GPIOB, 13 }, // SCK
		{ GPIOB, 15 }, // MOSI
		{ GPIOB, 14 }, // MISO
		{ GPIOA, 8 }, // CS
		{ GPIOA, 9 }, // DR
		{ DMA1_Channel4, 3 * 4 }, // RX DMA
		{ DMA1_Channel5, 4 * 4 }, // TX DMA
	},
};

struct DAQ_CHANNEL
{
	const struct MCP3914_PORT_CFG *cfg;
	unsigned channel_width;
	unsigned repeat;
	unsigned data_size;
	unsigned data_offset;
	volatile int8_t dma_setup;
};

uint8_t channel_no[2] = { 0, 1 };

static struct DAQ_CHANNEL daq_channel[NUM_DEV];
static volatile uint32_t active_mask;
static volatile uint32_t complete_mask;
static uint32_t full_size;
static volatile int stopping;
static uint8_t *new_record;
static volatile uint8_t mcp_tx_buf = 0xff;
static uint32_t start_sample = 0;
static uint32_t dropped_buffers = 0;
static uint64_t dropped_samples = 0;

static void update_statuscom(struct DAQ_CHANNEL *channel, uint32_t val)
{
	mcp_3914_parse_statuscom(val, &channel->channel_width, &channel->repeat);
	channel->data_size = channel->channel_width * channel->repeat;
}

static void configure_dma_common(DMA_Channel_TypeDef *channel, uint32_t ccr, volatile uint16_t *paddr)
{
	clk_enable(DMA_CHANNEL_TO_DMA(channel));

	channel->CCR = DMA_CCR1_PL_VERY_HIGH | DMA_CCR1_PSIZE_BYTE | DMA_CCR1_MSIZE_BYTE | ccr;
	channel->CPAR = (uint32_t) paddr;
}

// TODO: move this to EXTI
static const uint8_t EXTI_IRQ[16] =
{
	EXTI0_IRQn,
	EXTI1_IRQn,
	EXTI2_IRQn,
	EXTI3_IRQn,
	EXTI4_IRQn,
	EXTI9_5_IRQn,
	EXTI9_5_IRQn,
	EXTI9_5_IRQn,
	EXTI9_5_IRQn,
	EXTI9_5_IRQn,
	EXTI15_10_IRQn,
	EXTI15_10_IRQn,
	EXTI15_10_IRQn,
	EXTI15_10_IRQn,
	EXTI15_10_IRQn,
	EXTI15_10_IRQn,
};

//static unsigned gpio_to_source(const GPIO_TypeDef *gpio)
//{
//	uint32_t base = (uint32_t) gpio;
//#if defined STM32F10X_MD
//	assert(base >= GPIOA_BASE && base <= GPIOG_BASE);
//	if (base >= GPIOA_BASE && base <= GPIOG_BASE)
//	{
//		return 0 + (base - GPIOA_BASE) / (GPIOB_BASE - GPIOA_BASE);
//	}
//#else
//#error Unsupported MCU
//#endif
//	return -1;
//}

void exti_configure_line(unsigned line, const GPIO_TypeDef *gpio)
{
//	unsigned line_reg = line / 4;
//	unsigned line_shift = 4 * (line % 4);
//	unsigned source = gpio_to_source(gpio);
//	volatile uint32_t *exticr = &SYSCFG->EXTICR[line_reg];
//	*exticr = (*exticr & ~0xf << line_shift) | source << line_shift;
}

static void setup_channel(struct DAQ_CHANNEL *channel, const struct MCP3914_PORT_CFG *cfg)
{
	channel->cfg = cfg;

	mcp3914_port_configure(cfg, GPIO_OUT_SPEED_HIGH, SPI_CR1_BR_DIV_4); // TODO: frequency
	configure_dma_common(cfg->spi_rx_dma.dma_channel, DMA_CCR1_MINC, &cfg->spi->DR);
	configure_dma_common(cfg->spi_tx_dma.dma_channel, DMA_CCR1_DIR, &cfg->spi->DR);
	cfg->spi_tx_dma.dma_channel->CMAR = (uint32_t) &mcp_tx_buf;

	exti_configure_line(cfg->dr_pin.pin, cfg->dr_pin.port);
	EXTI->FTSR |= 1 << cfg->dr_pin.pin; // falling edge interrupt

	uint8_t pri = (0x700 - (SCB->AIRCR & 0x700)) >> 8;
	uint8_t pre = 4 - pri;
	uint8_t sub = 0xf >> pri;

	pri = 0 << pre; // TODO: preemption priority
	pri |= 0 & sub; // TODO: subpriority
	pri = pri << 4;

	unsigned irq = EXTI_IRQ[cfg->dr_pin.pin];

	NVIC->IP[irq] = pri;
	NVIC->ISER[irq >> 5] = (uint32_t) 1 << (irq & 0x1f);
	NVIC->ICPR[irq >> 5] = (1 << (irq & 0x1f)); /* Clear pending interrupt */ // TODO: move
}

void stop_measurement(void)
{
	if (active_mask != 0)
		stopping = 1;
}

/*
 * Reading data registers is allowed after tODR (25 ns). This is one period
 * of 40 MHz clock. Therefore, if the MCU clock is lower than 80 MHz, tODR
 * is no more than 2 MCU clock cycles, which is much less than the interrupt
 * latency.
 */

static void ext_interrupt(struct DAQ_CHANNEL *channel)
{
	const struct MCP3914_PORT_CFG *cfg = channel->cfg;

	uint32_t pr = EXTI->PR;
	uint32_t line_mask = 1 << cfg->dr_pin.pin;
	if ((pr & line_mask) == 0)
	{
		goto fin;
	}
	// For unknown reason, some interrupt keep pending after exiting ISR, causing another ISR immediately.
	// The loop below does not completely fixes the problem but seems to reduce the occurrence of skewed interrupts.
	// UPD: The problem was caused by open drain output of the DR signal, which dramatically increased the signal rise time,
	// generating multiple spurious interrupts.
	EXTI->PR = line_mask;
	if (channel->dma_setup)
	{
		uint32_t isr = DMA1->ISR;
		unsigned tx_flags = isr >> cfg->spi_tx_dma.dma_shift; // dma_get_flags(cfg->spi_tx_dma.dma_channel);
		unsigned rx_flags = isr >> cfg->spi_rx_dma.dma_shift; // dma_get_flags(cfg->spi_rx_dma.dma_channel);
		assert(!(tx_flags & DMA_STATUS_TEIF));
		assert(!(rx_flags & DMA_STATUS_TEIF));
		if (!(rx_flags & DMA_STATUS_TCIF)
				|| !(tx_flags & DMA_STATUS_TCIF)) // TODO what flags? Check other?
		{
			goto fin;
		}
		// TODO: other MCU will disable the channel
//		assert(!(cfg->spi_rx_dma.dma_channel->CCR & DMA_CCR1_EN));
//		assert(!(cfg->spi_tx_dma.dma_channel->CCR & DMA_CCR1_EN));
		cfg->spi_tx_dma.dma_channel->CCR &= ~DMA_CCR1_EN;
		cfg->spi_rx_dma.dma_channel->CCR &= ~DMA_CCR1_EN;
		// disable DMA request to avoid FIFO errors
		cfg->spi->CR2 &= ~SPI_CR2_TXDMAEN;
//		dma_clear_flags(cfg->spi_tx_dma.dma_channel, DMA_STATUS_ALL);
//		dma_clear_flags(cfg->spi_rx_dma.dma_channel, DMA_STATUS_ALL);
		DMA1->IFCR = DMA_STATUS_ALL << cfg->spi_tx_dma.dma_shift | DMA_STATUS_ALL << cfg->spi_rx_dma.dma_shift;
	}
	if (stopping)
	{
		if (active_mask & line_mask)
		{
			// TODO keep enabled all the time? Will it interfere with single SPI operations?
			cfg->spi->CR2 &= ~SPI_CR2_RXDMAEN;
			mcp3914_stream_end(cfg);
			channel->dma_setup = 0;
			active_mask &= ~line_mask;
			if (active_mask == 0)
			{
				stopping = 0;
				prod_discard();
			}
			EXTI->IMR &= ~line_mask;
		}
	}
	else
	{
		if (complete_mask & line_mask)
		{
//			goto fin; TODO?!
		}
		complete_mask |= line_mask;
		channel->dma_setup = 1;
		cfg->spi_tx_dma.dma_channel->CNDTR = channel->data_size;
		cfg->spi_rx_dma.dma_channel->CNDTR = channel->data_size;
		cfg->spi_tx_dma.dma_channel->CCR |= DMA_CCR1_EN;
		cfg->spi_rx_dma.dma_channel->CMAR = (uint32_t) new_record + channel->data_offset;
		cfg->spi_rx_dma.dma_channel->CCR |= DMA_CCR1_EN;
		cfg->spi->CR2 |= SPI_CR2_TXDMAEN;
		if (complete_mask == active_mask)
		{
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
			buf_tail_ptr += full_size;
			++new_buffer->hdr.num_samples;
			if (buf_tail_ptr > BUF_DATA_SZ - full_size)
			{
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
			}
			new_record = new_buffer->data + buf_tail_ptr;
			complete_mask = 0;
		}
	}
	EXTI->PR = line_mask; // this causes the thing to get stuck!
fin:;
}

void exti3_irq(void)
{
	ext_interrupt(&daq_channel[0]);
}

void exti5_9_irq(void)
{
	ext_interrupt(&daq_channel[1]);
}

void input_setup(void)
{
	unsigned i;
	for (i = 0; i != NUM_DEV; ++i)
	{
		const struct MCP3914_PORT_CFG *cfg = &mcp_cfg[i];
		struct DAQ_CHANNEL *channel = &daq_channel[i];

		setup_channel(channel, cfg);

		uint32_t security = mcp3914_read_reg(channel->cfg, MCP3914_REG_SECURITY);

		if (security == 0xa50000)
		{
			uint32_t config1 = mcp3914_read_reg(channel->cfg, MCP3914_REG_CONFIG1);
			config1 = (config1 & ~MCP3914_CONFIG1_CLK_MASK) | MCP3914_CONFIG1_CLK_INT;
			mcp3914_write_reg(cfg, MCP3914_REG_CONFIG1, config1);

			uint32_t statuscom = mcp3914_read_reg(channel->cfg, MCP3914_REG_STATUSCOM);
			update_statuscom(channel, statuscom);

			uint32_t config0 = mcp3914_read_reg(channel->cfg, MCP3914_REG_CONFIG0);
			config0 = (config0 & ~MCP3914_CONFIG0_PRE_MASK) | MCP3914_CONFIG0_PRE1;
			config0 = (config0 & ~MCP3914_CONFIG0_OSC_MASK) | MCP3914_CONFIG0_OSC_64;
			mcp3914_write_reg(cfg, MCP3914_REG_CONFIG0, config0);
		}
	}
}

int start_measurement(void)
{
	if (active_mask == 0)
	{
		uint32_t enabled_mask = 0;
		uint32_t sz = 0;

		unsigned i;
		for (i = 0; i != NUM_DEV; ++i)
		{
			const struct MCP3914_PORT_CFG *cfg = &mcp_cfg[i];
			struct DAQ_CHANNEL *channel = &daq_channel[i];
			uint32_t security = mcp3914_read_reg(channel->cfg, MCP3914_REG_SECURITY);
			if (security == 0xa50000)
			{
				if (channel->data_size != 0)
				{
					// TODO: DMA by words
					channel->data_offset = sz;
					mcp3914_stream_start(cfg, MCP3914_REG_CHANNEL_BASE + channel_no[0]);
					// TX DMA request is not enabled here to avoid FIFO error (DMA not ready)
					cfg->spi->CR2 |= SPI_CR2_RXDMAEN;
					channel->dma_setup = 0;
					enabled_mask |= 1 << cfg->dr_pin.pin;
					sz += channel->data_size;
				}
			}
		}
		if (enabled_mask != 0)
		{
			active_mask = enabled_mask;
			full_size = sz;
			complete_mask = 0;
			stopping = 0;
			start_sample = 0;
			buf_tail_tail = next_buffer(buf_tail_tail);
			struct Buffer *b = &buffer[buf_tail_head];
			b->hdr.start_sample = start_sample;
			b->hdr.num_samples = 0;
			buf_tail_ptr = 0;
			new_record = b->data;
			EXTI->IMR |= enabled_mask;
			return 0;
		}
	}
	return 1;
}

void input_write_register(unsigned reg, uint32_t val)
{
	unsigned port = reg / MCP3914_REG_NUM;
	unsigned reg_number = reg % MCP3914_REG_NUM;
	if (reg_number >= MCP3914_NUM_CHANNELS)
	{
		mcp3914_write_reg(&mcp_cfg[port], (enum MCP3914_REG) reg_number, val);
		if (reg == MCP3914_REG_STATUSCOM)
			update_statuscom(&daq_channel[port], val);
	}
}

uint32_t input_read_register(unsigned reg)
{
	unsigned port = reg / MCP3914_REG_NUM;
	unsigned reg_number = reg % MCP3914_REG_NUM;
	if (reg_number < MCP3914_NUM_CHANNELS)
		return mcp3914_read_channel(&mcp_cfg[port], reg_number, daq_channel[port].channel_width);
	else
		return mcp3914_read_reg(&mcp_cfg[port], (enum MCP3914_REG) reg_number);
}

#endif
