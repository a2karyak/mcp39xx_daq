#include <assert.h>
#include "mcp3914.h"
#include <spi.h>
#include <clk_enable.h>

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
		{ 1 * 4, DMA1_Channel2 }, // RX DMA
		{ 2 * 4, DMA1_Channel3 }, // TX DMA
	},
	{
		SPI2,
		{ GPIOB, 12 }, // RESET
		{ GPIOB, 13 }, // SCK
		{ GPIOB, 15 }, // MOSI
		{ GPIOB, 14 }, // MISO
		{ GPIOA, 8 }, // CS
		{ GPIOA, 9 }, // DR
		{ 3 * 4, DMA1_Channel4 }, // RX DMA
		{ 4 * 4, DMA1_Channel5 }, // TX DMA
	},
};

struct DAQ_CHANNEL
{
	const struct MCP3914_PORT_CFG *cfg;
	uint32_t data_size;
	uint32_t data_offset;
	volatile uint8_t dma_setup;
	volatile uint8_t sample[MCP3914_MAX_SAMPLE_SIZE];
};

static struct DAQ_CHANNEL daq_channel[NUM_DEV];
static volatile uint32_t active_mask;
static volatile uint32_t complete_mask;
static uint32_t full_size;
static volatile int stopping;
static volatile uint8_t mcp_tx_buf = 0xff;

extern void new_sample(int value);

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
//				TODO: discard buffer
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
		if (complete_mask == 0)
			new_sample(1);
		complete_mask |= line_mask;
		channel->dma_setup = 1;
		cfg->spi_tx_dma.dma_channel->CNDTR = channel->data_size;
		cfg->spi_rx_dma.dma_channel->CNDTR = channel->data_size;
		cfg->spi_tx_dma.dma_channel->CCR |= DMA_CCR1_EN;
		cfg->spi_rx_dma.dma_channel->CMAR = (uint32_t) channel->sample;
		cfg->spi_rx_dma.dma_channel->CCR |= DMA_CCR1_EN;
		cfg->spi->CR2 |= SPI_CR2_TXDMAEN;
		if (complete_mask == active_mask)
		{
			new_sample(0);
			// TODO: new sample
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

			// TODO: move
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
				unsigned channel_width = 0;
				unsigned repeat = 0;
				uint32_t statuscom = mcp3914_read_reg(channel->cfg, MCP3914_REG_STATUSCOM);
				switch (statuscom & MCP3914_STATUSCOM_WIDTH_DATA_MASK)
				{
				case MCP3914_STATUSCOM_WIDTH_DATA_16:
					channel_width = 2;
					break;
				case MCP3914_STATUSCOM_WIDTH_DATA_24:
					channel_width = 3;
					break;
				case MCP3914_STATUSCOM_WIDTH_DATA_32S:
				case MCP3914_STATUSCOM_WIDTH_DATA_32Z:
					channel_width = 4;
					break;
				}
				switch (statuscom & MCP3914_STATUSCOM_READ_MASK)
				{
				case MCP3914_STATUSCOM_READ_ONE:
					repeat = 1;
					break;
				case MCP3914_STATUSCOM_READ_GROUP:
					repeat = 2; // TODO: incorrect for all registers
					break;
				case MCP3914_STATUSCOM_READ_TYPES:
					repeat = 8; // TODO: incorrect for all registers
					break;
				case MCP3914_STATUSCOM_READ_ALL:
					repeat = 32;
					break;
				}
				// TODO verify length of CHANNEL registers READ/NON_STREAM

				channel->data_size = channel_width * repeat;
				if (channel->data_size != 0)
				{
					int i;
					for (i = 0; i != MCP3914_MAX_SAMPLE_SIZE; ++i)
						channel->sample[i] = 0;

					// TODO: DMA by words
					channel->data_offset = sz;
					mcp3914_stream_start(cfg, MCP3914_REG_CHANNEL_BASE /*+ channel[0]*/);
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
			// TODO: setup first buffer
			EXTI->IMR |= enabled_mask;
			return 0;
		}
	}
	return 1;
}
