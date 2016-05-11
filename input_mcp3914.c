#include <assert.h>
#include "mcp3914.h"
#include <spi.h>

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
		{ DMA1_Channel2 }, // RX DMA
		{ DMA1_Channel3 }, // TX DMA
	},
	{
		SPI2,
		{ GPIOB, 12 }, // RESET
		{ GPIOB, 13 }, // SCK
		{ GPIOB, 15 }, // MOSI
		{ GPIOB, 14 }, // MISO
		{ GPIOA, 8 }, // CS
		{ GPIOA, 9 }, // DR
		{ DMA1_Channel4 }, // RX DMA
		{ DMA1_Channel5 }, // TX DMA
	},
};

static struct MCP3914_PORT mcp_dev[NUM_DEV];

static void setup_port(const struct MCP3914_PORT_CFG *cfg, struct MCP3914_PORT *state)
{
	mcp3914_port_configure(cfg, GPIO_OUT_SPEED_HIGH, SPI_CR1_BR_DIV_64);
	mcp3914_port_init(cfg, state);
}

void input_setup(void)
{
	unsigned i;
	for (i = 0; i != NUM_DEV; ++i)
	{
		const struct MCP3914_PORT_CFG *cfg = &mcp_cfg[i];
		struct MCP3914_PORT *dev = &mcp_dev[i];

		setup_port(cfg, dev);

		uint32_t security = mcp3914_read_reg(dev, MCP3914_REG_SECURITY);
		uint32_t config1 = mcp3914_read_reg(dev, MCP3914_REG_CONFIG1);
		config1 = (config1 & ~MCP3914_CONFIG1_CLK_MASK) | MCP3914_CONFIG1_CLK_INT;
		mcp3914_write_reg(cfg, MCP3914_REG_CONFIG1, config1);
	}
}
