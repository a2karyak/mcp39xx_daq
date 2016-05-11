#ifndef MCP3914_H_
#define MCP3914_H_

#include "gpio.h"
#include "dma.h"

enum
{
	MCP3914_NUM_CHANNELS = 8
};

enum MCP3914_REG
{
	MCP3914_REG_CHANNEL_BASE = 0,
	MCP3914_REG_MOD = 8,
	MCP3914_REG_PHASE0 = 9,
	MCP3914_REG_PHASE1 = 10,
	MCP3914_REG_GAIN = 11,
	MCP3914_REG_STATUSCOM = 12,
	MCP3914_REG_CONFIG0 = 13,
	MCP3914_REG_CONFIG1 = 14,
	MCP3914_REG_OFFCAL_CH0 = 15, // every channel takes 2 register
	MCP3914_REG_GAINCAL_CH0 = 16, // every channel takes 2 register
	MCP3914_REG_SECURITY = 31
};

enum /* : uint32_t */ // TODO complete TODO separate file
{
	MCP3914_CONFIG0_PRE_MASK = 3 << 16,
	MCP3914_CONFIG0_PRE1 = 0 << 16,
	MCP3914_CONFIG0_PRE2 = 1 << 16,
	MCP3914_CONFIG0_PRE4 = 2 << 16,
	MCP3914_CONFIG0_PRE8 = 3 << 16,
	MCP3914_CONFIG1_VREF_MASK = 1 << 7,
	MCP3914_CONFIG1_VREF_EXT = 1 << 7,
	MCP3914_CONFIG1_VREF_INT = 0 << 7,
	MCP3914_CONFIG1_CLK_MASK = 1 << 6,
	MCP3914_CONFIG1_CLK_EXT = 1 << 6,
	MCP3914_CONFIG1_CLK_INT = 0 << 6
};

enum /* : uint8_t */
{
	MCP3914_CMD_ADDRESS = 1,
	MCP3914_CMD_READ_MASK = 1,
	MCP3914_CMD_WRITE_MASK = 0
};

enum MCP3914_STREAM
{
	MCP3914_STREAM_SINGLE = 0,
	MCP3914_STREAM_GROUP,
	MCP3914_STREAM_ALL

};

struct MCP3914_PORT_CFG
{
	SPI_TypeDef *spi;
	struct GPIO_PORT_PIN clk_pin;
	struct GPIO_PORT_PIN mosi_pin;
	struct GPIO_PORT_PIN miso_pin;
	struct GPIO_PORT_PIN cs_pin;
	struct GPIO_PORT_PIN reset_pin;
	struct GPIO_PORT_PIN dr_pin;
	const struct DMA_TARGET spi_rx_dma;
	const struct DMA_TARGET spi_tx_dma;
};

struct MCP3914_PORT
{
	const struct MCP3914_PORT_CFG *cfg;
	uint8_t channel_width;
	uint8_t repeat;
};

void mcp3914_port_configure(const struct MCP3914_PORT_CFG *port_cfg, enum GPIO_OUT_SPEED pin_speed, uint32_t spi_prescaler);
void mcp3914_port_init(const struct MCP3914_PORT_CFG *port_cfg, struct MCP3914_PORT *port);

static __inline
void mcp3914_select(const struct MCP3914_PORT_CFG *port)
{
	port->cs_pin.port->BSRR = 1 << (16 + port->cs_pin.pin);
}

static __inline
void mcp3914_deselect(const struct MCP3914_PORT_CFG *port)
{
	// TODO WAIT ????
	port->cs_pin.port->BSRR = 1 << port->cs_pin.pin;
}

void mcp3914_write_reg(const struct MCP3914_PORT_CFG *port, enum MCP3914_REG reg, uint32_t val);
uint32_t mcp3914_read_reg(const struct MCP3914_PORT *port, enum MCP3914_REG reg);
void mcp3914_read_channels_start(const struct MCP3914_PORT *port, uint8_t start_channel, enum MCP3914_STREAM stream);

#endif
