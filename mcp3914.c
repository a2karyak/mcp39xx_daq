#include <assert.h>
#include "mcp3914.h"

#include <spi.h>
#include <clk_enable.h>

extern void delay_us(const uint32_t usec);

void mcp3914_port_configure(const struct MCP3914_PORT_CFG *port, enum GPIO_OUT_SPEED pin_speed, uint32_t spi_prescaler)
{
	gpio_pin_write(port->reset_pin, 1);
	gpio_pin_write(port->cs_pin, 1);

	clk_enable(AFIO);
	clk_enable(port->reset_pin.port);
	clk_enable(port->sck_pin.port);
	clk_enable(port->mosi_pin.port);
	clk_enable(port->miso_pin.port);
	clk_enable(port->cs_pin.port);
	clk_enable(port->dr_pin.port);

	gpio_configure_out(port->reset_pin.port, port->reset_pin.pin, GPIO_OUT_PP, pin_speed);
	gpio_configure_af(port->sck_pin.port, port->sck_pin.pin, GPIO_OUT_PP, pin_speed);
	gpio_configure_af(port->mosi_pin.port, port->mosi_pin.pin, GPIO_OUT_PP, pin_speed);
	gpio_configure_in(port->miso_pin.port, port->miso_pin.pin);
	gpio_configure_out(port->cs_pin.port, port->cs_pin.pin, GPIO_OUT_PP, pin_speed);
	gpio_configure_in(port->dr_pin.port, port->dr_pin.pin);
	gpio_configure_in_pull(port->dr_pin.port, port->dr_pin.pin, GPIO_PULLUP); // single device configuration, requires DR_HIZ setting TODO NOPULL ????

	// reset MCP3914, may not be necessary
	gpio_pin_write(port->reset_pin, 0);
	delay_us(1000);
	gpio_pin_write(port->reset_pin, 1);
	delay_us(1000);

	// TODO: 16 bit SPI
	clk_enable(port->spi);
	port->spi->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | spi_prescaler;
	port->spi->CR1 |= SPI_CR1_SPE;

	clk_enable(DMA_CHANNEL_TO_DMA(port->spi_rx_dma.dma_channel));
	clk_enable(DMA_CHANNEL_TO_DMA(port->spi_tx_dma.dma_channel));
}

static uint8_t READ_CMD(uint8_t address, uint8_t reg)
{
	return (address << 6) | (reg << 1) | MCP3914_CMD_READ_MASK;
}

static uint8_t WRITE_CMD(uint8_t address, uint8_t reg)
{
	return (address << 6) | (reg << 1) | MCP3914_CMD_WRITE_MASK;
}

void mcp3914_write_reg(const struct MCP3914_PORT_CFG *cfg, enum MCP3914_REG reg, uint32_t val)
{
	mcp3914_select(cfg);
	spi_read_write(cfg->spi, WRITE_CMD(MCP3914_CMD_ADDRESS, reg));
	if (reg == MCP3914_REG_MOD)
		spi_read_write(cfg->spi, (val >> 24) & 0xff);
	spi_read_write(cfg->spi, (val >> 16) & 0xff);
	spi_read_write(cfg->spi, (val >> 8) & 0xff);
	spi_read_write(cfg->spi, val & 0xff);
	mcp3914_deselect(cfg);
}

static uint8_t mcp3914_reg_width(enum MCP3914_REG reg)
{
	if (reg >= MCP3914_REG_CHANNEL_BASE
			&& reg < MCP3914_REG_CHANNEL_BASE + MCP3914_NUM_CHANNELS)
		return 2; // minimal size
	else
	if (reg == MCP3914_REG_MOD)
		return 4;
	else
		return 3;
}

uint32_t mcp3914_read_reg(const struct MCP3914_PORT_CFG *cfg, enum MCP3914_REG reg)
{
	uint32_t val = 0;
	unsigned len = mcp3914_reg_width(reg);
	assert(len >= 2 && len <= 4);
	mcp3914_select(cfg);
	spi_read_write(cfg->spi, READ_CMD(MCP3914_CMD_ADDRESS, reg));
	switch (len)
	{
	case 4:
		val |= spi_read_write(cfg->spi, 0xff) << 24;
	case 3:
		val |= spi_read_write(cfg->spi, 0xff) << 16;
	case 2:
		val |= spi_read_write(cfg->spi, 0xff) << 8;
		val |= spi_read_write(cfg->spi, 0xff);
	}
	mcp3914_deselect(cfg);
	return val;
}

uint32_t mcp3914_read_channel(const struct MCP3914_PORT_CFG *cfg, unsigned channel, unsigned len)
{
	uint32_t val = 0;
	assert(len >= 2 && len <= 4);
	mcp3914_select(cfg);
	spi_read_write(cfg->spi, READ_CMD(MCP3914_CMD_ADDRESS, MCP3914_REG_CHANNEL_BASE + channel));
	switch (len)
	{
	case 4:
		val |= spi_read_write(cfg->spi, 0xff) << 24;
	case 3:
		val |= spi_read_write(cfg->spi, 0xff) << 16;
	case 2:
		val |= spi_read_write(cfg->spi, 0xff) << 8;
		val |= spi_read_write(cfg->spi, 0xff);
	}
	mcp3914_deselect(cfg);
	return val;
}

void mcp3914_stream_start(const struct MCP3914_PORT_CFG *cfg, unsigned reg)
{
	mcp3914_select(cfg);
	spi_read_write(cfg->spi, READ_CMD(MCP3914_CMD_ADDRESS, reg));
}

void mcp3914_stream_end(const struct MCP3914_PORT_CFG *cfg)
{
	mcp3914_deselect(cfg);
}
