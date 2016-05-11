#include <assert.h>
#include "mcp3914.h"

#include <spi.h>
#include <clk_enable.h>

void mcp3914_port_configure(const struct MCP3914_PORT_CFG *port, enum GPIO_OUT_SPEED pin_speed, uint32_t spi_prescaler)
{
	port->reset_pin.port->ODR |= 1 << port->reset_pin.pin; // RESET high
	port->cs_pin.port->ODR |= 1 << port->cs_pin.pin; // CS high

	gpio_configure_af(port->clk_pin.port, port->clk_pin.pin, GPIO_OUT_PP, pin_speed);
	gpio_configure_af(port->mosi_pin.port, port->mosi_pin.pin, GPIO_OUT_PP, pin_speed);
	gpio_configure_in(port->miso_pin.port, port->miso_pin.pin);
	gpio_configure_out(port->cs_pin.port, port->cs_pin.pin, GPIO_OUT_PP, pin_speed);
	gpio_configure_out(port->reset_pin.port, port->reset_pin.pin, GPIO_OUT_PP, pin_speed);
	gpio_configure_in(port->dr_pin.port, port->dr_pin.pin);
	gpio_configure_in_pull(port->dr_pin.port, port->dr_pin.pin, GPIO_PULLUP); // single device configuration, requires DR_HIZ setting TODO NOPULL ????

	clk_enable(port->spi);
	port->spi->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | spi_prescaler;
}

void mcp3914_port_init(const struct MCP3914_PORT_CFG *cfg, struct MCP3914_PORT *port)
{
	port->cfg = cfg;
	port->channel_width = 3;
	port->repeat = 8; // TODO

	cfg->reset_pin.port->ODR &= ~(1 << cfg->reset_pin.pin); // RESET high
	delay_us(1000);
	cfg->reset_pin.port->ODR |= 1 << cfg->reset_pin.pin; // RESET high
	delay_us(1000);

	cfg->spi->CR1 |= SPI_CR1_SPE;
}

void mcp3914_write_reg(const struct MCP3914_PORT_CFG *cfg, enum MCP3914_REG reg, uint32_t val)
{
	mcp3914_select(cfg);
	spi_read_write(cfg->spi, (MCP3914_CMD_ADDRESS << 6) | (reg << 1) | MCP3914_CMD_WRITE_MASK);
	if (reg == MCP3914_REG_MOD)
		spi_read_write(cfg->spi, (val >> 24) & 0xff);
	spi_read_write(cfg->spi, (val >> 16) & 0xff);
	spi_read_write(cfg->spi, (val >> 8) & 0xff);
	spi_read_write(cfg->spi, val & 0xff);
	mcp3914_deselect(cfg);
	// TODO verify length of CHANNEL registers WRITE/NON_STREAM
}

uint8_t mcp3914_reg_width(const struct MCP3914_PORT *port, enum MCP3914_REG reg)
{
	if (reg >= MCP3914_REG_CHANNEL_BASE
			&& reg < MCP3914_REG_CHANNEL_BASE + MCP3914_NUM_CHANNELS)
		return port->channel_width;
	else
	if (reg == MCP3914_REG_MOD)
		return 4;
	else
		return 3;
}

uint32_t mcp3914_read_reg(const struct MCP3914_PORT *port, enum MCP3914_REG reg)
{
	const struct MCP3914_PORT_CFG *cfg = port->cfg;
	uint32_t val = 0;
	uint8_t len = mcp3914_reg_width(port, reg);
	assert(len >= 2 && len <= 4);
	mcp3914_select(cfg);
	spi_read_write(cfg->spi, (MCP3914_CMD_ADDRESS << 6) | (reg << 1) | MCP3914_CMD_READ_MASK);
	switch (len)
	{
	case 4:
		val |= spi_read_write(cfg->spi, 0xff) << 24;
	case 3:
		val |= spi_read_write(cfg->spi, 0xff) << 16;
	default:
		val |= spi_read_write(cfg->spi, 0xff) << 8;
		val |= spi_read_write(cfg->spi, 0xff);
	}
	mcp3914_deselect(cfg);
	// TODO verify length of CHANNEL registers READ/NON_STREAM
	return val;
}
