#include <stdint.h>
#include <string.h> // memmove

#include <stm32_platform.h>
#include <cm3_vector.h>
#include <stm32_vector.h>
#include <clock.h>
#include <clk_enable.h>
#include <gpio.h>
#include <systick.h>
#include <timer.h>
#ifdef SERIAL_ENABLE
#include <dma.h>
#endif
#ifdef USB_ENABLE
#include <stm32_usb.h>
#endif
#include <board/board.h>

static const struct RCC_CLK_CONFIG clk_cfg =
{
	.clk_src = RCC_SYSCLK_SRC_CRYSTAL,
#ifdef STM32F10X_MD // 72 MHz
	.pll_mul = 9,
	.pll_hse_div = RCC_PLL_HSE_DIV_1,
#endif
	.ahb_pre = RCC_AHB_PRE_1,
	.apb1_pre = RCC_APB_PRE_2,
	.apb2_pre = RCC_APB_PRE_1
};

#define CPU_FREQ (SYSCLK_FREQ(&BOARD_CLOCK_CONFIG, &clk_cfg))

#include "delay.h"

#if defined INPUT_MCP3914
extern void exti3_irq(void);
extern void exti5_9_irq(void);
#elif defined INPUT_SIMULATION
extern void timer_interrupt(void);
#endif
extern void input_setup(void);
extern void input_write_register(unsigned reg, uint32_t val);
extern uint32_t input_read_register(unsigned reg);
extern size_t run_command(const char *p, uint16_t len, char *out_buf);

#include "buffer.h"

enum
{
	RX_BUF_SZ = 64 * 2, // has to be at least two packets to avoid deadlocks when receiving partial packets
	TX_BUF_SZ = 64 * 2,
};

static char rx_buf[RX_BUF_SZ];
static char tx_buf[TX_BUF_SZ];
static unsigned g_rx_ptr = 0;
static unsigned g_tx_unblocked = 0;
static unsigned tx_buffers = 0; // updated from a command

enum MAIN_STAT_TYPE
{
	MAIN_STAT_WORK_TIME,
	MAIN_STAT_COMM_TIME,
	_MAIN_STAT_COUNT
};

static unsigned long main_stat_clk[_MAIN_STAT_COUNT];
static unsigned main_stat_cnt[_MAIN_STAT_COUNT];

static unsigned scan_input(unsigned p, unsigned end)
{
	assert(p <= end);
	while (p != end && rx_buf[p] != '\n')
		++p;
	return p;
}

void read_data(unsigned n)
{
	tx_buffers += n;
}

#ifdef SERIAL_ENABLE

enum
{
	BAUD = 921600,
};

static USART_TypeDef *const uart = USART2;
static DMA_Channel_TypeDef *const channel_tx = DMA1_Channel7; // USART2 TX

static void usart_irq(void)
{
	uint32_t t0 = SysTick->VAL;
	if (uart->SR & USART_SR_RXNE)
	{
		unsigned p = g_rx_ptr; // TODO: barrier?
		assert(p + 1 <= RX_BUF_SZ);
		if (p + 1 <= RX_BUF_SZ)
		{
			rx_buf[p] = USART2->DR;
			g_rx_ptr = p + 1;
		}
	}
	if ((uart->CR1 & USART_CR1_TCIE)
			&& (uart->SR & USART_SR_TC))
	{
		channel_tx->CCR &= ~DMA_CCR1_EN;
		uart->CR1 &= ~USART_CR1_TCIE;
		g_tx_unblocked = 1;
	}
	main_stat_clk[MAIN_STAT_COMM_TIME] += systick_time_interval(&t0);
}

static void tx_xfer_start(const void *data, unsigned len)
{
	// TODO: disable interrupts
	channel_tx->CNDTR = len;
	channel_tx->CMAR = (uint32_t) data;
	uart->SR &= ~USART_SR_TC;
	uart->CR1 |= USART_CR1_TCIE;
	channel_tx->CCR |= DMA_CCR1_EN;
}

#endif

#ifdef USB_ENABLE

#include <usb.h>
#include "usb_device.h"

struct usb_device usb_dev;
static int8_t rx_blocked;
static unsigned tx_tail, tx_head, tx_avail;
static const uint8_t *xfer_tail, *xfer_head, *xfer_end;

static unsigned rx(struct usb_device *dev, uint8_t ep_num, struct usb_pipe *pipe, struct usb_buffer *buf, unsigned n_buf);
static unsigned tx(struct usb_device *dev, uint8_t ep_num, struct usb_pipe *pipe, struct usb_buffer *buf, unsigned n_buf);

static unsigned modulo_add(unsigned modulo, unsigned x, unsigned y)
{
	assert(y <= modulo);
	x += y;
	if (x >= modulo)
		x -= modulo;
	return x;
}

static void rx_start(struct usb_device *dev)
{
	struct usb_buffer rx_buf;
	assert(g_rx_ptr + 64 <= USB_PM_RX_NUM_PKT * 64);
	rx_buf.pm_offset = USB_PM_RX + g_rx_ptr;
	rx_buf.len = 64;
	usb_drv_recv(dev->driver, ENDPOINT_DATA_RX, 64, &rx_buf, 1);
}

void usb_connected(struct usb_device *dev)
{
	assert(USB_PM_RX + 64 <= USB_PM_MAX);
	g_rx_ptr = 0;
	rx_blocked = g_rx_ptr + 64 > USB_PM_RX_NUM_PKT * 64;
	assert(!rx_blocked);
	dev->ep[ENDPOINT_DATA_RX].rx.data_cb = rx;

	if (!rx_blocked)
		rx_start(dev);

	tx_tail = tx_head = 0;
	tx_avail = 2; /* TODO */
	dev->ep[ENDPOINT_DATA_TX].tx.data_cb = tx;
}

static unsigned rx(struct usb_device *dev, uint8_t ep_num, struct usb_pipe *pipe, struct usb_buffer *buf, unsigned n_buf)
{
	uint32_t t0 = SysTick->VAL;
	assert(ep_num == ENDPOINT_DATA_RX);
	assert(n_buf == 1);
	assert(g_rx_ptr + buf[0].len <= RX_BUF_SZ);
	usb_drv_copy_from_pm(dev->driver, ep_num, rx_buf + g_rx_ptr, buf[0]);
	g_rx_ptr += buf[0].len;
	rx_blocked = g_rx_ptr + 64 > RX_BUF_SZ;
	buf[0].len = 64;
	main_stat_clk[MAIN_STAT_COMM_TIME] += systick_time_interval(&t0);
	return rx_blocked ? 0 : 1;
}

static unsigned tx_progress(struct usb_buffer *buf, unsigned n_buf)
{
	unsigned i;
	for (i = 0; i != n_buf && xfer_tail != xfer_end; ++i)
	{
		buf[i].pm_offset = USB_PM_TX + tx_tail * 64;
		unsigned c = 64;
		if (c > xfer_end - xfer_tail)
			c = xfer_end - xfer_tail;
		buf[i].len = c;
		usb_drv_copy_to_pm(usb_dev.driver, ENDPOINT_DATA_TX, xfer_tail, buf[i]);
		xfer_tail += c;
		tx_tail = modulo_add(USB_PM_TX_NUM_PKT, tx_tail, 1);
	}
	return i;
}

static void tx_xfer_start(const void *data, unsigned len)
{
	assert(g_tx_unblocked);
	xfer_tail = xfer_head = (const uint8_t *)data;
	xfer_end = xfer_head + len;
	struct usb_buffer buf[USB_PM_TX_NUM_PKT];
	unsigned n_buf = tx_progress(buf, tx_avail);
	unsigned posted = usb_drv_send(usb_dev.driver, ENDPOINT_DATA_TX, len, buf, n_buf);
	assert(posted == n_buf);
	tx_avail -= posted;
}

static unsigned tx(struct usb_device *dev, uint8_t ep_num, struct usb_pipe *pipe, struct usb_buffer *buf, unsigned n_buf)
{
	uint32_t t0 = SysTick->VAL;
	assert(ep_num == ENDPOINT_DATA_TX);
	assert(n_buf != 0);
	assert(n_buf <= USB_PM_TX_NUM_PKT);
	unsigned posted = 0;
	unsigned i;
	for (i = 0; i != n_buf; ++i)
	{
		assert(buf[i].pm_offset == USB_PM_TX + tx_head * 64);
		assert(xfer_head + buf[i].len <= xfer_end);
		xfer_head += buf[i].len;
		tx_head = modulo_add(USB_PM_TX_NUM_PKT, tx_head, 1);
	}
	if (xfer_head == xfer_end)
	{
		assert(tx_avail + n_buf == 2); // TODO: 2
		g_tx_unblocked = 1;
	}
	else
	{
		// TODO: test with spare buffers, also need array of lengths
		if (tx_avail == 0)
			posted = tx_progress(buf, n_buf);
	}
	tx_avail += n_buf - posted;
	main_stat_clk[MAIN_STAT_COMM_TIME] += systick_time_interval(&t0);
	return posted;
}

#endif

__attribute__ ((section(".isr_vector_core")))
const struct CoreInterruptVector g_pfnVectors_Core =
{
	.estack = &_estack,
	.Reset_Handler = Reset_Handler,
};

__attribute__ ((section(".isr_vector_ext")))
const struct ExtInterruptVector g_pfnVectors_Ext =
{
#if defined INPUT_MCP3914
	.EXTI3_IRQHandler = exti3_irq,
	.EXTI9_5_IRQHandler = exti5_9_irq,
#elif defined INPUT_SIMULATION
	.TIM2_IRQHandler = timer_interrupt,
#endif
#ifdef SERIAL_ENABLE
	.USART2_IRQHandler = usart_irq,
#endif
};

void write_reg(unsigned reg, uint32_t val)
{
	if (reg < 1000)
		input_write_register(reg, val);
}

#ifdef INPUT_SIMULATION
extern unsigned long g_stat_irq_clk;
extern unsigned g_stat_irq_cnt;
#endif

uint32_t read_reg(unsigned reg)
{
	if (reg < 1000)
		return input_read_register(reg);

	reg -= 1000;

#ifdef USB_ENABLE
	if (reg < (_MAIN_STAT_COUNT + _USB_STAT_COUNT) * 2)
	{
		unsigned kind = reg / (_MAIN_STAT_COUNT + _USB_STAT_COUNT);
		unsigned r = reg % (_MAIN_STAT_COUNT + _USB_STAT_COUNT);
		switch (kind)
		{
		case 0: return (r < _USB_STAT_COUNT) ? usb_stat_cnt[r] : main_stat_cnt[r - _USB_STAT_COUNT];
		case 1: return (r < _USB_STAT_COUNT) ? usb_stat_clk[r] : main_stat_clk[r - _USB_STAT_COUNT];
		}
	}
#endif

	reg -= 1000;

#ifdef INPUT_SIMULATION
	switch (reg)
	{
	case 0: return g_stat_irq_clk;
	case 1: return g_stat_irq_cnt;
	}
#endif

	return 0;
}

void new_sample(int value)
{
	LED_PORT->ODR ^= value << LED_PIN1;
}

void enable_interrupt(unsigned irq)
{
	uint8_t pri = (0x700 - (SCB->AIRCR & 0x700)) >> 8;
	uint8_t pre = 4 - pri;
	uint8_t sub = 0xf >> pri;

	pri = 0 << pre; // TODO: preemption priority
	pri |= 0 & sub; // TODO: subpriority
	pri = pri << 4;

	NVIC->IP[irq] = pri;
	NVIC->ISER[irq >> 5] = (uint32_t) 1 << (irq & 0x1f);
	NVIC->ICPR[irq >> 5] = (1 << (irq & 0x1f)); /* Clear pending interrupt */ // TODO: move
}

#define START_CRITICAL_SECTION() \
		do { __disable_irq(); asm volatile ("" ::: "memory"); } while (0)

#define END_CRITICAL_SECTION() \
		do { __enable_irq(); asm volatile ("" ::: "memory"); } while (0)

int main()
{
	rcc_config(&BOARD_CLOCK_CONFIG, &clk_cfg);

	SysTick->LOAD = SysTick_LOAD_RELOAD_Msk;
	SysTick->VAL = 0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

	clk_enable(LED_PORT);
	gpio_configure_out(LED_PORT, LED_PIN1, GPIO_OUT_PP, GPIO_OUT_SPEED_LOW);

#ifdef SERIAL_ENABLE
	clk_enable(GPIOA);
	gpio_configure_af(GPIOA, 2, GPIO_OUT_PP, GPIO_OUT_SPEED_MEDIUM); // USART2 TX
	gpio_configure_in(GPIOA, 3); // USART2 RX

	clk_enable(uart);
	uart->BRR = (APB1_FREQ(&BOARD_CLOCK_CONFIG, &clk_cfg) * 2 + BAUD / 2) / BAUD;
	uart->CR1 = USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE;
	uart->CR3 = USART_CR3_DMAT;
	enable_interrupt(USART2_IRQn); // both RX and TC

	clk_enable(DMA1);
	channel_tx->CCR = DMA_CCR1_PSIZE_BYTE | DMA_CCR1_MINC | DMA_CCR1_MSIZE_BYTE | DMA_CCR1_DIR;
	channel_tx->CPAR = (uint32_t) &uart->DR;
#endif

#ifdef USB_ENABLE
	clk_enable(GPIOA);

	// set DP low to simulate disconnect
	GPIOA->ODR &= ~(1 << 12);
	gpio_configure_out(GPIOA, 12, GPIO_OUT_PP, GPIO_OUT_SPEED_10MHz);
	delay_ms(50);
	gpio_configure_in(GPIOA, 12);

	clk_enable((void *)USB_BASE);
	usb_dev.driver = usb_drv_init(&usb_dev);
#endif

	unsigned rx_cmd = 0, rx_scn = 0;
	unsigned tx_len = 0, tx_unblocked = 1;

	tx_buffers = 0;

	START_CRITICAL_SECTION();
	g_tx_unblocked = 1;
	g_rx_ptr = 0;
	END_CRITICAL_SECTION();

	input_setup();

	uint32_t clock = SysTick->VAL;
	uint32_t led_time = 0;
	while (1)
	{
#ifdef USB_ENABLE
		usb_drv_poll(usb_dev.driver);
#endif

		if (tx_unblocked // New transfer cannot be started because the same buffer is used for DMA.
				&& tx_len == 0)
		{
			START_CRITICAL_SECTION();
			unsigned rx_ptr = g_rx_ptr;
			END_CRITICAL_SECTION();

			rx_scn = scan_input(rx_scn, rx_ptr);

			if (rx_scn != rx_ptr)
			{
				uint32_t t0 = SysTick->VAL;
				tx_len = run_command(rx_buf + rx_cmd, rx_scn - rx_cmd, tx_buf);
				rx_cmd = ++rx_scn;

				START_CRITICAL_SECTION();
				rx_ptr = g_rx_ptr;
				rx_scn = scan_input(rx_scn, rx_ptr);
				if (rx_scn == rx_ptr)
				{
					memmove(rx_buf, rx_buf + rx_cmd, rx_ptr - rx_cmd);
					rx_scn -= rx_cmd;
					g_rx_ptr = rx_ptr - rx_cmd;
					rx_cmd = 0;
#ifdef USB_ENABLE
					if (rx_blocked && g_rx_ptr + 64 <= USB_PM_RX_NUM_PKT * 64)
					{
						rx_blocked = 0;
						rx_start(&usb_dev);
					}
#endif
				}
				END_CRITICAL_SECTION();
				main_stat_cnt[MAIN_STAT_WORK_TIME]++;
				main_stat_clk[MAIN_STAT_WORK_TIME] += systick_time_interval(&t0);
			}
		}

		if (!tx_unblocked)
		{
			asm volatile ("" ::: "memory");
			if (g_tx_unblocked)
			{
				if (tx_len != 0)
					tx_len = 0;
				else
				{
					tx_buffers--;
					cons_close_buffer();
				}
				tx_unblocked = 1;
			}
		}

		// TODO: wait until streaming ends before outputting cancel status
		if (tx_unblocked)
		{
			if (tx_len != 0)
			{
				uint32_t t0 = SysTick->VAL;
				tx_xfer_start(tx_buf, tx_len);
				g_tx_unblocked = tx_unblocked = 0;
				main_stat_clk[MAIN_STAT_COMM_TIME] += systick_time_interval(&t0);
			}
			else
			if (tx_buffers != 0 && cons_buffer_available())
			{
				uint32_t t0 = SysTick->VAL;
				tx_xfer_start(cons_get_buffer(), BUF_SZ);
				g_tx_unblocked = tx_unblocked = 0;
				main_stat_clk[MAIN_STAT_COMM_TIME] += systick_time_interval(&t0);
			}
		}

		uint32_t elapsed = systick_time_interval(&clock);
		if (periodic_timer(&led_time, CPU_FREQ / 2, elapsed))
			LED_PORT->ODR ^= 1 << LED_PIN1;
	}
}
