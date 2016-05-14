#include <stdlib.h>
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
#ifdef SERIAL_ENABLE
	BAUD = 921600,
#endif
};

static char rx_buf[RX_BUF_SZ];
static char tx_buf[TX_BUF_SZ];
static unsigned g_rx_ptr = 0;
static unsigned g_tx_unblocked = 0;
static unsigned tx_buffers = 0; // updated from a command

#ifdef SERIAL_ENABLE
static USART_TypeDef *const uart = USART2;
static DMA_Channel_TypeDef *const channel_tx = DMA1_Channel7; // USART2 TX
#endif

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
static void usart_irq(void)
{
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
}
#endif

static void tx_xfer_start(const void *p, unsigned len)
{
#ifdef SERIAL_ENABLE
	// TODO: disable interrupts
	channel_tx->CNDTR = len;
	channel_tx->CMAR = (uint32_t) p;
	uart->SR &= ~USART_SR_TC;
	uart->CR1 |= USART_CR1_TCIE;
	channel_tx->CCR |= DMA_CCR1_EN;
#endif
	g_tx_unblocked = 0;
}

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
	if (reg < 2000)
		input_write_register(reg, val);
}

uint32_t read_reg(unsigned reg)
{
	if (reg < 2000)
		return input_read_register(reg);
	else
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
	gpio_configure_out(LED_PORT, LED_PIN1, GPIO_OUT_PP, GPIO_OUT_SPEED_2MHz);

#ifdef SERIAL_ENABLE
	clk_enable(GPIOA);
	gpio_configure_out(GPIOA, 0, GPIO_OUT_PP, GPIO_OUT_SPEED_MEDIUM);
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
		START_CRITICAL_SECTION();
		unsigned rx_ptr = g_rx_ptr;
		END_CRITICAL_SECTION();

		rx_scn = scan_input(rx_scn, rx_ptr);

		if (tx_unblocked // New transfer cannot be started because the same buffer is used for DMA.
				&& tx_len == 0
				&& rx_scn != rx_ptr)
		{
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
			}
			END_CRITICAL_SECTION();
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
				tx_xfer_start(tx_buf, tx_len);
				tx_unblocked = 0;
			}
			else
			if (tx_buffers != 0 && cons_buffer_available())
			{
				tx_xfer_start(cons_get_buffer(), BUF_SZ);
				tx_unblocked = 0;
			}
		}

		uint32_t elapsed = systick_time_interval(&clock);
		if (periodic_timer(&led_time, CPU_FREQ / 2, elapsed))
			LED_PORT->ODR ^= 1 << LED_PIN1;
	}
}
