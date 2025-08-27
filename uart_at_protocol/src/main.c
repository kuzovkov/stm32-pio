#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

// Blue Pill LED on PB2 per task
#define LED_GPIO_PORT GPIOB
#define LED_PIN GPIO_PIN_2

// USART1 for RX/TX with interrupts
UART_HandleTypeDef huart1;

// RX line buffer
#define RX_BUFFER_SIZE 128
static uint8_t rx_byte;
static char rx_line[RX_BUFFER_SIZE];
static uint16_t rx_index = 0;

// Simple TX ring buffer for non-blocking prints
#define TX_BUFFER_SIZE 512
static uint8_t tx_buffer[TX_BUFFER_SIZE];
static volatile uint16_t tx_head = 0; // next write
static volatile uint16_t tx_tail = 0; // next read
static volatile bool tx_busy = false;

static volatile bool led_state = false;

// Forward declarations
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void at_process_line(const char *line);
static void uart_tx_start_if_needed(void);
static void uart_write_bytes(const uint8_t *data, uint16_t len);
static void uart_write_str(const char *s);
static void uart_write_line(const char *s);
static void uart_printf(const char *fmt, ...);

// PlatformIO provides SystemInit in startup; HAL will use SysTick for HAL_GetTick

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_USART1_UART_Init();

	// Start RX interrupt for single byte reception
	HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

	// Announce ready
	uart_write_line("READY");

	while (1)
	{
		// Blink handling for non-blocking could be timer-based; simple idle loop here
	}
}

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	__HAL_RCC_AFIO_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_AFIO_REMAP_SWJ_NOJTAG();

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9; // 8MHz * 9 = 72MHz
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		while (1) {}
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
			RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; // 36MHz
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; // 72MHz
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		while (1) {}
	}

	// SysTick at 1ms
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

static void MX_GPIO_Init(void)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// PB2 LED
	GPIO_InitStruct.Pin = LED_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);

	HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET);
}

static void MX_USART1_UART_Init(void)
{
	__HAL_RCC_USART1_CLK_ENABLE();

	// USART1 TX PA9, RX PA10
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_9; // TX
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_10; // RX
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		while (1) {}
	}

	HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}

static inline uint16_t advance_index(uint16_t idx)
{
	return (uint16_t)((idx + 1) % TX_BUFFER_SIZE);
}

static void uart_tx_start_if_needed(void)
{
	if (!tx_busy && tx_head != tx_tail)
	{
		tx_busy = true;
		uint8_t b = tx_buffer[tx_tail];
		tx_tail = advance_index(tx_tail);
		HAL_UART_Transmit_IT(&huart1, &b, 1);
	}
}

static void uart_write_bytes(const uint8_t *data, uint16_t len)
{
	for (uint16_t i = 0; i < len; i++)
	{
		uint16_t next = advance_index(tx_head);
		// drop if buffer full (simple handling)
		if (next == tx_tail)
			break;
		tx_buffer[tx_head] = data[i];
		tx_head = next;
	}
	uart_tx_start_if_needed();
}

static void uart_write_str(const char *s)
{
	uart_write_bytes((const uint8_t *)s, (uint16_t)strlen(s));
}

static void uart_write_line(const char *s)
{
	uart_write_str(s);
	uart_write_str("\r\n");
}

static void uart_printf(const char *fmt, ...)
{
	char buf[128];
	va_list ap;
	va_start(ap, fmt);
	int n = vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	if (n < 0) return;
	if (n > (int)sizeof(buf)) n = sizeof(buf);
	uart_write_bytes((uint8_t *)buf, (uint16_t)n);
}

// RX complete callback - collect lines ended with \r or \n
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		uint8_t ch = rx_byte;
		if (ch == '\r' || ch == '\n')
		{
			if (rx_index > 0)
			{
				rx_line[rx_index] = '\0';
				at_process_line(rx_line);
				rx_index = 0;
			}
		}
		else
		{
			if (rx_index < (RX_BUFFER_SIZE - 1))
			{
				rx_line[rx_index++] = (char)ch;
			}
		}
		HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		if (tx_tail != tx_head)
		{
			uint8_t b = tx_buffer[tx_tail];
			tx_tail = advance_index(tx_tail);
			HAL_UART_Transmit_IT(&huart1, &b, 1);
		}
		else
		{
			tx_busy = false;
		}
	}
}

void USART1_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart1);
}

static void set_led(bool on)
{
	led_state = on;
	HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void do_blink(unsigned count)
{
	for (unsigned i = 0; i < count; i++)
	{
		set_led(true);
		// Use simple delay loop instead of HAL_Delay to avoid SysTick conflict
		for (volatile uint32_t j = 0; j < 360000; j++) {} // ~500ms at 72MHz
		set_led(false);
		for (volatile uint32_t j = 0; j < 360000; j++) {} // ~500ms at 72MHz
	}
}

static bool starts_with(const char *s, const char *prefix)
{
	return strncmp(s, prefix, strlen(prefix)) == 0;
}

static void trim(char *s)
{
	// Trim leading spaces
	char *p = s;
	while (*p == ' ' || *p == '\t') p++;
	if (p != s) memmove(s, p, strlen(p) + 1);
	// Trim trailing spaces
	size_t n = strlen(s);
	while (n > 0 && (s[n-1] == ' ' || s[n-1] == '\t')) { s[n-1] = '\0'; n--; }
}

static void at_ok(void)
{
	uart_write_line("OK");
}

static void at_error(void)
{
	uart_write_line("ERROR");
}

static void at_process_line(const char *line_in)
{
	char line[RX_BUFFER_SIZE];
	strncpy(line, line_in, sizeof(line));
	line[sizeof(line)-1] = '\0';
	trim(line);

	if (strlen(line) == 0)
		return;

	if (strcmp(line, "AT") == 0)
	{
		at_ok();
		return;
	}

	if (!starts_with(line, "AT+"))
	{
		at_error();
		return;
	}

	const char *cmd = line + 3;

	if (starts_with(cmd, "MODE="))
	{
		const char *arg = cmd + 5;
		if (strcmp(arg, "1") == 0)
		{
			set_led(true);
			at_ok();
			return;
		}
		if (strcmp(arg, "0") == 0)
		{
			set_led(false);
			at_ok();
			return;
		}
		at_error();
		return;
	}

	if (starts_with(cmd, "MODE?"))
	{
		uart_printf("+MODE:%d\r\n", led_state ? 1 : 0);
		at_ok();
		return;
	}

	if (strcmp(cmd, "TIME") == 0)
	{
		uart_printf("+TIME:%lu\r\n", (unsigned long)HAL_GetTick());
		at_ok();
		return;
	}

	if (starts_with(cmd, "BLINK="))
	{
		const char *arg = cmd + 6;
		char *endptr = NULL;
		long n = strtol(arg, &endptr, 10);
		if (endptr == arg || n < 0 || n > 1000)
		{
			at_error();
			return;
		}

		do_blink((unsigned)n);
		at_ok();
		return;
	}

	at_error();
}


// HAL weak callbacks for assert and tick if needed (optional)
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}


