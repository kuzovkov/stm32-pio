#include <stm32f1xx_ll_rcc.h>
#include <stm32f1xx_ll_cortex.h>
#include <stm32f1xx.h>
#include <stm32f103xb.h>
#include <stm32f1xx_ll_gpio.h>
#include <stdio.h>
#include "stm32f1xx_hal.h" 

#define 	RCC_CFGR_PLLSRC_HSE   ((uint32_t)0x00010000)
#define LED_PB2_ON() GPIOB->BSRR |= GPIO_BSRR_BS2
#define LED_PB2_OFF() GPIOB->BSRR |= GPIO_BSRR_BR2

uint8_t ButtonState = 0;
UART_HandleTypeDef huart2;

void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART1;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

int __io_putchar(int ch)
{
    ITM_SendChar(ch);
    return ch;
}

void SetSysClockTo72 (void)
{
	RCC->CR |= RCC_CR_HSEON; //BIT N16 HSEON
	while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == RESET) {}
	FLASH-> ACR &= ~FLASH_ACR_PRFTBE;
  FLASH-> ACR |= FLASH_ACR_PRFTBE;
	FLASH-> ACR &= ~FLASH_ACR_LATENCY;
	FLASH-> ACR |= FLASH_ACR_LATENCY_2; //set 010
	RCC-> CFGR &= -RCC_CFGR_HPRE;
	RCC-> CFGR |= RCC_CFGR_HPRE_DIV1;
	RCC-> CFGR &= -RCC_CFGR_PPRE2;
	RCC-> CFGR |= RCC_CFGR_PPRE2_DIV1;
	RCC-> CFGR &= ~RCC_CFGR_PPRE1;
  RCC-> CFGR |= RCC_CFGR_PPRE1_DIV2;

  RCC-> CFGR &= (uint32_t) ((uint32_t) ~ (RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
	RCC-> CFGR |=	(uint32_t) (RCC_CFGR_PLLSRC_HSE |	RCC_CFGR_PLLMULL9);
	RCC-> CR |= RCC_CR_PLLON;
	while(READ_BIT(RCC->CR, RCC_CR_PLLRDY)!=(RCC_CR_PLLRDY)){}
	RCC-> CFGR &= ~RCC_CFGR_SW;
	RCC-> CFGR |= RCC_CFGR_SW_PLL; //10 - PLL

  while (READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL){}
		
}

/**
 * инициализация кнопки
 * 
 */
void PINA_0_INIT(void) //Button PA0
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  GPIOA->CRL &= ~GPIO_CRL_MODE0_0;
  GPIOA->CRL &= ~GPIO_CRL_MODE0_1;
  GPIOA->CRL &= ~GPIO_CRL_CNF0_0;
  GPIOA->CRL |= GPIO_CRL_CNF0_1;
}

/**
 * инициализация GPIO PB2
 * 
 */
void PINB_2_INIT(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  GPIOB->CRL &= ~GPIO_CRL_MODE2_0;
  GPIOB->CRL |= GPIO_CRL_MODE2_1;
  GPIOB->CRL &= ~GPIO_CRL_CNF2_0;
  GPIOB->CRL &= ~GPIO_CRL_CNF2_1;
}

/**
 * Включение светодиода на плате при нажатии кнопки 
 * 
 * @return int 
 */
int main(void)
{
  HAL_Init();
  SetSysClockTo72();
  PINB_2_INIT();
  PINA_0_INIT();
  while(1)
  {
     ButtonState = READ_BIT(GPIOA->IDR, GPIO_IDR_IDR0);
     if(ButtonState == 1)
     {
       LED_PB2_ON();
       printf("LED_PB2_ON()");
     }
     else
     {
       LED_PB2_OFF();
       printf("LED_PB2_OFF()");
     }
   }

  return 0;
}