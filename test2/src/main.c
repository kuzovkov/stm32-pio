#include <stm32f1xx_ll_rcc.h>
#include <stm32f1xx_ll_cortex.h>
#include <stm32f1xx.h>
#include <stm32f103xb.h>

#define 	RCC_CFGR_PLLSRC_HSE   ((uint32_t)0x00010000)

/**
 * Настройка тактирования
 * 
 */
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
 * инициализация GPIO
 * 
 */
void PORTC_13_INIT(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
  GPIOC->CRH &= ~GPIO_CRH_MODE13;
  GPIOC->CRH |= GPIO_CRH_MODE13_1;
  GPIOC->CRH &= ~GPIO_CRH_CNF13;
}

/**
 * инициализация GPIO
 * 
 */
void PORTB_2_INIT(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  GPIOB->CRL &= ~GPIO_CRL_MODE2_0;
  GPIOB->CRL |= GPIO_CRL_MODE2_1;
  GPIOB->CRL &= ~GPIO_CRL_CNF2_0;
  GPIOB->CRL &= ~GPIO_CRL_CNF2_1;

}

/**
 * инициализация GPIO
 * 
 */
void PORTB_5_INIT(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  GPIOB->CRL &= ~GPIO_CRL_MODE5_0;
  GPIOB->CRL |= GPIO_CRL_MODE5_1;
  GPIOB->CRL &= ~GPIO_CRL_CNF5_0;
  GPIOB->CRL &= ~GPIO_CRL_CNF5_1;

}


/**
 * Мигание светодиодами на плате (PB2), PC13, PB5
 * @return int 
 */
int main(void)
{
  SetSysClockTo72();
  PORTC_13_INIT();
  PORTB_2_INIT();
  PORTB_5_INIT();
  while (1)
  {
    GPIOC->BSRR |= GPIO_BSRR_BS13; //установка в 1
    GPIOB->BSRR |= GPIO_BSRR_BS2;//установка в 1
    GPIOB->BSRR |= GPIO_BSRR_BS5;//установка в 1
    for(int i = 0; i < 1000000; i++);//задерржка
    GPIOC->BSRR |= GPIO_BSRR_BR13;//установка в 0
    GPIOB->BSRR |= GPIO_BSRR_BR2;//установка в 0
    GPIOB->BSRR |= GPIO_BSRR_BR5;//установка в 0
    for(int i = 0; i < 1000000; i++);//задерржка
  }
  
  return 0;
}