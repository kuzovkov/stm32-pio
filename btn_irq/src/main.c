#include <stm32f1xx_ll_rcc.h>
#include <stm32f1xx_ll_cortex.h>
#include <stm32f1xx.h>
#include <stm32f103xb.h>
#include <stm32f1xx_ll_gpio.h>

#define 	RCC_CFGR_PLLSRC_HSE   ((uint32_t)0x00010000)
#define LED_PB2_ON() GPIOB->BSRR |= GPIO_BSRR_BS2
#define LED_PB2_OFF() GPIOB->BSRR |= GPIO_BSRR_BR2

uint8_t ButtonState = 0;
void Interrupt_EXTI_PA0_Init(void);
void PINA_0_INIT(void);
void PINB_2_INIT(void);


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
void PINA_0_INIT(void) //Button на PA0
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  GPIOA->CRL &= ~GPIO_CRL_MODE0_0;
  GPIOA->CRL &= ~GPIO_CRL_MODE0_1;
  GPIOA->CRL &= ~GPIO_CRL_CNF0_0;
  GPIOA->CRL |= GPIO_CRL_CNF0_1;
  //GPIOA->BSRR = GPIO_BSRR_BS9; 
  /**  подтяжка к питанию но если, мы подключили самосто­ятельно кнопку напрямую к выводу РА0 на плате BluePill без 
  по­добной аппаратной подтяжки, то строку кода нужно раскомментировать.*/
}

/**
 * инициализация GPIO PB2 (светодиод на плате)
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

void Interrupt_EXTI_PA0_Init(void)
{
  EXTI->PR |= EXTI_PR_PR0;
  EXTI->IMR |= EXTI_IMR_IM0;
  AFIO->EXTICR[0] &= ~AFIO_EXTICR1_EXTI0_PA;
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  EXTI->FTSR |= EXTI_FTSR_TR0;
  NVIC_EnableIRQ(EXTI0_IRQn);
  NVIC_SetPriority(EXTI0_IRQn, 0);

}

void EXTI0_IRQHandler(void)
{
  EXTI->PR |= EXTI_PR_PR0;
  if (ButtonState == 1)
  {
    LED_PB2_ON();
    for (int i=0; i< 10000000; i++){};

  } 
}
/**
 * Включение светодиода на плате при нажатии кнопки 
 * 
 * @return int 
 */
int main(void)
{
   SetSysClockTo72();
   PINB_2_INIT();
   PINA_0_INIT();
   Interrupt_EXTI_PA0_Init();

   while(1)
   {
     ButtonState = ((GPIOA->IDR) & (GPIO_IDR_IDR0));
     LED_PB2_OFF();
   }

  return 0;
}
