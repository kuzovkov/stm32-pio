#include <stm32f1xx_ll_rcc.h>
#include <stm32f1xx_ll_cortex.h>
#include <stm32f1xx.h>
#include <stm32f103xb.h>
#include <stm32f1xx_ll_gpio.h>

#define 	RCC_CFGR_PLLSRC_HSE   ((uint32_t)0x00010000)

uint32_t temp = 0; //для сохранения принятых байт
uint32_t i; //для работы счетчика задержки
char data = 0; //для отправки данных на ПК

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
 * инициализация GPIO PB2 (светодиод на плате, плата BluePill+)
 * 
 */
void PINB_2_INIT(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; //RCC
  GPIOB->CRL &= ~GPIO_CRL_MODE2_0; //0: Выход, максимальная частота 2 MHz;
  GPIOB->CRL |= GPIO_CRL_MODE2_1; //1: Выход, максимальная частота 2 MHz;
  GPIOB->CRL &= ~GPIO_CRL_CNF2_0; //00: General purpose output push-pull — выход в режиме Push-pull;
  GPIOB->CRL &= ~GPIO_CRL_CNF2_1; //00: General purpose output push-pull - выход в режиме Push-pull;
}

/**
 * инициализация GPIO PC13 (светодиод на плате, плата BluePill стандартная)
 * 
 */
void PINC_13_INIT(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; //RCC
  GPIOC->CRH &= ~GPIO_CRH_MODE13_0; //0: Выход, максимальная частота 2 MHz;
  GPIOC->CRH |= GPIO_CRH_MODE13_1; //1: Выход, максимальная частота 2 MHz;
  GPIOC->CRH &= ~GPIO_CRH_CNF13_0; //00: General purpose output push-pull — выход в режиме Push-pull;
  GPIOC->CRH &= ~GPIO_CRH_CNF13_1; //00: General purpose output push-pull - выход в режиме Push-pull;
}



void USART1_IRQHandler (void)
{
  if ((USART1-> SR & USART_SR_RXNE) !=0) //Условие попадания в обработчик
  {
    temp = USART1-> DR;
    if (temp == '1')
    {
      // GPIOC-> ODR ^= GPIO_ODR_ODR13; //BluePill
      GPIOB-> ODR ^= GPIO_ODR_ODR2; //BluePill+
    }
    else if (temp == '2')
    {
      USART1-> DR = 0xAE; //символ товарного знака
    }
  }
}

/**
 * Работа с USART в режиме прерываний
 * 
 * @return int 
 */
int main(void)
{
  SetSysClockTo72();
  // инициализация всех портов тактирования
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;
  //USART1 настройка вывода на передачу
  GPIOA->CRH &= ~GPIO_CRH_CNF9; // clear CNF bit 9
  GPIOA->CRH |= GPIO_CRH_CNF9_1; // set CNF bit 9 to 10 Push-Pull
  GPIOA->CRH |= GPIO_CRH_MODE9_0;
  //настройка вывода на прием
  GPIOA->CRH &= ~GPIO_CRH_CNF10;// clear CNF bit 10
  GPIOA->CRH |= GPIO_CRH_CNF10_0;// set CNF bit 10 to 10 Hiz
  GPIOA->CRH &= ~GPIO_CRH_MODE10;

  //Настройка USART1 регистрами
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN; //USART1 clock on
  USART1->BRR = 0x1D4C; //baudrate 9600 and 72 Mhz RCC
  USART1->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; //USART1 on, TX on, RX on, разрешаем прерывания по риему данных

  PINB_2_INIT();
  //инициализация прерывания для USART1
  NVIC_EnableIRQ(USART1_IRQn);
  __enable_irq(); // разрешаем прерывания на глобальном уровне

  while(1)
  {
    for (int i = 0; i < 10000000; i++){};
    USART1->DR = data++; //записываем данные для передачи
    while((USART1->SR & USART_SR_TC) == 0){}; // ждем окончание передачи данных
    USART1->SR = ~USART_SR_TC; //очищаем флаг окончания переда­чи для возможности дальнейшей безопасной передачи

  }
  return 0;
}
