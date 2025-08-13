#include <stm32f1xx_ll_rcc.h>
#include <stm32f1xx_ll_cortex.h>
#include <stm32f1xx.h>
#include <stm32f103xb.h>
#include <stm32f1xx_ll_gpio.h>
#include <string.h>

#define 	RCC_CFGR_PLLSRC_HSE   ((uint32_t)0x00010000)
#define LED_PB2_ON() GPIOB->BSRR |= GPIO_BSRR_BS2
#define LED_PB2_OFF() GPIOB->BSRR |= GPIO_BSRR_BR2
#define RXSIZE 20

uint8_t ReceivedBuffer[20];
uint8_t MainBuffer[50];
uint8_t Index = 0;

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

void UART1_Config(void)
{
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
  USART1->BRR = 0x271; //baudrate 115200 and 72 Mhz RCC
  USART1->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE; //USART1 on, TX on, RX on, разрешаем прерывания по приему данных
  USART1->CR3 |= (1 << 7); //Включение разрешения передачи через DMA
  USART1->CR3 |= (1 << 6); //Включение разрешения приема через DMA
}

void DMA_Init(void)
{
  RCC->AHBENR |= 1 << 0; //включение тактирования DMA1
  DMA1_Channel5->CCR |= (1 << 1) | (1 << 2) | (1 << 3);  //включение прерываний DMA, TCIE, HTIE, TEIE биты для разрешения прерывания 
  DMA1_Channel5->CCR &= ~(1 << 4); //установка направления передачи данных, читаем биты из периферии
  DMA1_Channel5->CCR |= (1 << 5); //включение циклическогого режима(CIRC), 
  DMA1_Channel5->CCR |= (1 << 7); //увеличение объема памяти(MIRC), 
  DMA1_Channel5->CCR |= (1 << 7); //увеличение объема памяти(MIRC), 
  DMA1_Channel5->CCR &= ~(3 << 3); //размер данных  периферии (PSIZE), 00 : 8bit data
  DMA1_Channel5->CCR &= ~(3 << 10); //размер данных в памяти (MSIZE), 00 : 8bit data
  DMA1_Channel5->CCR &= ~(3 << 12); //уровень приоритета PL=0 низкий
}
/** 
 * @param source адрес источника
 * @param destination адрес назначения
 * @param datasize размер данных
 */
void DMA_Config(uint32_t source, uint32_t destination, uint16_t datasize)
{
  DMA1_Channel5->CNDTR = datasize; //устанавливаем размер данных
  DMA1_Channel5->CPAR = source; //устанавливаем периферийный адрес в регистре PAR
  DMA1_Channel5->CMAR = destination; //устанавливаем адрес памяти в регистре MAR
  DMA1_Channel5->CCR |= 1 << 0; //включить DMA1

}

/**
 * Это основная функция, в которой крутится весь смысл того
чего мы затеяли в этом разделе :).
Имя обработчика взять из ассемблерного файла
startup_stm32f10x_md. s, в котором прописаны все возможные
обработчики прерываний, которые могут быть в данном МК.
 * 
 */
void DMA1_Channel5_IRQHandler(void)
{
  // Если установлено прерывание на половину передачи, то
  if ((DMA1-> ISR) & (1 << 18))
  {
    memcpy (&MainBuffer [Index], &ReceivedBuffer [0], RXSIZE/2);
    DMA1-> IFCR |= (1 << 18);
    Index = Index+ (RXSIZE/2);
    if (Index> 49) 
    {
      Index = 0;
    }
    LED_PB2_ON ();
  }
  // Если установлено прерывание завершения полной пере­дачи, то
  if ((DMA1-> ISR) & (1 << 17))
  {
    memcpy (&MainBuffer [Index],&ReceivedBuffer[RXSIZE/2], RXSIZE/2);
    DMA1-> IFCR |= (1 <<17); //Очистка бита прерывания
    Index = Index+ (RXSIZE/2);
    if (Index> 49)
    {
      Index=0;
    }
    LED_PB2_OFF ();
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
  UART1_Config();
  DMA_Init();
  NVIC_SetPriority(DMA1_Channel5_IRQn, 0);
  NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  DMA_Config((uint32_t) &USART1->DR, (uint32_t) ReceivedBuffer, RXSIZE);
  PINB_2_INIT();

  while(1)
  {
  }
  return 0;
}
