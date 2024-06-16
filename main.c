/*
File    : main.c
Software "Kurs STM32 PCBtech"
Lesson 6: SPI2 connect to W25Q64 (memory).
Student: antigo1989@gmail.com
*/

#include "main.h"

/***************************inline function*******************************/
#define CS_HIGHT GPIOE->BSRR |= GPIO_BSRR_BS3;
#define CS_LOW GPIOE->BSRR |= GPIO_BSRR_BR3;


//global values
volatile uint32_t addr_read = 0x00;
/*********************************main************************************/
int main(void) {
  //Values
  
  //System init
  SystemInit();
  RCC_Init();
  __enable_irq();
  //GPIO init
  key_led_Init();
  //Connections init
  SPI2_Init();
  
  #ifndef DATA_IN_MEMORY
    w25clear_sector(ADDR1);
    w25write(ADDR1, LED1, SPI_F8B);
    w25write(ADDR2, (LED3|(LED2<<8)), SPI_F16B);
  #endif

  while(1){
    if(addr_read > 0){
      uint8_t mem_read = (uint8_t)w25read(addr_read) & 0xFF;
      switch(mem_read){
        case LED1:
          GPIOE->BSRR |= GPIO_BSRR_BR13|GPIO_BSRR_BS14|GPIO_BSRR_BS15;
          break;
        case LED2:
          GPIOE->BSRR |= GPIO_BSRR_BR14|GPIO_BSRR_BS13|GPIO_BSRR_BS15;
          break;
        case LED3:
          GPIOE->BSRR |= GPIO_BSRR_BR15|GPIO_BSRR_BS13|GPIO_BSRR_BS14;
          break;
        case CLEAR:
          GPIOE->BSRR |= GPIO_BSRR_BS13|GPIO_BSRR_BS14|GPIO_BSRR_BS15;
          break;
      }
      addr_read = 0;
    }
  __NOP();
  }
}

/***********************interrupts function**************************/
//keys
void EXTI15_10_IRQHandler(void){
  switch(EXTI->PR & (EXTI_PR_PR10|EXTI_PR_PR11|EXTI_PR_PR12)){
      case EXTI_PR_PR10:
        addr_read = ADDR1;
        break;
      case EXTI_PR_PR11:
        addr_read = ADDR2;
        break;
      case EXTI_PR_PR12:
        addr_read = ADDR3;
        break;
  } 
  EXTI->PR |= EXTI_PR_PR10|EXTI_PR_PR11|EXTI_PR_PR12;
}

/****************************** function**********************************/
uint16_t w25send_byte(uint16_t data){
  if(!(SPI2->CR1 & SPI_CR1_DFF)){
    SPI2->DR = (data & 0xFF);
  }else{
    SPI2->DR = (data & 0xFFFF);
  }
  while((SPI2->SR & SPI_SR_TXE) == 0){}
  while((SPI2->SR & SPI_SR_RXNE) == 0){}
  return SPI2->DR;
}

void w25write(uint32_t addr, uint16_t data, uint8_t f16b){
  CS_LOW
  w25send_byte(WR_EN);
  CS_HIGHT
  CS_LOW
  w25send_byte(PG_PROG);
  w25send_byte((addr>>16) & 0xFF);
  w25send_byte((addr>>8) & 0xFF);
  w25send_byte(addr & 0xFF);
  if(f16b == SPI_F16B){
    SPI2->CR1 |= SPI_CR1_DFF; //16bit
  }
  w25send_byte(data);
  if(f16b == SPI_F16B){
    SPI2->CR1 &= ~(SPI_CR1_DFF); //8bit
  }
  CS_HIGHT
  CS_LOW
  w25send_byte(RD_SR1);
  while(w25send_byte(0x0000) & 0x01){}
  CS_HIGHT
}

uint8_t w25read(uint32_t addr){
  CS_LOW
  w25send_byte(RD_DATA);
  w25send_byte((addr>>16) & 0xFF);
  w25send_byte((addr>>8) & 0xFF);
  w25send_byte(addr & 0xFF);
  uint8_t ret = w25send_byte(0x00) & 0xFF;
  CS_HIGHT
  return ret;
}

void w25clear_sector(uint32_t addr){
  CS_LOW
  w25send_byte(WR_EN);
  CS_HIGHT
  CS_LOW
  w25send_byte(SECT_ER);
  w25send_byte((addr>>16) & 0xFF);
  w25send_byte((addr>>8) & 0xFF);
  w25send_byte(addr & 0xFF);
  CS_HIGHT
  CS_LOW
  w25send_byte(RD_SR1);
  while((w25send_byte(0x00) & 0x01) == 1){}
  CS_HIGHT
}

void key_led_Init(void){
  // Clock BUS Initial
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PE|SYSCFG_EXTICR3_EXTI11_PE;
  SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI12_PE;
  //GPIO 
  GPIOE->MODER |= GPIO_MODER_MODE13_0|GPIO_MODER_MODE14_0|GPIO_MODER_MODE15_0;
  GPIOE->MODER &= ~(GPIO_MODER_MODE10|GPIO_MODER_MODE11|GPIO_MODER_MODE12);
  //Interrupts keys
  EXTI->PR |= EXTI_PR_PR10|EXTI_PR_PR11|EXTI_PR_PR12;
  EXTI->FTSR |= EXTI_FTSR_TR10|EXTI_FTSR_TR11|EXTI_FTSR_TR12;
  EXTI->IMR |= EXTI_IMR_IM10|EXTI_IMR_IM11|EXTI_IMR_IM12;
  //Interrupt NVIC Enable
  NVIC_EnableIRQ(EXTI15_10_IRQn);
  //LED turn off
  GPIOE->BSRR |= GPIO_BSRR_BS13|GPIO_BSRR_BS14|GPIO_BSRR_BS15;
}

void SPI2_Init(void){
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN|RCC_AHB1ENR_GPIOCEN|RCC_AHB1ENR_GPIOEEN;
  //MOSI MISO Pins
  GPIOC->MODER |= GPIO_MODER_MODER3_1|GPIO_MODER_MODER2_1;
  GPIOC->AFR[0] |= (5<<GPIO_AFRL_AFSEL3_Pos)|(5<<GPIO_AFRL_AFSEL2_Pos);
  GPIOC->PUPDR |= GPIO_PUPDR_PUPD3_1|GPIO_PUPDR_PUPD2_1;
  //SCK Pin
  GPIOB->MODER |= GPIO_MODER_MODER10_1;
  GPIOB->AFR[1] |= (5<<GPIO_AFRH_AFSEL10_Pos);
  GPIOB->PUPDR |= GPIO_PUPDR_PUPD10_1;
  //CS Pin
  GPIOE->MODER |= GPIO_MODER_MODER3_0;
  GPIOE->OTYPER &= ~(GPIO_OTYPER_OT3);
  GPIOE->PUPDR |= GPIO_PUPDR_PUPD3_1;
  GPIOE->BSRR |= GPIO_BSRR_BS3;
  //SPI
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
  SPI2->CR1 |= (0b100<<SPI_CR1_BR_Pos)|SPI_CR1_MSTR|SPI_CR1_SSM|SPI_CR1_SSI; //42MHz/32 = 1,32MHz Master CS-manual
  SPI2->CR1 &= ~(SPI_CR1_DFF|SPI_CR1_CPHA|SPI_CR1_CPOL); //8bit mode0
  SPI2->CR1 |= SPI_CR1_SPE;
  //W25 memory
  CS_LOW
  w25send_byte(EN_RST);
  CS_HIGHT
  CS_LOW
  w25send_byte(RST);
  CS_HIGHT
}
/*************************** End of file ****************************/
