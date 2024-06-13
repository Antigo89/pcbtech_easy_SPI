/*
File    : main.c

*/
#include "main.h"

//Prototypr internal function
void key_led_Init(void);
//global values
volatile uint8_t user_flags = 0x00;

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

  //LED turn off
  GPIOE->BSRR |= GPIO_BSRR_BS13|GPIO_BSRR_BS14|GPIO_BSRR_BS15;


  while(1){
    __NOP();
  }
}

/****************************** function**********************************/
void key_led_Init(void){
  // Clock BUS Initial
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PE;
  //GPIO 
  GPIOE->MODER |= GPIO_MODER_MODE13_0|GPIO_MODER_MODE14_0|GPIO_MODER_MODE15_0;
  GPIOE->MODER &= ~(GPIO_MODER_MODE10|GPIO_MODER_MODE11|GPIO_MODER_MODE12);
  //Interrupts keys
  EXTI->PR |= EXTI_PR_PR10|EXTI_PR_PR11|EXTI_PR_PR12;
  EXTI->FTSR |= EXTI_FTSR_TR10|EXTI_FTSR_TR11|EXTI_FTSR_TR12;
  EXTI->IMR |= EXTI_IMR_IM10|EXTI_IMR_IM11|EXTI_IMR_IM12;
  //Interrupt NVIC Enable
  NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/***********************interrupts function**************************/
//keys
void EXTI15_10_IRQHandler(void){
  switch(EXTI->PR & (EXTI_PR_PR10|EXTI_PR_PR11|EXTI_PR_PR12)){
    case EXTI_PR_PR10:
      GPIOE->BSRR |= GPIO_BSRR_BR13|GPIO_BSRR_BS14|GPIO_BSRR_BS15;
      break;
    case EXTI_PR_PR11:
      GPIOE->BSRR |= GPIO_BSRR_BR14|GPIO_BSRR_BS13|GPIO_BSRR_BS15;
      break;
    case EXTI_PR_PR12:
      GPIOE->BSRR |= GPIO_BSRR_BR15|GPIO_BSRR_BS13|GPIO_BSRR_BS14;
      break;
  }
  EXTI->PR |= EXTI_PR_PR10|EXTI_PR_PR11|EXTI_PR_PR12;
}

/*************************** End of file ****************************/
