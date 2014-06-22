
#include "stm32f407xx.h"
#include <string.h>
#include <stdio.h>

#define PIN_5 (5)
#define PIN_9 (9)
#define PIN_10 (10)
#define GPIO_MODE_OUTPUT_PP    ((uint32_t)0x00000001)  /*!< Output Push Pull Mode   */
#define GPIO_MODE_AF_PP        ((uint32_t)0x00000002)   /*!< Alternate Function Push Pull Mode     */

#define  GPIO_SPEED_HIGH        ((uint32_t)0x00000003)  /*!< High speed    */
 
#define GPIO_PULLUP            ((uint32_t)0x00000001)  /*!< Pull-up activation      */

// init user LED
void init_ld2()
{
  
  RCC->AHB1ENR |= 1;

  //d5 output
  GPIOA->MODER &= ~(GPIO_MODER_MODER0 << (PIN_5 * 2));
  GPIOA->MODER |= GPIO_MODE_OUTPUT_PP << (PIN_5 * 2);
  
  // speed
  GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (PIN_5 * 2));
  GPIOA->OSPEEDR |= GPIO_SPEED_HIGH << (PIN_5 * 2);

  //pull-up
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (PIN_5 * 2));
  GPIOA->PUPDR |= (GPIO_PULLUP << (PIN_5 * 2));

}

void init_usart1()
{

  RCC->APB2ENR |= 1 << 4;

  //PA9 output(moder 01)
  GPIOA->MODER &= ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10); 
  GPIOA->MODER |= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;
  
  // speed (high speed)
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9 | GPIO_OSPEEDER_OSPEEDR10;

  // Output Type
  GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_10);

  //pull-up
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR9 | GPIO_PUPDR_PUPDR10);
  GPIOA->PUPDR |= GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR10_0;

  // alternate function
  GPIOA->AFR[1] |= 0x770;

  USART1->CR1 &= (uint32_t)~((uint32_t)~USART_CR1_UE);;

  /* Clear STOP[13:12] bits */
//  USART1->CR2 &= (uint32_t)~((uint32_t)USART_CR2_STOP);
//  USART1->CR3 &= (uint32_t)~((uint32_t)(USART_CR3_RTSE | USART_CR3_CTSE));

//  USART1->CR1 &= (uint32_t)~((uint32_t)(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8));

  USART1->CR2 = 0;
  USART1->CR3 = 0;
  USART1->CR1 = 0;


  // Configure BRR by deviding the bus clock with the baud rate
  USART1->BRR = (84000000 / 9600);
//  USART1->BRR = 0x222e;

  USART1->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
  
  
}

void usart_snd(int data) {
  USART1->DR = data;

  // wait for TX
  while ((USART1->SR & USART_SR_TXE) == 0);
}


void usart_snd_str(char *str) {
  int   i = 0;
  
  while(str[i] != 0) {
    usart_snd(str[i++]);
  }
}


int notmain ( void )
{
  init_ld2();
  init_usart1();

  while(1)
  {
    char buf[10];

    sprintf(buf, "hello!!\r\n");

    GPIOA->ODR ^= 1 << PIN_5;
    usart_snd_str(buf);

    for(int i=0;i<3000000;i++){
      asm("nop");
    } 

  }
  return(0);
}
