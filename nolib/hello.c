#include "stm32f407xx.h"
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include "rcc.h"
#include "systick.h"

#define PIN_5 (5)

#define GPIO_MODE_OUTPUT_PP    ((uint32_t)0x00000001)  /*!< Output Push Pull Mode   */
#define GPIO_MODE_AF_PP        ((uint32_t)0x00000002)   /*!< Alternate Function Push Pull Mode     */
#define GPIO_SPEED_HIGH        ((uint32_t)0x00000003)  /*!< High speed    */
#define GPIO_PULLUP            ((uint32_t)0x00000001)  /*!< Pull-up activation      */


// User LEDの初期化
void init_ld2()
{
  
  RCC->AHB1ENR |= 1;

  //PA5出力(Push & Pullモード)
  GPIOA->MODER &= ~(GPIO_MODER_MODER0 << (PIN_5 * 2));
  GPIOA->MODER |= GPIO_MODE_OUTPUT_PP << (PIN_5 * 2);
  
  // 出力速度　最高速に設定
  GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (PIN_5 * 2));
  GPIOA->OSPEEDR |= GPIO_SPEED_HIGH << (PIN_5 * 2);

  //プルアップ設定
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (PIN_5 * 2));
  GPIOA->PUPDR |= (GPIO_PULLUP << (PIN_5 * 2));
}

// USART1の初期化
// PA9, PA10に対して設定している
void init_usart1()
{

  RCC->APB2ENR |= 1 << 4;

  //PA9、PA10の出力(moder 01)
  GPIOA->MODER &= ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10); 
  GPIOA->MODER |= GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1;
  
  // GPIOの出力速度（高速）
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9 | GPIO_OSPEEDER_OSPEEDR10;

  // Output Type
  GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_10);

  //プルアップ設定
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR9 | GPIO_PUPDR_PUPDR10);
  GPIOA->PUPDR |= GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR10_0;

  // オルタネート設定
  GPIOA->AFR[1] |= 0x770;

  USART1->CR1 &= (uint32_t)~((uint32_t)~USART_CR1_UE);

  /* 8N1設定（8ビット、ノンパリティ、ストップビット1bit設定） */
  USART1->CR2 &= (uint32_t)~((uint32_t)USART_CR2_STOP);
  USART1->CR3 &= (uint32_t)~((uint32_t)(USART_CR3_RTSE | USART_CR3_CTSE));

  USART1->CR1 &= (uint32_t)~((uint32_t)(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8));

  // 9600bsp設定
  USART1->BRR = (84000000 / 9600);

  //USART1許可 / Tx許可 /Rx許可
  USART1->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

// USART1へ1文字出力する
void send_char_usart1(int data) {
  USART1->DR = data;

  // wait for TX
  while ((USART1->SR & USART_SR_TXE) == 0);
}

// 文字列を出力する
void send_string_usart1(char *str) {
  int   i = 0;
  
  while(str[i] != 0) {
    send_char_usart1(str[i++]);
  }
}

// main関数相当の処理
int notmain ( void )
{
  // RCC(RealtimeClockControl)の初期化
  HAL_RCC_OscConfig();
  HAL_RCC_ClockConfig();

  // User LEDの初期化
  init_ld2();
  // USART1の初期化
  init_usart1();

  while(1)
  {
    // LEDを明滅させる
    GPIOA->ODR ^= 1 << PIN_5;
    // 文字列をUSART1で出力する
    send_string_usart1("hello world\r\n");

    // 1000ms(=1sec)待つ
    Delay(1000);

  }
  return(0);
}

