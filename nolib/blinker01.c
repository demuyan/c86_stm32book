
#include "stm32f407xx.h"

#define PIN_5 (5)
#define GPIO_MODE_OUTPUT_PP    ((uint32_t)0x00000001)  /*!< Output Push Pull Mode   */
#define GPIO_SPEED_FAST        ((uint32_t)0x00000002)  /*!< Fast speed              */
#define GPIO_PULLUP            ((uint32_t)0x00000001)  /*!< Pull-up activation      */

int notmain ( void )
{
  RCC->AHB1ENR |= 1;

  //d5 output
  GPIOA->MODER &= ~(GPIO_MODER_MODER0 << (PIN_5 * 2));
  GPIOA->MODER |= GPIO_MODE_OUTPUT_PP << (PIN_5 * 2);
  
  // speed
  GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (PIN_5 * 2));
  GPIOA->OSPEEDR |= GPIO_SPEED_FAST << (PIN_5 * 2);

  //pull-up
  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (PIN_5 * 2));
  GPIOA->PUPDR |= (GPIO_PULLUP << (PIN_5 * 2));

  while(1)
  {
    GPIOA->ODR ^= 1 << PIN_5;
    for(int i=0;i<500000;i++) continue;
  }
  return(0);
}
