#include "mbed.h"

//------------------------------------
// Hyperterminal configuration
// 9600 bauds, 8-bit data, no parity
//------------------------------------

#define GPIOA_MODER ((uint32_t)0x40020000)
#define GPIOA_OTYPER ((uint32_t)0x40020004)
#define GPIOA_OSPEEDR ((uint32_t)0x40020008)
#define GPIOA_AFRL  ((uint32_t)0x40020020)
#define GPIOA_AFRH  ((uint32_t)0x40020024)

#define USART1_CR1 ((uint32_t)0x4001100c)
#define USART1_CR2 ((uint32_t)0x40011010)
#define USART1_CR3 ((uint32_t)0x40011014)
#define USART1_GTPR ((uint32_t)0x40011018)

//Serial pc(SERIAL_TX, SERIAL_RX); // USART2
Serial pc(PA_9, PA_10); // USART1

DigitalOut myled(LED1);

int main() {
  int i = 1;
  pc.printf("Hello World !\n");
  while(1) {

    uint32_t* reg = (uint32_t*)USART1_GTPR;
    pc.printf("USART1_GTPR=%x ", *reg);

    reg = (uint32_t*)GPIOA_AFRH;
    pc.printf("GPIOA_AFRH=%x ", *reg);

    reg = (uint32_t*)GPIOA_AFRL;
    pc.printf("GPIOA_AFRL=%x ", *reg);

    wait(1);
//    pc.printf("This program runs since %d seconds.\n", i++);
    myled = !myled;
  }
}


