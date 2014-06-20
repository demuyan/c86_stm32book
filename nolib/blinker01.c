//http://gitorious.org/~tormod/unofficial-clones/dfuse-dfu-util
//dfu-util -d 0483:df11 -c 1 -i 0 -a 0 -s 0x08000000 -D flashblinker.bin

#include "stm32f407xx.h"

void PUT32 ( unsigned int, unsigned int );
unsigned int GET32 ( unsigned int );

// #define PERIPH_BASE     0x40000000

/* USER LED connected to PIN 5 of GPIOA */
#define LED_PIN         5
#define OUTPUT_MODE     (0x10|0x03) // output mode: push-pull

/* RCC peripheral addresses applicable to GPIOA */
//#define RCC_BASE        (PERIPH_BASE + 0x23800)
#define RCC_AHB1ENR     (RCC_BASE + 0x30)
 
/* GPIOA peripheral addresses */
//#define GPIOA_BASE      (PERIPH_BASE + 0x20000)
#define GPIOA_MODER     (GPIOA_BASE)
#define GPIOA_BSRR      (GPIOA_BASE + 0x10)
#define GPIOA_OSPEEDR   (GPIOA_BASE + 0x8)
#define GPIOA_ODR       (GPIOA_BASE + 0x14)
#define GPIOA_PUPDR     (GPIOA_BASE + 0x0c)

int notmain ( void )
{
    volatile unsigned int ra;
    unsigned int rx;

    ra=GET32(RCC_AHB1ENR);
    ra|=1<<0; //enable port A
    PUT32(RCC_AHB1ENR,ra);

    //d5 output
    ra=GET32(GPIOA_MODER);
    ra&= ~(3 << (5*2));
    ra|= 1 << (5*2);
    PUT32(GPIOA_MODER,ra);

    // speed
    ra=GET32(GPIOA_OSPEEDR);
    ra&= ~(3 << (5*2));
    ra|= 2 << (5*2);
    PUT32(GPIOA_OSPEEDR,ra);

    //pull-up
    ra=GET32(GPIOA_PUPDR);
    ra&= ~(3 << (5*2));
    ra|= 1 << (5*2);
    PUT32(GPIOA_PUPDR,ra);

    for(rx=0;;rx++)
    {
      unsigned int ab = GET32(GPIOA_ODR);      // set LED pin high
      ab ^= 0x20; 
      PUT32(GPIOA_ODR,ab);   
      for(ra=0;ra<200000;ra++) continue;
    }
    return(0);
}
