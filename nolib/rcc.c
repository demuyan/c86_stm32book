

#include "stm32f407xx.h"

#include "define_rcc.h"
#include "systick.h"

static __IO uint32_t uwtick;

static inline void NVIC_SetPriority2(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    SCB->SHP[((uint32_t)(IRQn) & 0xF)-4] = ((priority << (8 - __NVIC_PRIO_BITS)) & 0xff); } /* set Priority for Cortex-M  System Interrupts */
  else {
    NVIC->IP[(uint32_t)(IRQn)] = ((priority << (8 - __NVIC_PRIO_BITS)) & 0xff);    }        /* set Priority for device specific Interrupts  */
}

static inline uint32_t SysTick_Config2(uint32_t ticks)
{
  if ((ticks - 1) > SysTick_LOAD_RELOAD_Msk)  return (1);      /* Reload value impossible */

  SysTick->LOAD  = ticks - 1;                                  /* set reload register */
  NVIC_SetPriority2(SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);  /* set Priority for Systick Interrupt */
  SysTick->VAL   = 0;                                          /* Load the SysTick Counter Value */
  SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                   SysTick_CTRL_TICKINT_Msk   |
                   SysTick_CTRL_ENABLE_Msk;                    /* Enable SysTick IRQ and SysTick Timer */
  return (0);                                                  /* Function successful */
}

void Delay(__IO uint32_t Delay)
{
  uint32_t timingdelay;
  
  timingdelay = uwtick + Delay;
  while(uwtick < timingdelay)
  {
    asm("nop");
  }
}


void SysTick_Handler(void)
{
  uwtick++;
}

uint32_t HAL_RCC_OscConfig()
{

//  uint32_t timeout = 0;   

  /*----------------------------- HSI Configuration --------------------------*/
    
  /* When the HSI is used as system clock it will not disabled */
  if((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_HSI) || ((__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_PLL) && ((RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSI)))
  {
  }
  else
  {
    /* Enable the Internal High Speed oscillator (HSI). */
    __HAL_RCC_HSI_ENABLE();

    /* Wait till HSI is ready */  
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET){} 
                
    /* Adjusts the Internal High Speed oscillator (HSI) calibration value.*/
    __HAL_RCC_HSI_CALIBRATIONVALUE_ADJUST(16);
  }

  /*-------------------------------- PLL Configuration -----------------------*/
  /* Check the parameters */
  /* Check if the PLL is used as system clock or not */
  if(__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_CFGR_SWS_PLL)
  { 
    /* Disable the main PLL. */
    __HAL_RCC_PLL_DISABLE();

    /* Wait till PLL is ready */  
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) != RESET){}        

    /* Configure the main PLL clock source, multiplication and division factors. */
    __HAL_RCC_PLL_CONFIG(RCC_PLLSOURCE_HSI,
                         16,
                         336,
                         RCC_PLLP_DIV4,
                         7);
    /* Enable the main PLL. */
    __HAL_RCC_PLL_ENABLE();

    /* Wait till PLL is ready */  
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET){ }
  }
  else
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

uint32_t HAL_RCC_ClockConfig()
{

//  uint32_t timeout = 0;   
 
  /* To correctly read data from FLASH memory, the number of wait states (LATENCY) 
    must be correctly programmed according to the frequency of the CPU clock 
    (HCLK) and the supply voltage of the device. */
  
  /* Increasing the CPU frequency */
  if(FLASH_LATENCY_2 > (FLASH->ACR & FLASH_ACR_LATENCY))
  {    
    /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_2);
    
    /* Check that the new number of wait states is taken into account to access the Flash
       memory by reading the FLASH_ACR register */
    if((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_LATENCY_2)
    {
      return HAL_ERROR;
    }

    /* Check the PLL ready flag */  
    if(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET)
    {
      return HAL_ERROR;
    }
    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_SYSCLKSOURCE_PLLCLK);

    while (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_CFGR_SWS_PLL){}
  }
  /* Decreasing the CPU frequency */
  else
  {

    /* Check the PLL ready flag */  
    if(__HAL_RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET)
    {
      return HAL_ERROR;
    }

    MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, RCC_SYSCLKSOURCE_PLLCLK);

    while (__HAL_RCC_GET_SYSCLK_SOURCE() != RCC_CFGR_SWS_PLL){}
    
    /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_2);
    
    /* Check that the new number of wait states is taken into account to access the Flash
       memory by reading the FLASH_ACR register */
    if((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_LATENCY_2)
    {
      return HAL_ERROR;
    }
  }
  
  /*-------------------------- HCLK Configuration ----------------------------*/ 
  MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_SYSCLK_DIV1);
  
  /*-------------------------- PCLK1 Configuration ---------------------------*/ 
  
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_HCLK_DIV2);
  
  /*-------------------------- PCLK2 Configuration ---------------------------*/ 
  MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, ((RCC_HCLK_DIV1) << 3));
  
  /* Setup SysTick Timer for 1 msec interrupts.
     ------------------------------------------
    The SysTick_Config() function is a CMSIS function which configure:
       - The SysTick Reload register with value passed as function parameter.
       - Configure the SysTick IRQ priority to the lowest value (0x0F).
       - Reset the SysTick Counter register.
       - Configure the SysTick Counter clock source to be Core Clock Source (HCLK).
       - Enable the SysTick Interrupt.
       - Start the SysTick Counter.*/
  SysTick_Config2(84000000 / 1000);
  
  return HAL_OK;
}
