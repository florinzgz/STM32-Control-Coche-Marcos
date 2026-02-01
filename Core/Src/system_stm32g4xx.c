#include "stm32g4xx.h"

uint32_t SystemCoreClock = 170000000;
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};

void SystemInit(void)
{
  #if defined(USER_VECT_TAB_ADDRESS)
    SCB->VTOR = VECT_TAB_BASE_ADDRESS | VECT_TAB_OFFSET;
  #endif
  
  FPU->FPCCR |= ((3UL << 30) | (3UL << 28));
}

void SystemCoreClockUpdate(void)
{
  uint32_t tmp, pllvco, pllr, pllsource, pllm;
  
  tmp = (RCC->CFGR & RCC_CFGR_SWS) >> RCC_CFGR_SWS_Pos;
  
  switch (tmp)
  {
    case 0x01:
      SystemCoreClock = HSE_VALUE;
      break;
    case 0x02:
      pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC);
      pllm = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1U;
      
      if (pllsource == 0x02)
      {
        pllvco = (HSI_VALUE / pllm);
      }
      else
      {
        pllvco = (HSE_VALUE / pllm);
      }
      
      pllvco = pllvco * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos);
      pllr = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos) + 1U) * 2U;
      SystemCoreClock = pllvco/pllr;
      break;
    case 0x00:
    default:
      SystemCoreClock = HSI_VALUE;
      break;
  }
  
  tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos)];
  SystemCoreClock >>= tmp;
}