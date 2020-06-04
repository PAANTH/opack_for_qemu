#include "firm_updater.h"

#include <stdint.h>

#include "stm32f1xx_hal.h"

#define VECT_CNT  128U
#define FU_ESTACK  0x2000FFFFU
#define FU_SRAM_BASE  0x20000000U
uint32_t int_vector_table[VECT_CNT] __attribute__((section(".pashahod_section"), aligned(1024)));

static void  fu_tim4_handler(void);
static void  fu_default_handler(void);

static void fu_default_handler(void)
{
  while(1){};
}

static void  fu_tim4_handler(void)
{
  uint8_t set_pin = 0;

  TIM4->CR1 &= ~TIM_CR1_CEN;
  TIM4->CNT = 0;
  TIM4->SR &= ~TIM_SR_UIF;

  set_pin = (GPIOB->ODR & GPIO_PIN_14);

  // if (set_pin) {
     GPIOB->BSRR = GPIO_PIN_14;
  // } else {
  //   GPIOB->BSRR = (uint32_t)GPIO_PIN_14 << 16U;
  // }

  TIM4->CR1 |= TIM_CR1_CEN;
}

static void fu_init_custom_vtable(void)
{
  uint32_t ram_ptr = FU_SRAM_BASE;

  int_vector_table[0] = FU_ESTACK;
  for(int i = 1; i < VECT_CNT; i++) {
    int_vector_table[i] = (uint32_t)fu_default_handler;
  }
  //46 is i if TIM4_IRQHandler
  int_vector_table[46] = (uint32_t)fu_tim4_handler;

  for(int i = 0; i < VECT_CNT; i++) {
    *((uint32_t*)(ram_ptr + i*4)) = int_vector_table[i];
  }

}
void fu_main(void)
{

  uint32_t func_addr = (uint32_t)fu_tim4_handler;
  uint32_t ram_ptr = FU_SRAM_BASE;
  GPIOB->BSRR |= GPIO_PIN_15;

  fu_init_custom_vtable();
  SCB->VTOR = FU_SRAM_BASE;


  //set msp
  __ASM volatile ("MSR msp, %0\n" : : "r" (FU_ESTACK) );
  //enable irq
  __ASM volatile ("cpsie i" : : : "memory");

  TIM4->CR1 |= TIM_CR1_CEN;

  while(1){
  }
}


