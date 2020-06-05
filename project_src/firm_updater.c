#include "firm_updater.h"

#include "stm32f1xx_hal.h"

#define VECT_CNT  128U
#define FU_ESTACK  0x2000FFFFU
#define FU_SRAM_BASE  0x20000000U

#define FU_FLASH_BASE 0x08000000U

#define STM32F107_LAST_PAGE_ADDR  0x0803F800U
#define STM32F107_PAGE_SIZE       0x800U

/* Delay definition */
#define erase_timeout    0x000B0000U
#define program_timeout  0x00002000U

typedef enum
{
  FLASH_BUSY = 1,
  FLASH_ERROR_PG,
  FLASH_ERROR_WRP,
  FLASH_COMPLETE,
  FLASH_TIMEOUT
} flash_status_t;


uint32_t int_vector_table[VECT_CNT] __attribute__((section(".pashahod_section"), aligned(1024)));

extern update_info_t uinfo;

static uint8_t replace_core(void);


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

  if (set_pin) {
    GPIOB->BSRR = GPIO_PIN_14;
  } else {
    GPIOB->BSRR = (uint32_t)GPIO_PIN_14 << 16U;
  }

  TIM4->CR1 |= TIM_CR1_CEN;
  __ASM volatile ("bx lr" : : : "memory"); //??
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
  uint8_t ret = 0;

  GPIOB->BSRR = (uint32_t)GPIO_PIN_0 << 16U;

  fu_init_custom_vtable();
  SCB->VTOR = FU_SRAM_BASE;

  ret = replace_core();
  switch(ret) {
    case 0:
      GPIOB->BSRR = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_14 | GPIO_PIN_15;
      break;
    case 1:
      GPIOB->BSRR = GPIO_PIN_15;
      break;
    case 2:
      GPIOB->BSRR = GPIO_PIN_14;
      break;
    case 3:
      GPIOB->BSRR = GPIO_PIN_14 | GPIO_PIN_15;
      break;
    default:
      GPIOB->BSRR = GPIO_PIN_0 | GPIO_PIN_14 | GPIO_PIN_15;
      break;
  }
  //set msp
  //__ASM volatile ("MSR msp, %0\n" : : "r" (FU_ESTACK) );
  //enable irq
  //__ASM volatile ("cpsie i" : : : "memory");

  //TIM4->CR1 |= TIM_CR1_CEN;

  while(1){
  }
}


static flash_status_t fu_flash_getstatus(void)
{

  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {
    return FLASH_BUSY;
  }

  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGERR)) {
    return FLASH_ERROR_PG;
  }

  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR)) {
    return FLASH_ERROR_WRP;
 }

  /* Return the Flash Status */
  return FLASH_COMPLETE;
}

static void err_handler(uint8_t ret)
{
  switch(ret) {
    case 1:
      GPIOB->BSRR = GPIO_PIN_15;
      break;
    case 2:
      GPIOB->BSRR = GPIO_PIN_14;
      break;
    case 3:
      GPIOB->BSRR = GPIO_PIN_14 | GPIO_PIN_15;
      break;
    default:
      GPIOB->BSRR = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_14 | GPIO_PIN_15;
      break;
  }

  while(1) {}
}

static flash_status_t fu_flash_waitforlastoperation(uint32_t timeout)
{
  flash_status_t status = FLASH_COMPLETE;

  /* Check for the Flash Status */
  status = fu_flash_getstatus();
  /* Wait for a Flash operation to complete or a timeout to occur */
  while((status == FLASH_BUSY) && (timeout != 0x00))
  {
    status = fu_flash_getstatus();
    //timeout--;
  }
  if(timeout == 0x00)
  {
    status = FLASH_TIMEOUT;
  }
  /* Return the operation status */
  return status;
}

/**
  * @brief  Erases a specified FLASH page.
  * @note   This function can be used for all STM32F10x devices.
  * @param  page_addr: The page address to be erased.
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
static uint8_t fu_flash_erase_page(uint32_t page_addr)
{
  flash_status_t status = FLASH_COMPLETE;
   /* Wait for last operation to be completed */
  status = fu_flash_waitforlastoperation(erase_timeout);
  if (status != FLASH_COMPLETE) {
    return 1;

    // if (status == FLASH_ERROR_WRP) {
    //   err_handler(2);
    // }

    // if (status == FLASH_ERROR_PG) {
    //   err_handler(1);
    // }

    // if (status == FLASH_BUSY) {
    //   err_handler(3);
    // }
    // while(1){}
  }

/* if the previous operation is completed, proceed to erase the page */
  FLASH->CR|= FLASH_CR_PER;
  FLASH->AR = page_addr;
  FLASH->CR|= FLASH_CR_STRT;

  /* Wait for last operation to be completed */
  status = fu_flash_waitforlastoperation(erase_timeout);
  if (status != FLASH_COMPLETE) {
    return 2;
    //err_handler(3);
  }
  /* Disable the PER Bit */
  FLASH->CR &= ~FLASH_CR_PER;
  return 0;
}


static uint8_t fu_erase(uint32_t start_from_addr, uint32_t erase_untill_addr){

  flash_status_t st;
  uint32_t addr = start_from_addr;
  uint32_t pages_amount = 0;

  if (start_from_addr % STM32F107_PAGE_SIZE) { //adress is not aligned
    return 1;
  }

  if(erase_untill_addr > STM32F107_LAST_PAGE_ADDR){
    return 2;
  }

  pages_amount = (erase_untill_addr - start_from_addr) / STM32F107_PAGE_SIZE;

  for(uint8_t i = 0; i < pages_amount; i++){
    st = fu_flash_erase_page(addr);
    if(st){
       return 3;
    }
    addr += STM32F107_PAGE_SIZE;
  }
  return 0;

}



static uint8_t fu_flash_program_halfword(uint32_t Address, uint16_t Data)
{
  flash_status_t status = FLASH_COMPLETE;
  status = fu_flash_waitforlastoperation(program_timeout);
  if (status != FLASH_COMPLETE) {
    return 1;
  }

    SET_BIT(FLASH->CR, FLASH_CR_PG);
  /* Write data in the address */
  *(__IO uint16_t*)Address = Data;

  status = fu_flash_waitforlastoperation(program_timeout);
  if (status != FLASH_COMPLETE) {
    return 2;
  }

  return 0;
}

static uint8_t fu_flash_unlock(void)
{

  if(READ_BIT(FLASH->CR, FLASH_CR_LOCK) != RESET)
  {
    /* Authorize the FLASH Registers access */
    WRITE_REG(FLASH->KEYR, FLASH_KEY1);
    WRITE_REG(FLASH->KEYR, FLASH_KEY2);

    /* Verify Flash is unlocked */
    if(READ_BIT(FLASH->CR, FLASH_CR_LOCK) != RESET)
    {
      return 1;
    }
  }

  return 0;

}

static uint8_t fu_flash_lock(void)
{
  /* Set the LOCK Bit to lock the FLASH Registers access */
  SET_BIT(FLASH->CR, FLASH_CR_LOCK);

  return 0;
}

static uint8_t replace_core(void)
{

  uint32_t start_addr = uinfo.start_addr;
  uint32_t bytelen = uinfo.bytelen;

  uint32_t addr_cpy = start_addr;
  uint32_t addr_cpy_to = FU_FLASH_BASE;

  uint16_t carrier_hword=0;
  uint8_t ret = 0;

  fu_flash_unlock();

  //erase old core
  ret = fu_erase(FU_FLASH_BASE, start_addr);
  if(ret){
    return 1;
  }

  //copy new one
  for(int i = 0; i < bytelen; i++) {
    carrier_hword = *(uint16_t *)addr_cpy;
    ret = fu_flash_program_halfword(addr_cpy_to, carrier_hword);
    if (ret) {
      return 2;
    }
    addr_cpy += 2;
    addr_cpy_to += 2;
  }

  CLEAR_BIT(FLASH->CR, FLASH_CR_PG);

  //erase storage
  ret = fu_erase(start_addr, STM32F107_LAST_PAGE_ADDR);
  if(ret){
    return 3;
  }

  fu_flash_lock();

  return 0;
}
