
#include <stdint.h>
#include <string.h>
#include "stm32f1xx_hal.h"


/*!
  \brief config_sysclk
        Функция настраивает системное тактирование на 168MHz.
  \return Код ошибки.
*/
uint32_t config_sysclk(void)
{
  RCC_ClkInitTypeDef rcc_clk_init_str;
  RCC_OscInitTypeDef rcc_osc_init_str;
  __HAL_RCC_PWR_CLK_ENABLE();

  rcc_osc_init_str.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  rcc_osc_init_str.Prediv1Source  = RCC_PREDIV1_SOURCE_PLL2;
  rcc_osc_init_str.HSEState       = RCC_HSE_ON;
  rcc_osc_init_str.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  rcc_osc_init_str.PLL.PLLState   = RCC_PLL_ON;
  rcc_osc_init_str.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  rcc_osc_init_str.PLL.PLLMUL     = RCC_PLL_MUL9;
  rcc_osc_init_str.PLL2.PLL2State = RCC_PLL2_ON;
  rcc_osc_init_str.PLL2.PLL2MUL   = RCC_PLL2_MUL8;
  rcc_osc_init_str.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;

  if (HAL_RCC_OscConfig(&rcc_osc_init_str) != HAL_OK) {
    return 1;
  }

  rcc_clk_init_str.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  rcc_clk_init_str.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  rcc_clk_init_str.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  rcc_clk_init_str.APB1CLKDivider = RCC_HCLK_DIV2;
  rcc_clk_init_str.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&rcc_clk_init_str, FLASH_LATENCY_2) != HAL_OK) {
    return 2;
  }
  return 0;
}

TIM_HandleTypeDef htim4;
void config_hw(void)
{
  GPIO_InitTypeDef  gpio_init_struct;

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();

//LED0
  gpio_init_struct.Pin       = GPIO_PIN_0 | GPIO_PIN_15;
  gpio_init_struct.Mode      = GPIO_MODE_OUTPUT_PP ;
  gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio_init_struct);

//button
  gpio_init_struct.Pin       = GPIO_PIN_13;
  gpio_init_struct.Mode      = GPIO_MODE_INPUT;
  gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;
  gpio_init_struct.Pull     = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &gpio_init_struct);

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7200;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 5000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  NVIC_EnableIRQ(TIM4_IRQn);
  __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
  HAL_TIM_Base_Init(&htim4);

}


void jump_to_sram(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __disable_irq();
  memcpy((uint8_t *)0x20000000, (uint8_t *)0x08020000, 7180);
  void *reset_handler_addr = (void*)(*(uint32_t*)0x20000004);
  void (*reset_handler)() = (void(*)())reset_handler_addr;
  reset_handler();
}


int main (void)
{
  GPIO_PinState button_pressed = GPIO_PIN_SET;

  HAL_Init();

  config_sysclk();
  config_hw();

  HAL_TIM_Base_Start(&htim4);
  while(1) {
    button_pressed = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
    if (button_pressed == GPIO_PIN_RESET) {
      HAL_TIM_Base_Stop(&htim4);
      jump_to_sram();
    }

  }
}

void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim4);
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_15);
  HAL_TIM_Base_Start(&htim4);
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

// Фатальная ошибка теста внутренней оперативной памяти
void Error_RAM_Test()
{


  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  //do {
   (void)(RCC->APB2ENR & RCC_APB2ENR_IOPBEN); // Delay after an RCC peripheral clock enabling
   __NOP();
   __NOP();
   __NOP();
  //} while (0);
  // // Config LED
  GPIOB->CRH &= ~GPIO_CRH_MODE15_Msk;
  GPIOB->CRH |= (1U << GPIO_CRH_MODE15_Pos);
  GPIOB->CRH &= ~GPIO_CRH_CNF15_Msk;

  //Light LED
  GPIOB->BSRR |= GPIO_PIN_15;

  __disable_irq();


  // Останов
Endless:
  goto Endless;
  __NOP();
}




