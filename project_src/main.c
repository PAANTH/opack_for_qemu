
#include "inttypes.h"
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
  __HAL_RCC_TIM4_CLK_ENABLE();

//LEDS
  gpio_init_struct.Pin       = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_14 | GPIO_PIN_15;
  gpio_init_struct.Mode      = GPIO_MODE_OUTPUT_PP ;
  gpio_init_struct.Speed     = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio_init_struct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7200;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  NVIC_EnableIRQ(TIM4_IRQn);
  __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE);
  HAL_TIM_Base_Init(&htim4);
  HAL_TIM_Base_Start(&htim4);

}

int main (void)
{
   uint32_t start = 0;
   uint32_t curr = 0;

  __enable_irq();
  config_sysclk();
  HAL_Init();
  config_hw();

  start = HAL_GetTick();
  while(1) {
    curr = HAL_GetTick();
    if ((curr - start) > 500) {
      start = curr;
      //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_14 | GPIO_PIN_15);
    }

  }
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}


void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim4);
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_14 | GPIO_PIN_15);
  HAL_TIM_Base_Start(&htim4);
}
