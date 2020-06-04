#include "firm_updater.h"

#include <stdint.h>

#include "stm32f1xx_hal.h"

void fu_main(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1 | GPIO_PIN_14, GPIO_PIN_SET);
  while(1){}
}
