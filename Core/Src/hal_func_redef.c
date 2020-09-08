/*
 * hal_func_redef.c - переопределенные функции HAL.
 * Объявлены в отдельном '.c'-файле для избежания проблем с '.cpp'-компилятором.
 */

#include "stm32f4xx_hal.h"

extern __IO uint32_t uwTick;

/**
  * @brief Переопределенная функция для HAL-задержек.
  * Добавлено включение Systick в случае, если он был выключен ранее.
  * Необходимо, т.к. Systick можно программно отключать до входа в сон\стоп режим.
  * @param Delay: specifies the delay time length, in milliseconds.
  */
void HAL_Delay(uint32_t Delay)
{
	uint32_t tickstart = HAL_GetTick();
	uint32_t wait = Delay;

	/* Add a period to guaranty minimum wait */
	if(wait < HAL_MAX_DELAY)
		wait++;

	/* Start systick if disabled */
	if(!(SysTick->CTRL & SysTick_CTRL_ENABLE_Msk))
		SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk);

	while((HAL_GetTick() - tickstart) < wait)
	{
		__asm("NOP");
	}
}

/**
  * @brief Provide a tick value in millisecond.
  * Переопределенная функция - добавлено включение Systick в случае, если он был выключен ранее.
  * @retval tick value
  */
uint32_t HAL_GetTick(void)
{
	/* Start systick if disabled */
	if(!(SysTick->CTRL & SysTick_CTRL_ENABLE_Msk))
		SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk);

	return uwTick;
}
