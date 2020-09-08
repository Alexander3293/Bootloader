/*
 * hal_func_redef.c - ���������������� ������� HAL.
 * ��������� � ��������� '.c'-����� ��� ��������� ������� � '.cpp'-������������.
 */

#include "stm32f4xx_hal.h"

extern __IO uint32_t uwTick;

/**
  * @brief ���������������� ������� ��� HAL-��������.
  * ��������� ��������� Systick � ������, ���� �� ��� �������� �����.
  * ����������, �.�. Systick ����� ���������� ��������� �� ����� � ���\���� �����.
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
  * ���������������� ������� - ��������� ��������� Systick � ������, ���� �� ��� �������� �����.
  * @retval tick value
  */
uint32_t HAL_GetTick(void)
{
	/* Start systick if disabled */
	if(!(SysTick->CTRL & SysTick_CTRL_ENABLE_Msk))
		SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk);

	return uwTick;
}
