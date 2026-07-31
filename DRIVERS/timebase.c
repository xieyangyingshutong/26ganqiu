#include "timebase.h"

static __IO uint32_t timebase_ms = 0;

/**
  * @brief  Start a 1 kHz SysTick time base.
  * @note   Call this after the legacy delay_ms() startup delays have finished,
  *         because delay_ms() temporarily owns the SysTick peripheral.
  */
void timebase_init(void)
{
	timebase_ms = 0U;
	if(SysTick_Config(SystemCoreClock / 1000U) != 0U)
	{
		/* The 24-bit reload value cannot represent the requested tick. */
		while(1)
		{
		}
	}
}

/**
  * @brief  Advance the system time. Called only by SysTick_Handler().
  */
void timebase_tick_isr(void)
{
	++timebase_ms;
}

/**
  * @brief  Return elapsed milliseconds since timebase_init().
  */
uint32_t timebase_millis(void)
{
	return timebase_ms;
}
