#include "board.h"
#include "delay.h"
#include "timebase.h"
#include "camera_uart.h"
#include "ball_control.h"

/**
  * @brief  Ball-on-beam controller entry point.
  *
  * USART1 (PA9/PA10) controls the Emm_V5 motor driver.
  * USART3 (PB10/PB11) receives 12-byte vision packets from K230 UART4.
  */
int main(void)
{
	CameraPacket_t camera_packet;
	uint32_t now_ms;

	/* Initialize USART1 and the existing motor-driver receive path. */
	board_init();

	/* Wait for the motor driver, enable it, declare the manually levelled beam
	 * as zero, then use the ZDT\1-compatible 0xFD absolute-position command. */
	delay_ms(500);
	ball_control_init();

	/* delay_ms() owns SysTick, therefore start the permanent time base only
	 * after all startup delays. USART3 starts last so its timestamps are valid. */
	timebase_init();
	camera_uart_init();

	while(1)
	{
		if(camera_uart_get_latest(&camera_packet))
		{
			/* Sample time after copying the packet so now_ms can never be
			 * earlier than the ISR timestamp of that same packet. */
			now_ms = timebase_millis();
			ball_control_handle_camera_packet(&camera_packet, now_ms);
		}

		ball_control_service(timebase_millis());

		/* SysTick and either USART wake the CPU. PID and motor work stays in
		 * thread mode rather than extending an interrupt service routine. */
		__WFI();
	}
}
