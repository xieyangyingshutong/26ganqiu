#include "camera_uart.h"
#include "timebase.h"

#define CAMERA_HEADER_0               0xAAU
#define CAMERA_HEADER_1               0x55U

static uint8_t camera_rx_buffer[CAMERA_PACKET_LENGTH] = {0};
static uint8_t camera_rx_index = 0;

static __IO CameraPacket_t camera_latest_packet = {0};
static __IO uint32_t camera_packet_generation = 0;
static uint32_t camera_consumed_generation = 0;
static __IO uint32_t camera_valid_count = 0;
static __IO uint32_t camera_error_count = 0;

static int16_t camera_read_i16_le(const uint8_t *data)
{
	uint16_t value;

	value = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
	return (int16_t)value;
}

static bool camera_packet_semantics_valid(const uint8_t *raw)
{
	uint8_t mode;
	int16_t position_x10;
	int16_t setpoint_x10;

	mode = (uint8_t)((raw[3] & CAMERA_FLAG_MODE_MASK) >> 4);
	position_x10 = camera_read_i16_le(&raw[5]);
	setpoint_x10 = camera_read_i16_le(&raw[9]);

	if((mode < 1U) || (mode > 3U) || (raw[4] > 100U))
	{
		return false;
	}

	if(((raw[3] & CAMERA_FLAG_BALL_VALID) != 0U) &&
	   ((position_x10 < -1200) || (position_x10 > 1200)))
	{
		return false;
	}

	if(((raw[3] & CAMERA_FLAG_SETPOINT_VALID) != 0U) &&
	   ((setpoint_x10 < -1200) || (setpoint_x10 > 1200)))
	{
		return false;
	}

	return true;
}

static void camera_publish_packet(const uint8_t *raw)
{
	/* Odd generation means a write is in progress; even means stable. */
	++camera_packet_generation;
	camera_latest_packet.sequence = raw[2];
	camera_latest_packet.flags = raw[3];
	camera_latest_packet.confidence_percent = raw[4];
	camera_latest_packet.position_x10 = camera_read_i16_le(&raw[5]);
	camera_latest_packet.velocity_x10 = camera_read_i16_le(&raw[7]);
	camera_latest_packet.setpoint_x10 = camera_read_i16_le(&raw[9]);
	camera_latest_packet.received_ms = timebase_millis();
	++camera_valid_count;
	++camera_packet_generation;
}

static void camera_resynchronize(void)
{
	uint8_t start;
	uint8_t remaining;
	uint8_t i;

	/* Search the failed frame for another complete AA 55 prefix. */
	for(start = 1U; start < (CAMERA_PACKET_LENGTH - 1U); ++start)
	{
		if((camera_rx_buffer[start] == CAMERA_HEADER_0) &&
		   (camera_rx_buffer[start + 1U] == CAMERA_HEADER_1))
		{
			remaining = (uint8_t)(CAMERA_PACKET_LENGTH - start);
			for(i = 0U; i < remaining; ++i)
			{
				camera_rx_buffer[i] = camera_rx_buffer[start + i];
			}
			camera_rx_index = remaining;
			return;
		}
	}

	if(camera_rx_buffer[CAMERA_PACKET_LENGTH - 1U] == CAMERA_HEADER_0)
	{
		camera_rx_buffer[0] = CAMERA_HEADER_0;
		camera_rx_index = 1U;
	}
	else
	{
		camera_rx_index = 0U;
	}
}

static void camera_parse_byte(uint8_t data)
{
	uint8_t checksum;
	uint8_t i;

	if(camera_rx_index == 0U)
	{
		if(data == CAMERA_HEADER_0)
		{
			camera_rx_buffer[0] = data;
			camera_rx_index = 1U;
		}
		return;
	}

	if(camera_rx_index == 1U)
	{
		if(data == CAMERA_HEADER_1)
		{
			camera_rx_buffer[1] = data;
			camera_rx_index = 2U;
		}
		else if(data == CAMERA_HEADER_0)
		{
			/* The new byte may itself be the first header byte. */
			camera_rx_buffer[0] = data;
			camera_rx_index = 1U;
		}
		else
		{
			camera_rx_index = 0U;
		}
		return;
	}

	camera_rx_buffer[camera_rx_index] = data;
	++camera_rx_index;

	if(camera_rx_index < CAMERA_PACKET_LENGTH)
	{
		return;
	}

	checksum = 0U;
	for(i = 0U; i < (CAMERA_PACKET_LENGTH - 1U); ++i)
	{
		checksum ^= camera_rx_buffer[i];
	}

	if((checksum == camera_rx_buffer[CAMERA_PACKET_LENGTH - 1U]) &&
	   camera_packet_semantics_valid(camera_rx_buffer))
	{
		camera_publish_packet(camera_rx_buffer);
		camera_rx_index = 0U;
	}
	else
	{
		++camera_error_count;
		camera_resynchronize();
	}
}

void camera_uart_init(void)
{
	GPIO_InitTypeDef gpio_init;
	USART_InitTypeDef usart_init;
	NVIC_InitTypeDef nvic_init;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	NVIC_DisableIRQ(USART3_IRQn);
	USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
	USART_Cmd(USART3, DISABLE);

	/* PB10 - USART3_TX (available for future diagnostics to K230). */
	gpio_init.GPIO_Pin = GPIO_Pin_10;
	gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
	gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &gpio_init);

	/* PB11 - USART3_RX from K230 GPIO36/UART4_TXD. */
	gpio_init.GPIO_Pin = GPIO_Pin_11;
	gpio_init.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &gpio_init);

	usart_init.USART_BaudRate = 115200;
	usart_init.USART_WordLength = USART_WordLength_8b;
	usart_init.USART_StopBits = USART_StopBits_1;
	usart_init.USART_Parity = USART_Parity_No;
	usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart_init.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART3, &usart_init);

	camera_rx_index = 0U;
	camera_packet_generation = 0U;
	camera_consumed_generation = 0U;
	camera_valid_count = 0U;
	camera_error_count = 0U;

	/* Reading SR then DR clears any stale RXNE/ORE condition. */
	(void)USART3->SR;
	(void)USART3->DR;
	USART_Cmd(USART3, ENABLE);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	nvic_init.NVIC_IRQChannel = USART3_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority = 2;
	nvic_init.NVIC_IRQChannelSubPriority = 0;
	nvic_init.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic_init);
}

bool camera_uart_get_latest(CameraPacket_t *packet)
{
	uint32_t generation_before;
	uint32_t generation_after;

	if(packet == 0)
	{
		return false;
	}

	while(1)
	{
		generation_before = camera_packet_generation;
		if(generation_before == camera_consumed_generation)
		{
			return false;
		}

		if((generation_before & 1U) != 0U)
		{
			continue;
		}

		packet->sequence = camera_latest_packet.sequence;
		packet->flags = camera_latest_packet.flags;
		packet->confidence_percent = camera_latest_packet.confidence_percent;
		packet->position_x10 = camera_latest_packet.position_x10;
		packet->velocity_x10 = camera_latest_packet.velocity_x10;
		packet->setpoint_x10 = camera_latest_packet.setpoint_x10;
		packet->received_ms = camera_latest_packet.received_ms;
		generation_after = camera_packet_generation;

		if((generation_before == generation_after) &&
		   ((generation_after & 1U) == 0U))
		{
			break;
		}
	}

	camera_consumed_generation = generation_after;
	return true;
}

uint32_t camera_uart_get_valid_count(void)
{
	return camera_valid_count;
}

uint32_t camera_uart_get_error_count(void)
{
	return camera_error_count;
}

void USART3_IRQHandler(void)
{
	uint16_t status;
	uint8_t data;

	status = USART3->SR;

	if((status & (USART_FLAG_ORE | USART_FLAG_NE |
	              USART_FLAG_FE | USART_FLAG_PE)) != 0U)
	{
		/* SR was read above; reading DR now clears the receive error. */
		data = (uint8_t)USART3->DR;
		(void)data;
		camera_rx_index = 0U;
		++camera_error_count;
		return;
	}

	if((status & USART_FLAG_RXNE) != 0U)
	{
		data = (uint8_t)(USART3->DR & 0x00FFU);
		camera_parse_byte(data);
	}
}
