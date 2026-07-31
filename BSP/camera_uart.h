#ifndef __CAMERA_UART_H
#define __CAMERA_UART_H

#include "stm32f10x.h"
#include <stdbool.h>

#define CAMERA_PACKET_LENGTH          12U

/* Keep these bits synchronized with demo-camera(3).py. */
#define CAMERA_FLAG_BALL_VALID        0x01U
#define CAMERA_FLAG_SETPOINT_VALID    0x02U
#define CAMERA_FLAG_TRACK_FRESH       0x04U
#define CAMERA_FLAG_MODE2_DONE        0x08U
#define CAMERA_FLAG_MODE_MASK         0x30U
#define CAMERA_FLAG_BALL_FRESH        0x40U
#define CAMERA_FLAG_EDGE_RECOVERY     0x80U

typedef struct {
	uint8_t sequence;
	uint8_t flags;
	uint8_t confidence_percent;
	int16_t position_x10;
	int16_t velocity_x10;
	int16_t setpoint_x10;
	uint32_t received_ms;
} CameraPacket_t;

/* PB10 = USART3_TX, PB11 = USART3_RX, 115200 8N1. */
void camera_uart_init(void);

/* Returns true when a newer valid packet is available. If several packets
 * arrive before it is called, only the newest packet is returned. */
bool camera_uart_get_latest(CameraPacket_t *packet);

uint32_t camera_uart_get_valid_count(void);
uint32_t camera_uart_get_error_count(void);

#endif
