#ifndef __BALL_CONTROL_H
#define __BALL_CONTROL_H

#include "stm32f10x.h"
#include "camera_uart.h"

typedef enum {
	BALL_CONTROL_WAITING_CAMERA = 0,
	BALL_CONTROL_ARMING,
	BALL_CONTROL_ACTIVE,
	BALL_CONTROL_PREDICTED,
	BALL_CONTROL_EDGE_RECOVERY,
	BALL_CONTROL_BALL_INVALID,
	BALL_CONTROL_CAMERA_TIMEOUT,
	BALL_CONTROL_MOTOR_FAULT
} BallControlState_t;

/* Exposed for the Keil Watch window during commissioning. */
typedef struct {
	BallControlState_t state;
	uint8_t camera_sequence;
	uint8_t camera_flags;
	uint8_t fresh_packet_streak;
	float requested_setpoint_mm;
	float filtered_setpoint_mm;
	float ball_position_mm;
	float ball_velocity_mm_s;
	float error_mm;
	float integral_output_deg;
	float output_angle_deg;
	int32_t desired_motor_pulses;
	int32_t sent_motor_pulses;
	uint32_t last_packet_age_ms;
	uint32_t accepted_packets;
	uint32_t rejected_packets;
	uint8_t motor_link_ready;
	uint8_t motor_last_function;
	uint32_t motor_commands_sent;
} BallControlStatus_t;

extern __IO BallControlStatus_t ball_control_status;

/* Uses the proven ZDT\1 startup sequence and legacy delay_ms(); call before
 * timebase_init(). The current beam pose is declared as absolute position 0. */
void ball_control_init(void);

/* Called when camera_uart_get_latest() returns a new packet. */
void ball_control_handle_camera_packet(const CameraPacket_t *packet,
	                                    uint32_t now_ms);

/* Call continuously from the main loop. Handles timeout and motor slew. */
void ball_control_service(uint32_t now_ms);

#endif
