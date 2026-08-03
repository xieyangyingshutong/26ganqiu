#include "ball_control.h"
#include "ball_control_config.h"
#include "Emm_V5.h"
#include "delay.h"

#define BALL_EMM_FUNCTION_POSITION            0xFDU

__IO BallControlStatus_t ball_control_status;

static float pid_integral_output_deg = 0.0f;
static float filtered_setpoint_mm = 0.0f;
static float stiction_boost_deg = 0.0f;
static int32_t desired_motor_pulses = BALL_MOTOR_LEVEL_PULSES;
static int32_t sent_motor_pulses = BALL_MOTOR_LEVEL_PULSES;

static uint32_t last_camera_packet_ms = 0U;
static uint32_t last_ball_fresh_ms = 0U;
static uint32_t last_pid_update_ms = 0U;
static uint32_t last_motor_service_ms = 0U;
static uint32_t last_motor_send_ms = 0U;
static uint32_t stiction_candidate_since_ms = 0U;

static uint8_t fresh_packet_streak = 0U;
static bool camera_packet_seen = false;
static bool camera_sequence_seen = false;
static bool controller_armed = false;
static bool setpoint_filter_initialized = false;
static bool motor_link_ready = false;
static uint8_t last_camera_sequence = 0U;
static int8_t stiction_direction = 0;

static float control_absf(float value)
{
	return (value >= 0.0f) ? value : -value;
}

static float control_clampf(float value, float minimum, float maximum)
{
	if(value < minimum)
	{
		return minimum;
	}
	if(value > maximum)
	{
		return maximum;
	}
	return value;
}

static int32_t control_clamp_i32(int32_t value, int32_t minimum,
	                             int32_t maximum)
{
	if(value < minimum)
	{
		return minimum;
	}
	if(value > maximum)
	{
		return maximum;
	}
	return value;
}

static int32_t control_round_to_i32(float value)
{
	if(value >= 0.0f)
	{
		return (int32_t)(value + 0.5f);
	}
	return (int32_t)(value - 0.5f);
}

static void ball_control_get_output_limits(float requested_limit,
	                                       float *lower_limit,
	                                       float *upper_limit)
{
	float pulse_scale;
	float angle_at_minimum;
	float angle_at_maximum;
	float mechanical_lower;
	float mechanical_upper;

	pulse_scale = BALL_MOTOR_PULSES_PER_BEAM_DEGREE *
	              BALL_MOTOR_DIRECTION_SIGN;
	if(control_absf(pulse_scale) < 0.001f)
	{
		*lower_limit = 0.0f;
		*upper_limit = 0.0f;
		return;
	}

	angle_at_minimum = ((float)BALL_MOTOR_MIN_PULSES -
	                    (float)BALL_MOTOR_LEVEL_PULSES) / pulse_scale;
	angle_at_maximum = ((float)BALL_MOTOR_MAX_PULSES -
	                    (float)BALL_MOTOR_LEVEL_PULSES) / pulse_scale;

	if(angle_at_minimum < angle_at_maximum)
	{
		mechanical_lower = angle_at_minimum;
		mechanical_upper = angle_at_maximum;
	}
	else
	{
		mechanical_lower = angle_at_maximum;
		mechanical_upper = angle_at_minimum;
	}

	*lower_limit = control_clampf(-requested_limit,
	                              mechanical_lower, mechanical_upper);
	*upper_limit = control_clampf(requested_limit,
	                              mechanical_lower, mechanical_upper);
	if(*lower_limit > *upper_limit)
	{
		*lower_limit = 0.0f;
		*upper_limit = 0.0f;
	}
}

static void ball_control_send_absolute_position(int32_t target_pulses)
{
	uint8_t direction;
	uint32_t magnitude;

	target_pulses = control_clamp_i32(target_pulses,
	                                  BALL_MOTOR_MIN_PULSES,
	                                  BALL_MOTOR_MAX_PULSES);
	if(target_pulses >= 0)
	{
		direction = BALL_MOTOR_POSITIVE_IS_CW ? 0U : 1U;
		magnitude = (uint32_t)target_pulses;
	}
	else
	{
		direction = BALL_MOTOR_POSITIVE_IS_CW ? 1U : 0U;
		magnitude = (uint32_t)(-target_pulses);
	}

	/* ZDT\1 uses the widely-supported 0xFD ordinary absolute-position
	 * command. It does not require F1/FC firmware support or a PA10 ACK. */
	Emm_V5_Pos_Control(BALL_MOTOR_ADDRESS,
	                   direction,
	                   BALL_MOTOR_POSITION_SPEED_RPM,
	                   BALL_MOTOR_POSITION_ACCELERATION,
	                   magnitude,
	                   1U,
	                   false);
	sent_motor_pulses = target_pulses;
	ball_control_status.sent_motor_pulses = sent_motor_pulses;
	ball_control_status.motor_last_function = BALL_EMM_FUNCTION_POSITION;
	++ball_control_status.motor_commands_sent;
}

static void ball_control_reset_transient_state(void)
{
	last_pid_update_ms = 0U;
	setpoint_filter_initialized = false;
	stiction_boost_deg = 0.0f;
	stiction_candidate_since_ms = 0U;
	stiction_direction = 0;
	/* Keep the integral output: it is the learned neutral-angle correction
	 * for tube slope, crank preload and static mechanical bias. */
	ball_control_status.integral_output_deg = pid_integral_output_deg;
	ball_control_status.stiction_boost_deg = 0.0f;
	ball_control_status.stiction_time_ms = 0U;
	ball_control_status.output_angle_deg = 0.0f;
	ball_control_status.error_mm = 0.0f;
}

static void ball_control_enter_safe_state(BallControlState_t state)
{
	if(!motor_link_ready)
	{
		ball_control_status.state = BALL_CONTROL_MOTOR_FAULT;
		return;
	}

	if(controller_armed)
	{
		ball_control_reset_transient_state();
	}

	controller_armed = false;
	fresh_packet_streak = 0U;
	desired_motor_pulses = BALL_MOTOR_LEVEL_PULSES;
	ball_control_status.state = state;
	ball_control_status.fresh_packet_streak = 0U;
	ball_control_status.desired_motor_pulses = BALL_MOTOR_LEVEL_PULSES;
}

static float ball_control_slew(float current, float target,
	                           float rate_per_second, float dt_seconds)
{
	float maximum_change;
	float difference;

	maximum_change = rate_per_second * dt_seconds;
	difference = target - current;
	if(difference > maximum_change)
	{
		difference = maximum_change;
	}
	else if(difference < -maximum_change)
	{
		difference = -maximum_change;
	}
	return current + difference;
}

static void ball_control_update_stiction(bool ball_fresh,
	                                     bool edge_recovery,
	                                     float error_mm,
	                                     float velocity_mm_s,
	                                     float dt_seconds,
	                                     uint32_t now_ms)
{
	int8_t requested_direction;
	uint32_t stuck_time_ms;
	float requested_boost;

	/* A predicted position cannot prove that the ball is stationary. Hold the
	 * current decision for a short prediction; the normal safety timeout will
	 * reset it if fresh measurements do not return. */
	if(!ball_fresh)
	{
		return;
	}

	if(edge_recovery ||
	   (control_absf(error_mm) < BALL_STICTION_ERROR_THRESHOLD_MM) ||
	   (control_absf(velocity_mm_s) >
	    BALL_STICTION_VELOCITY_THRESHOLD_MM_S))
	{
		stiction_candidate_since_ms = 0U;
		stiction_direction = 0;
		stiction_boost_deg = ball_control_slew(
			stiction_boost_deg, 0.0f,
			BALL_STICTION_BOOST_FALL_DEG_PER_S, dt_seconds);
		ball_control_status.stiction_boost_deg = stiction_boost_deg;
		ball_control_status.stiction_time_ms = 0U;
		return;
	}

	requested_direction = (error_mm > 0.0f) ? 1 : -1;
	if(stiction_direction != requested_direction)
	{
		stiction_direction = requested_direction;
		stiction_candidate_since_ms = now_ms;
		stiction_boost_deg = 0.0f;
	}

	stuck_time_ms = (uint32_t)(now_ms - stiction_candidate_since_ms);
	if(stuck_time_ms >= BALL_STICTION_CONFIRM_MS)
	{
		requested_boost = (float)stiction_direction *
		                  BALL_STICTION_MAX_BOOST_DEG;
		stiction_boost_deg = ball_control_slew(
			stiction_boost_deg, requested_boost,
			BALL_STICTION_BOOST_RISE_DEG_PER_S, dt_seconds);
	}
	else
	{
		stiction_boost_deg = ball_control_slew(
			stiction_boost_deg, 0.0f,
			BALL_STICTION_BOOST_FALL_DEG_PER_S, dt_seconds);
	}

	ball_control_status.stiction_boost_deg = stiction_boost_deg;
	ball_control_status.stiction_time_ms = stuck_time_ms;
}

static bool ball_control_packet_values_valid(const CameraPacket_t *packet)
{
	if(((packet->flags & CAMERA_FLAG_BALL_VALID) == 0U) ||
	   ((packet->flags & CAMERA_FLAG_SETPOINT_VALID) == 0U) ||
	   (packet->confidence_percent < BALL_CAMERA_MIN_CONFIDENCE_PERCENT))
	{
		return false;
	}

	if((packet->position_x10 < BALL_POSITION_MIN_X10) ||
	   (packet->position_x10 > BALL_POSITION_MAX_X10))
	{
		return false;
	}

	return true;
}

void ball_control_init(void)
{
	ball_control_status.state = BALL_CONTROL_WAITING_CAMERA;
	ball_control_status.camera_sequence = 0U;
	ball_control_status.camera_flags = 0U;
	ball_control_status.fresh_packet_streak = 0U;
	ball_control_status.requested_setpoint_mm = 0.0f;
	ball_control_status.filtered_setpoint_mm = 0.0f;
	ball_control_status.ball_position_mm = 0.0f;
	ball_control_status.ball_velocity_mm_s = 0.0f;
	ball_control_status.error_mm = 0.0f;
	ball_control_status.integral_output_deg = 0.0f;
	ball_control_status.stiction_boost_deg = 0.0f;
	ball_control_status.stiction_time_ms = 0U;
	ball_control_status.output_angle_deg = 0.0f;
	ball_control_status.desired_motor_pulses = BALL_MOTOR_LEVEL_PULSES;
	ball_control_status.sent_motor_pulses = BALL_MOTOR_LEVEL_PULSES;
	ball_control_status.last_packet_age_ms = 0U;
	ball_control_status.accepted_packets = 0U;
	ball_control_status.rejected_packets = 0U;
	ball_control_status.motor_link_ready = 0U;
	ball_control_status.motor_last_function = 0U;
	ball_control_status.motor_commands_sent = 0U;

	pid_integral_output_deg = 0.0f;
	filtered_setpoint_mm = 0.0f;
	stiction_boost_deg = 0.0f;
	desired_motor_pulses = BALL_MOTOR_LEVEL_PULSES;
	sent_motor_pulses = BALL_MOTOR_LEVEL_PULSES;
	fresh_packet_streak = 0U;
	camera_packet_seen = false;
	camera_sequence_seen = false;
	controller_armed = false;
	setpoint_filter_initialized = false;
	motor_link_ready = false;
	last_camera_sequence = 0U;
	last_camera_packet_ms = 0U;
	last_ball_fresh_ms = 0U;
	last_pid_update_ms = 0U;
	last_motor_service_ms = 0U;
	last_motor_send_ms = 0U;
	stiction_candidate_since_ms = 0U;
	stiction_direction = 0;

	/* Use the same startup order as the hardware-proven ZDT\1 project. The
	 * motor is intentionally controlled one-way so a missing PA10 reply or an
	 * older driver firmware cannot prevent PA9 motion commands from running. */
	Emm_V5_En_Control(BALL_MOTOR_ADDRESS, true, false);
	delay_ms(20);

#if BALL_MOTOR_ZERO_CURRENT_POSITION_ON_BOOT
	Emm_V5_Reset_CurPos_To_Zero(BALL_MOTOR_ADDRESS);
	delay_ms(20);
#endif

	/* A zero command alone does not select a repeatable side of the crank
	 * backlash. Move away and return to level from the same side on every boot. */
#if (BALL_MOTOR_STARTUP_PRELOAD_PULSES != BALL_MOTOR_LEVEL_PULSES)
	ball_control_send_absolute_position(BALL_MOTOR_STARTUP_PRELOAD_PULSES);
	delay_ms(BALL_MOTOR_STARTUP_PRELOAD_HOLD_MS);
#endif
	ball_control_send_absolute_position(BALL_MOTOR_LEVEL_PULSES);
	delay_ms(BALL_MOTOR_STARTUP_PRELOAD_HOLD_MS);
	motor_link_ready = true;
	ball_control_status.motor_link_ready = 1U;
}

void ball_control_handle_camera_packet(const CameraPacket_t *packet,
	                                    uint32_t now_ms)
{
	bool ball_fresh;
	bool edge_recovery;
	float dt_seconds;
	float requested_setpoint;
	float position_mm;
	float velocity_mm_s;
	float error_mm;
	float pid_error_mm;
	float candidate_integral;
	float unsaturated_output;
	float lower_output_limit;
	float upper_output_limit;
	float output_angle;
	float pulse_target;
	int32_t target_pulses;
	uint32_t prediction_age_ms;
	uint32_t packet_age_ms;

	if(packet == 0)
	{
		return;
	}
	if(!motor_link_ready)
	{
		ball_control_status.state = BALL_CONTROL_MOTOR_FAULT;
		return;
	}

	if(camera_sequence_seen && (packet->sequence == last_camera_sequence))
	{
		++ball_control_status.rejected_packets;
		return;
	}
	camera_sequence_seen = true;
	last_camera_sequence = packet->sequence;

	packet_age_ms = (uint32_t)(now_ms - packet->received_ms);
	camera_packet_seen = true;
	last_camera_packet_ms = packet->received_ms;
	ball_control_status.last_packet_age_ms = packet_age_ms;
	ball_control_status.camera_sequence = packet->sequence;
	ball_control_status.camera_flags = packet->flags;
	if(packet_age_ms > BALL_CAMERA_TIMEOUT_MS)
	{
		++ball_control_status.rejected_packets;
		ball_control_enter_safe_state(BALL_CONTROL_CAMERA_TIMEOUT);
		return;
	}

	ball_fresh = ((packet->flags & CAMERA_FLAG_BALL_FRESH) != 0U);
	edge_recovery = ((packet->flags & CAMERA_FLAG_EDGE_RECOVERY) != 0U);

	if(ball_fresh)
	{
		last_ball_fresh_ms = packet->received_ms;
	}

	if(!ball_control_packet_values_valid(packet))
	{
		++ball_control_status.rejected_packets;
		ball_control_enter_safe_state(BALL_CONTROL_BALL_INVALID);
		return;
	}

	position_mm = (float)packet->position_x10 / 10.0f;
	velocity_mm_s = control_clampf((float)packet->velocity_x10 / 10.0f,
	                              -BALL_VELOCITY_LIMIT_MM_PER_S,
	                               BALL_VELOCITY_LIMIT_MM_PER_S);
	requested_setpoint = control_clampf((float)packet->setpoint_x10 / 10.0f,
	                                   -(float)BALL_SETPOINT_LIMIT_X10 / 10.0f,
	                                    (float)BALL_SETPOINT_LIMIT_X10 / 10.0f);

	ball_control_status.ball_position_mm = position_mm;
	ball_control_status.ball_velocity_mm_s = velocity_mm_s;
	/* Keep the learned neutral bias across mode changes. The setpoint slew
	 * limiter handles the target transition without throwing the bias away. */
	ball_control_status.requested_setpoint_mm = requested_setpoint;
	++ball_control_status.accepted_packets;

	/* ZDT\1 starts from a valid real ball observation without also requiring
	 * the track detector to be fresh in that exact camera frame. */
	if(!controller_armed)
	{
		if(ball_fresh)
		{
			if(fresh_packet_streak < 255U)
			{
				++fresh_packet_streak;
			}
		}
		else
		{
			fresh_packet_streak = 0U;
		}

		ball_control_status.fresh_packet_streak = fresh_packet_streak;
		ball_control_status.state = BALL_CONTROL_ARMING;
		desired_motor_pulses = BALL_MOTOR_LEVEL_PULSES;
		ball_control_status.desired_motor_pulses = BALL_MOTOR_LEVEL_PULSES;

		if(fresh_packet_streak < BALL_REQUIRED_FRESH_PACKETS)
		{
			return;
		}

		controller_armed = true;
		filtered_setpoint_mm = position_mm;
		setpoint_filter_initialized = true;
		last_pid_update_ms = now_ms;
	}

	if(!ball_fresh)
	{
		prediction_age_ms = (uint32_t)(now_ms - last_ball_fresh_ms);
		if((edge_recovery &&
		    (prediction_age_ms > BALL_EDGE_RECOVERY_LIMIT_MS)) ||
		   (!edge_recovery &&
		    (prediction_age_ms > BALL_SHORT_PREDICTION_LIMIT_MS)))
		{
			ball_control_enter_safe_state(BALL_CONTROL_BALL_INVALID);
			return;
		}
	}

	if(last_pid_update_ms == 0U)
	{
		dt_seconds = 0.04f;
	}
	else
	{
		dt_seconds = (float)((uint32_t)(now_ms - last_pid_update_ms)) /
		             1000.0f;
		dt_seconds = control_clampf(dt_seconds, 0.001f, 0.20f);
	}
	last_pid_update_ms = now_ms;

	if(!setpoint_filter_initialized)
	{
		filtered_setpoint_mm = position_mm;
		setpoint_filter_initialized = true;
	}
	filtered_setpoint_mm = ball_control_slew(filtered_setpoint_mm,
	                                        requested_setpoint,
	                                        BALL_SETPOINT_SLEW_MM_PER_S,
	                                        dt_seconds);

	error_mm = filtered_setpoint_mm - position_mm;
	pid_error_mm = error_mm;
	if(control_absf(pid_error_mm) < BALL_PID_ERROR_DEADBAND_MM)
	{
		pid_error_mm = 0.0f;
	}

	candidate_integral = pid_integral_output_deg;
	if(ball_fresh && !edge_recovery &&
	   (control_absf(error_mm) <= BALL_PID_INTEGRAL_ACTIVE_ERROR_MM))
	{
		candidate_integral += BALL_PID_KI_DEG_PER_MM_S *
		                      pid_error_mm * dt_seconds;
		candidate_integral = control_clampf(candidate_integral,
		                                    -BALL_PID_I_OUTPUT_LIMIT_DEG,
		                                     BALL_PID_I_OUTPUT_LIMIT_DEG);
	}
	else
	{
		/* Freeze during short predictions/edge recovery. Decaying here made
		 * ordinary detector flicker continually erase the level correction. */
		candidate_integral = pid_integral_output_deg;
	}

	ball_control_get_output_limits(
		edge_recovery ? BALL_PID_EDGE_MAX_BEAM_ANGLE_DEG :
		                BALL_PID_MAX_BEAM_ANGLE_DEG,
		&lower_output_limit,
		&upper_output_limit);
	ball_control_update_stiction(ball_fresh, edge_recovery,
	                             error_mm, velocity_mm_s,
	                             dt_seconds, now_ms);
	unsaturated_output = BALL_PID_KP_DEG_PER_MM * pid_error_mm +
	                     candidate_integral -
	                     BALL_PID_KD_DEG_PER_MM_PER_S * velocity_mm_s +
	                     stiction_boost_deg;

	/* Conditional integration: reject only an integral change that deepens
	 * saturation. This still permits deliberate bias decay during prediction. */
	if(((unsaturated_output > upper_output_limit) &&
	    (candidate_integral > pid_integral_output_deg)) ||
	   ((unsaturated_output < lower_output_limit) &&
	    (candidate_integral < pid_integral_output_deg)))
	{
		candidate_integral = pid_integral_output_deg;
		unsaturated_output = BALL_PID_KP_DEG_PER_MM * pid_error_mm +
		                     candidate_integral -
		                     BALL_PID_KD_DEG_PER_MM_PER_S * velocity_mm_s +
		                     stiction_boost_deg;
	}
	pid_integral_output_deg = candidate_integral;
	output_angle = control_clampf(unsaturated_output,
	                              lower_output_limit,
	                              upper_output_limit);

	pulse_target = (float)BALL_MOTOR_LEVEL_PULSES +
	               output_angle * BALL_MOTOR_PULSES_PER_BEAM_DEGREE *
	               BALL_MOTOR_DIRECTION_SIGN;
	target_pulses = control_round_to_i32(pulse_target);
	target_pulses = control_clamp_i32(target_pulses,
	                                   BALL_MOTOR_MIN_PULSES,
	                                   BALL_MOTOR_MAX_PULSES);
	desired_motor_pulses = target_pulses;

	ball_control_status.state = edge_recovery ?
	                            BALL_CONTROL_EDGE_RECOVERY :
	                            (ball_fresh ? BALL_CONTROL_ACTIVE :
	                                          BALL_CONTROL_PREDICTED);
	ball_control_status.filtered_setpoint_mm = filtered_setpoint_mm;
	ball_control_status.error_mm = error_mm;
	ball_control_status.integral_output_deg = pid_integral_output_deg;
	ball_control_status.output_angle_deg = output_angle;
	ball_control_status.desired_motor_pulses = desired_motor_pulses;
}

void ball_control_service(uint32_t now_ms)
{
	uint32_t elapsed_ms;
	uint32_t service_elapsed_ms;
	uint32_t send_elapsed_ms;
	float maximum_step_float;
	int32_t maximum_step_pulses;
	int32_t delta;
	int32_t next_target;

	if(!motor_link_ready)
	{
		ball_control_status.state = BALL_CONTROL_MOTOR_FAULT;
		return;
	}

	if(camera_packet_seen)
	{
		elapsed_ms = (uint32_t)(now_ms - last_camera_packet_ms);
		ball_control_status.last_packet_age_ms = elapsed_ms;
		if(elapsed_ms > BALL_CAMERA_TIMEOUT_MS)
		{
			ball_control_enter_safe_state(BALL_CONTROL_CAMERA_TIMEOUT);
		}
	}
	else
	{
		ball_control_status.state = BALL_CONTROL_WAITING_CAMERA;
		desired_motor_pulses = BALL_MOTOR_LEVEL_PULSES;
		ball_control_status.desired_motor_pulses = BALL_MOTOR_LEVEL_PULSES;
	}

	/* Do not retain a stale correction forever when the real ball has been
	 * absent. Short dropouts keep it; a long absence starts cleanly. */
	if(!controller_armed && (last_ball_fresh_ms != 0U) &&
	   ((uint32_t)(now_ms - last_ball_fresh_ms) >
	    BALL_PID_INTEGRAL_MEMORY_MS))
	{
		pid_integral_output_deg = 0.0f;
		ball_control_status.integral_output_deg = 0.0f;
	}

	service_elapsed_ms = (uint32_t)(now_ms - last_motor_service_ms);
	if(service_elapsed_ms < BALL_MOTOR_COMMAND_INTERVAL_MS)
	{
		return;
	}
	last_motor_service_ms = now_ms;
	send_elapsed_ms = (uint32_t)(now_ms - last_motor_send_ms);
	if(service_elapsed_ms > BALL_MOTOR_MAX_SLEW_DT_MS)
	{
		service_elapsed_ms = BALL_MOTOR_MAX_SLEW_DT_MS;
	}

	maximum_step_float = BALL_MOTOR_PULSES_PER_BEAM_DEGREE *
	                     BALL_TILT_SLEW_DEG_PER_S *
	                     (float)service_elapsed_ms / 1000.0f;
	maximum_step_pulses = control_round_to_i32(maximum_step_float);
	if(maximum_step_pulses < 1)
	{
		maximum_step_pulses = 1;
	}
	if(maximum_step_pulses > BALL_MOTOR_MAX_STEP_PULSES)
	{
		maximum_step_pulses = BALL_MOTOR_MAX_STEP_PULSES;
	}

	delta = desired_motor_pulses - sent_motor_pulses;
	delta = control_clamp_i32(delta, -maximum_step_pulses,
	                         maximum_step_pulses);
	next_target = sent_motor_pulses + delta;
	next_target = control_clamp_i32(next_target,
	                                 BALL_MOTOR_MIN_PULSES,
	                                 BALL_MOTOR_MAX_PULSES);

	if(((next_target - sent_motor_pulses) >=
	    BALL_MOTOR_MIN_CHANGE_PULSES) ||
	   ((sent_motor_pulses - next_target) >=
	    BALL_MOTOR_MIN_CHANGE_PULSES) ||
	   (send_elapsed_ms >= BALL_MOTOR_COMMAND_KEEPALIVE_MS))
	{
		ball_control_send_absolute_position(next_target);
		last_motor_send_ms = now_ms;
	}
}
