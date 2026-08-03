#ifndef __BALL_CONTROL_CONFIG_H
#define __BALL_CONTROL_CONFIG_H

/* ------------------------- Motor/geometry setup ------------------------- */

#define BALL_MOTOR_ADDRESS                         1U
#define BALL_MOTOR_POSITION_SPEED_RPM             300U
#define BALL_MOTOR_POSITION_ACCELERATION           30U

/* The tube must be mechanically level when the board is powered/reset.
 * This command only declares the present motor position as coordinate zero;
 * it does not move the mechanism. Set to 0 only if the driver already has a
 * trustworthy absolute zero from an external homing procedure. */
#define BALL_MOTOR_ZERO_CURRENT_POSITION_ON_BOOT    1

/* Change +1.0f to -1.0f if a positive control command makes the ball move
 * toward negative X (away from a positive setpoint). */
#define BALL_MOTOR_DIRECTION_SIGN                  1.0f

/* Match the direction convention used by the proven ZDT\1 controller:
 * a positive signed absolute pulse target is encoded as CW when this is 1. */
#define BALL_MOTOR_POSITIVE_IS_CW                   1U

/* This linkage-dependent value must be measured. At the current ZDT\1-derived
 * +/-8 degree software limit, the default permits +/-320 motor pulses. */
#define BALL_MOTOR_PULSES_PER_BEAM_DEGREE          40.0f

/* Independent hard travel guards. Measure both directions separately; crank
 * linkages are commonly asymmetric. All three values use the motor driver's
 * signed absolute pulse coordinate. */
#define BALL_MOTOR_LEVEL_PULSES                      0L
#define BALL_MOTOR_MIN_PULSES                     -320L
#define BALL_MOTOR_MAX_PULSES                      320L

/* Remove crank/linkage backlash at every power-up by approaching level from
 * one fixed direction. 40 pulses is about 1 degree with the current linkage.
 * Reverse the sign only if the chosen preload direction is mechanically bad. */
#define BALL_MOTOR_STARTUP_PRELOAD_PULSES           -40L
#define BALL_MOTOR_STARTUP_PRELOAD_HOLD_MS          160U

#define BALL_MOTOR_COMMAND_INTERVAL_MS             20U
#define BALL_MOTOR_COMMAND_KEEPALIVE_MS            500U
#define BALL_MOTOR_MAX_SLEW_DT_MS                   50U
#define BALL_MOTOR_MAX_STEP_PULSES                  24L
#define BALL_MOTOR_MIN_CHANGE_PULSES                 1L

/* ---------------------------- PID parameters ---------------------------- */

/* Output unit is requested beam angle in degrees. With 40 pulses/degree these
 * are exactly the ZDT\1 gains of Kp=2.00, Ki=0.15 and Kd=0.35 in pulse units. */
#define BALL_PID_KP_DEG_PER_MM                      0.05000f
#define BALL_PID_KI_DEG_PER_MM_S                    0.00375f
#define BALL_PID_KD_DEG_PER_MM_PER_S                0.00875f

#define BALL_PID_MAX_BEAM_ANGLE_DEG                 8.0f
#define BALL_PID_EDGE_MAX_BEAM_ANGLE_DEG            8.0f
#define BALL_PID_I_OUTPUT_LIMIT_DEG                 4.0f
#define BALL_PID_INTEGRAL_ACTIVE_ERROR_MM         120.0f
#define BALL_PID_ERROR_DEADBAND_MM                  1.0f

/* A short visual dropout must not erase the learned tube-level bias. Clear it
 * only after the real ball has been absent continuously for this long. */
#define BALL_PID_INTEGRAL_MEMORY_MS                5000U

/* Static-friction release. When the ball has a visible position error but is
 * nearly stationary, add a temporary tilt in the error direction. The boost
 * ramps in/out so that a released ball is not hit by a discontinuous command. */
#define BALL_STICTION_ERROR_THRESHOLD_MM             5.0f
#define BALL_STICTION_VELOCITY_THRESHOLD_MM_S        8.0f
#define BALL_STICTION_CONFIRM_MS                    300U
#define BALL_STICTION_MAX_BOOST_DEG                   1.5f
#define BALL_STICTION_BOOST_RISE_DEG_PER_S            2.5f
#define BALL_STICTION_BOOST_FALL_DEG_PER_S            5.0f

/* Limits reference and actuator changes independently of the PID gains. */
#define BALL_SETPOINT_SLEW_MM_PER_S                 80.0f
#define BALL_TILT_SLEW_DEG_PER_S                    30.0f

/* -------------------------- Link/safety limits -------------------------- */

#define BALL_CAMERA_TIMEOUT_MS                     500U
#define BALL_SHORT_PREDICTION_LIMIT_MS             220U
#define BALL_EDGE_RECOVERY_LIMIT_MS                600U
#define BALL_REQUIRED_FRESH_PACKETS                  1U
#define BALL_CAMERA_MIN_CONFIDENCE_PERCENT          20U

#define BALL_SETPOINT_LIMIT_X10                    1200
#define BALL_POSITION_MIN_X10                     -1200
#define BALL_POSITION_MAX_X10                      1200
#define BALL_VELOCITY_LIMIT_MM_PER_S                600.0f

/* Catch unsafe or internally inconsistent travel settings at build time. */
#if (BALL_MOTOR_ADDRESS == 0U) || (BALL_MOTOR_ADDRESS > 255U)
#error "BALL_MOTOR_ADDRESS must select one motor (1..255)"
#endif

#if (BALL_MOTOR_POSITION_SPEED_RPM > 5000U)
#error "BALL_MOTOR_POSITION_SPEED_RPM exceeds the Emm_V5 range"
#endif

#if (BALL_MOTOR_POSITION_ACCELERATION > 255U)
#error "BALL_MOTOR_POSITION_ACCELERATION exceeds the Emm_V5 range"
#endif

#if (BALL_MOTOR_POSITIVE_IS_CW != 0U) && (BALL_MOTOR_POSITIVE_IS_CW != 1U)
#error "BALL_MOTOR_POSITIVE_IS_CW must be 0 or 1"
#endif

#if (BALL_MOTOR_MIN_PULSES >= BALL_MOTOR_LEVEL_PULSES)
#error "BALL_MOTOR_MIN_PULSES must be below BALL_MOTOR_LEVEL_PULSES"
#endif

#if (BALL_MOTOR_LEVEL_PULSES >= BALL_MOTOR_MAX_PULSES)
#error "BALL_MOTOR_LEVEL_PULSES must be below BALL_MOTOR_MAX_PULSES"
#endif

#if (BALL_MOTOR_STARTUP_PRELOAD_PULSES < BALL_MOTOR_MIN_PULSES) || \
    (BALL_MOTOR_STARTUP_PRELOAD_PULSES > BALL_MOTOR_MAX_PULSES)
#error "BALL_MOTOR_STARTUP_PRELOAD_PULSES is outside the travel limits"
#endif

#if (BALL_MOTOR_STARTUP_PRELOAD_HOLD_MS == 0U)
#error "BALL_MOTOR_STARTUP_PRELOAD_HOLD_MS must be nonzero"
#endif

#if BALL_MOTOR_ZERO_CURRENT_POSITION_ON_BOOT && (BALL_MOTOR_LEVEL_PULSES != 0L)
#error "Automatic current-position zeroing requires LEVEL_PULSES == 0"
#endif

#if (BALL_MOTOR_COMMAND_INTERVAL_MS == 0U)
#error "BALL_MOTOR_COMMAND_INTERVAL_MS must be nonzero"
#endif

#if (BALL_MOTOR_MAX_STEP_PULSES <= 0L)
#error "BALL_MOTOR_MAX_STEP_PULSES must be positive"
#endif

#if (BALL_MOTOR_MIN_CHANGE_PULSES <= 0L)
#error "BALL_MOTOR_MIN_CHANGE_PULSES must be positive"
#endif

#if (BALL_REQUIRED_FRESH_PACKETS == 0U)
#error "BALL_REQUIRED_FRESH_PACKETS must be nonzero"
#endif

#endif
