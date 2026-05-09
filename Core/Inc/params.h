/**
 * @file params.h
 * @brief Global adjustable parameters for the PID Motor Controller.
 */

#ifndef PARAMS_H
#define PARAMS_H

/* ============================================================================
 * PID GAINS (Tuning)
 * ============================================================================ */

/* Speed Loop (Inner) */
#define DEFAULT_SPEED_KP    1.0f
#define DEFAULT_SPEED_KI    2.0f
#define DEFAULT_SPEED_KD    0.0f
#define DEFAULT_SPEED_KF    1.302f  /**< Your characterized Feed-Forward gain */

/* Position Loop (Outer) */
#define DEFAULT_POS_KP      1.2f    /**< Increased for snappy response with S-Curve */
#define DEFAULT_POS_KI      0.05f
#define DEFAULT_POS_KD      0.1f

/* ============================================================================
 * MOTION & JOG SPEEDS
 * ============================================================================ */

#define JOG_SPEED_FINE      10.0f   /**< Increased for better feedback */
#define MOVE_SPEED_COARSE   69.75f  /**< 7.304 rad/s converted to RPM */

#define STEP_SIZE_COARSE    10.0f
#define STEP_SIZE_FINE      1.0f

/* ============================================================================
 * HARDWARE LIMITS
 * ============================================================================ */

#define MAX_VOLTAGE_LIMIT   24.0f
#define SUPPLY_VOLTAGE      24.0f

#define PID_INTEGRAL_MAX    50.0f
#define POS_INTEGRAL_MAX    200.0f

#define DEFAULT_MIN_PWM     5.0f    /**< Increased to overcome static friction */
#define DEFAULT_MAX_ACCEL   262.51f /**< 27.49 rad/s^2 converted to RPM/s^2 */

/* ============================================================================
 * SAFETY PROTOCOLS
 * ============================================================================ */

#define STALL_PWM_THRESHOLD      25.0f   /**< Increased from 15% */
#define STALL_VELOCITY_THRESHOLD 0.5f    /**< More sensitive low-speed detection */
#define STALL_TIME_MS            2000    /**< Give it 2 seconds to start moving */
#define STALL_SETTLING_ERROR_DEG 5.0f    /**< Don't trigger stall if error < 5 deg */

#define ENCODER_FAULT_PWM_THRESHOLD 25.0f   /**< PWM threshold for hardware check */
#define ENCODER_INVERSION_RPM_LIMIT 5.0f    /**< RPM threshold for inversion check */

#define SOFT_LIMIT_DEG           720.0f  /**< 2 full rounds limit from home */

/* ============================================================================
 * SYSTEM TIMING
 * ============================================================================ */

#define HOME_HOLD_TIME_MS   3000

#endif /* PARAMS_H */
