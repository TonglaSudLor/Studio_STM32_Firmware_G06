/**
 * @file motor_controller.h
 * @brief Professional PID Motor Controller for STM32
 * 
 * This module provides a dual-loop (position/speed) PID controller with
 * trajectory generation, autotuning, and safety monitoring features.
 * 
 * @author Gemini CLI (Refactored)
 * @date May 2026
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* --- Hardware Configuration --- */
#define MOTOR_ENCODER_PPR           2048    /**< Encoder pulses per revolution */
#define MOTOR_GEAR_RATIO            1.0f    /**< Motor gear ratio */
#define MOTOR_CONTROL_FREQ_HZ       100     /**< Control loop frequency (Hz) */

/* --- Control Structures --- */

/**
 * @brief Motor control operating modes
 */
typedef enum {
    MOTOR_MODE_STOPPED = 0,
    MOTOR_MODE_SPEED,
    MOTOR_MODE_POSITION,
    MOTOR_MODE_AUTOTUNE,
    MOTOR_MODE_AUTOTUNE_SPEED,
    MOTOR_MODE_TEST,
    MOTOR_MODE_GHOST
} Motor_ControlMode_t;

/**
 * @brief System control modes (Base or Joystick)
 */
typedef enum {
    CONTROL_MODE_BASE_SYSTEM = 0,
    CONTROL_MODE_JOYSTICK = 1
} Control_SystemMode_t;

/**
 * @brief System fault codes (Bitmask)
 */
typedef enum {
    FAULT_NONE = 0x00,
    FAULT_MOTOR_STALLED = 0x01,
    FAULT_ENCODER_ERROR = 0x02,
    FAULT_JOYSTICK_LOST = 0x04,
    FAULT_OVER_ROTATION = 0x08     /**< Motor exceeded rotation limit */
} Motor_FaultCode_t;

/**
 * @brief Jog operation modes
 */
typedef enum {
    JOG_COARSE = 0,
    JOG_FINE = 1
} Motor_JogMode_t;

/**
 * @brief Autotune trigger states
 */
typedef enum {
    ATUNE_IDLE = 0,
    ATUNE_POS,    /**< Start Position Autotune */
    ATUNE_SPEED   /**< Start Speed Autotune */
} Motor_AutotuneTrigger_t;

/**
 * @brief Autotune execution status
 */
typedef enum {
    STATUS_IDLE = 0,
    STATUS_RUNNING_POS,
    STATUS_RUNNING_SPEED,
    STATUS_SUCCESS,
    STATUS_ERROR_LIMIT_EXCEEDED
} Motor_AutotuneStatus_t;

/**
 * @brief Tuning parameters for PID loops
 */
typedef struct {
    // PID Speed Loop
    float speed_Kp;
    float speed_Ki;
    float speed_Kd;
    float speed_Kf;            /**< Feed-Forward Gain (PWM per RPM) */
    
    // PID Position Loop
    float pos_Kp;
    float pos_Ki;
    float pos_Kd;
    
    // Jog & Step Settings
    float jog_speed_fine;      /**< RPM for continuous jog */
    float move_speed_coarse;   /**< RPM for step movement */
    float step_size_coarse;    /**< Degrees per click in Coarse mode */
    float step_size_fine;      /**< Degrees per click in Fine mode */
    float min_pwm;             /**< Minimum PWM to overcome friction */
    float max_accel;           /**< Maximum acceleration (RPM/s) */
} Motor_TuningParams_t;

/**
 * @brief PID Controller state and configuration
 */
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float integral_max;
    float error_prev;
    float d_filt;              /**< Low-pass filter state for derivative term */
    float output_min, output_max;
} PID_Controller_t;

/**
 * @brief Encoder state and measurement data
 */
typedef struct {
    int32_t absolute_counts;
    uint32_t count_prev;
    float current_position_deg;
    float filtered_rpm;
} Encoder_Data_t;

/**
 * @brief Trajectory configuration parameters
 */
typedef struct {
    float max_velocity;
    float max_acceleration;
    float jerk_smoothing;
} Trajectory_Config_t;

/**
 * @brief current trajectory state
 */
typedef struct {
    float target_pos;
    float target_vel;
    float current_setpoint_pos;
    float current_setpoint_vel;
    float current_setpoint_accel;
} Trajectory_State_t;

/* --- Ghost Mode Buffering --- */
#define GHOST_BUFFER_MAX 200
typedef struct {
    uint32_t tick;
    int32_t pos_x100;
    int32_t target_x100;
} Ghost_Buffer_t;

/* --- Public Variables --- */
extern volatile Motor_ControlMode_t current_mode;
extern volatile Control_SystemMode_t control_system_mode;
extern volatile Motor_JogMode_t jog_mode;
extern volatile Motor_AutotuneTrigger_t autotune_trigger;
extern volatile Motor_AutotuneStatus_t autotune_status;
extern volatile int tuning_progress;
extern volatile Motor_TuningParams_t tuning;
extern volatile bool is_joystick_connected;
extern volatile bool emergency_stop;
extern volatile bool safety_enabled;
extern volatile Motor_FaultCode_t fault_code;
extern volatile float target_position_deg;
extern volatile float buffered_target_pos;   /**< Ghost target for S-curve testing */
extern volatile bool ghost_move_active;
extern volatile uint32_t ghost_settle_start_tick;

extern Ghost_Buffer_t ghost_buffer[GHOST_BUFFER_MAX];
extern volatile uint32_t ghost_buffer_idx;
extern volatile bool ghost_dump_requested;

extern PID_Controller_t pid_speed;
extern PID_Controller_t pid_position;
extern Encoder_Data_t encoder;
extern Trajectory_State_t trajectory;

/* --- Public Functions --- */

/**
 * @brief Initialize motor control system
 */
void Motor_Init(void);

/**
 * @brief Move motor to target position in degrees
 * @param target_degrees Target position
 */
void Motor_MoveToPosition(float target_degrees);

/**
 * @brief Set voltage limits for PWM mapping
 * @param max_voltage Maximum allowed voltage (V)
 * @param supply_voltage Actual supply voltage (V)
 */
void Motor_SetVoltageLimit(float max_voltage, float supply_voltage);

/**
 * @brief Configure motion profile for position control
 * @param max_rpm Maximum speed (RPM)
 * @param max_accel Maximum acceleration (RPM/s^2)
 * @param smoothing S-Curve smoothing factor (0.0 to 1.0)
 */
void Motor_SetMotionProfile(float max_rpm, float max_accel, float smoothing);

/**
 * @brief Set target velocity for speed control
 * @param rpm Target velocity in RPM
 */
void Motor_SetJogVelocity(float rpm);

/**
 * @brief Process incoming command characters
 * @param cmd Command character
 */
void Motor_ProcessCommand(char cmd);

/**
 * @brief Update selection button state
 * @param pressed True if button is pressed
 */
void Motor_UpdateSelectionButton(bool pressed);

/**
 * @brief Update mode button state
 * @param pressed True if button is pressed
 */
void Motor_UpdateModeButton(bool pressed);

/**
 * @brief Update control mode button state
 * @param pressed True if button is pressed
 */
void Motor_UpdateControlModeButton(bool pressed);

/**
 * @brief Set joystick connection status
 * @param connected True if joystick is connected
 */
void Motor_SetConnectionStatus(bool connected);

/**
 * @brief Stream telemetry data to MATLAB
 */
void Motor_SendDataToMatlab(void);

/**
 * @brief Start relay-based position autotune
 */
void Motor_StartAutotune(void);

/**
 * @brief Start relay-based speed autotune
 */
void Motor_StartAutotuneSpeed(void);

/**
 * @brief Main control loop ISR (100Hz)
 */
void Motor_ControlLoop(void);

/**
 * @brief Get current position in degrees
 * @return float Position (deg)
 */
float Motor_GetPosition(void);

/**
 * @brief Get current speed in RPM
 * @return float Speed (RPM)
 */
float Motor_GetSpeed(void);

/* --- Gripper Functions --- */
void Gripper_Up(void);
void Gripper_Down(void);
void Gripper_Open(void);
void Gripper_Close(void);
void Gripper_Toggle(void);
void Gripper_Sequence_Pick(void);
void Gripper_Sequence_Place(void);

#endif /* MOTOR_CONTROLLER_H */