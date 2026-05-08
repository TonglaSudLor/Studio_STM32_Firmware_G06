/**
 * @file motor_controller.c
 * @brief Professional PID Motor Controller for STM32
 * 
 * @author Gemini CLI (Refactored)
 * @date May 2026
 */

#include "motor_controller.h"
#include "params.h"
#include <math.h>
#include <stdio.h>

/* ============================================================================
 * Private System Variables
 * ============================================================================ */

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;

volatile Motor_ControlMode_t current_mode = MOTOR_MODE_STOPPED;
volatile Control_SystemMode_t control_system_mode = CONTROL_MODE_BASE_SYSTEM;
volatile Motor_JogMode_t jog_mode = JOG_COARSE;
volatile Motor_AutotuneTrigger_t autotune_trigger = ATUNE_IDLE;
volatile Motor_AutotuneStatus_t autotune_status = STATUS_IDLE;
volatile int tuning_progress = 0;
volatile Motor_TuningParams_t tuning;
volatile bool is_joystick_connected = false;
volatile bool emergency_stop = false;
volatile bool safety_enabled = true;
volatile Motor_FaultCode_t fault_code = FAULT_NONE;
volatile float target_position_deg = 0.0f;
volatile float buffered_target_pos = 0.0f;
volatile bool ghost_move_active = false;
volatile uint32_t ghost_settle_start_tick = 0;
Ghost_Buffer_t ghost_buffer[GHOST_BUFFER_MAX];
volatile uint32_t ghost_buffer_idx = 0;
volatile bool ghost_dump_requested = false;

static float control_dt = 0.01f;
static uint32_t open_loop_test_tick = 0;
static uint32_t m_button_hold_tick = 0;
static bool m_button_active = false;
static uint32_t y_button_hold_tick = 0;
static bool y_button_active = false;
static uint32_t b_button_hold_tick = 0;
static bool b_button_active = false;
static float last_rpm_for_accel = 0.0f;
static uint32_t a_press_tick = 0;    
static uint32_t stall_timer = 0;
static uint32_t encoder_fault_timer = 0;
static uint32_t joystick_watchdog_timer = 0;
static int32_t last_absolute_counts = 0;
static bool a_button_is_held = false;
static bool returning_home = false;

/* Autotune State Machine */
static struct {
    float relay_output; 
    float peak_max; 
    float peak_min;          
    uint32_t last_flip_tick; 
    uint32_t period_sum; 
    float amplitude_sum;     
    int cycle_count; 
    int8_t motor_sign; 
    bool direction;          
    float center_pos; 
    float target_val;        
} atune;

PID_Controller_t pid_speed;
PID_Controller_t pid_position;
Encoder_Data_t encoder;
Trajectory_Config_t motion_config;
Trajectory_State_t trajectory;

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */
static float PID_Compute(PID_Controller_t *pid, float setpoint, float feedback);
static void PWM_Apply(float duty_cycle);
static void Encoder_Update(void);
static void Trajectory_Generator_Update(void);

/* ============================================================================
 * PID Control Implementation
 * ============================================================================ */

/**
 * @brief Compute PID output
 */
static float PID_Compute(PID_Controller_t *pid, float setpoint, float feedback)
{
    float error = setpoint - feedback;
    float p_term = pid->Kp * error;
    
    pid->integral += error * control_dt;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    
    float i_term = pid->Ki * pid->integral;
    float d_input = (feedback - pid->error_prev); 
    float d_term_raw = (pid->Kd * d_input) / control_dt;
    
    // Low-pass filter for derivative term
    pid->d_filt = (0.8f * pid->d_filt) + (0.2f * d_term_raw); 
    pid->error_prev = feedback;
    
    return p_term + i_term - pid->d_filt;
}

/**
 * @brief Apply PWM duty cycle to motor
 */
static void PWM_Apply(float duty_cycle)
{
    float min_p = tuning.min_pwm;
    
    if (fabsf(duty_cycle) < 0.1f) { 
        duty_cycle = 0.0f; 
    } else {
        if (duty_cycle > 0) duty_cycle += min_p;
        else duty_cycle -= min_p;
    }
    
    if (duty_cycle > pid_speed.output_max) duty_cycle = pid_speed.output_max;
    else if (duty_cycle < pid_speed.output_min) duty_cycle = pid_speed.output_min;

    bool forward = (duty_cycle >= 0); 
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim8);
    uint32_t pwm_value = (uint32_t)((arr * fabsf(duty_cycle)) / 100.0f);
    
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwm_value);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
 * @brief Update encoder readings and calculate RPM
 */
static void Encoder_Update(void)
{
    uint32_t current_count = __HAL_TIM_GET_COUNTER(&htim3);
    int32_t delta = (int32_t)current_count - (int32_t)encoder.count_prev;
    
    // Handle timer rollover
    if (delta > 32767) delta -= 65536;
    else if (delta < -32768) delta += 65536;
    
    encoder.count_prev = current_count;
    encoder.absolute_counts += delta;
    
    float counts_per_rev = MOTOR_ENCODER_PPR * 4 * MOTOR_GEAR_RATIO;
    encoder.current_position_deg = ((float)encoder.absolute_counts / counts_per_rev) * 360.0f;
    
    float instant_rpm = ((float)delta / counts_per_rev / control_dt) * 60.0f;
    encoder.filtered_rpm = (0.15f * instant_rpm) + (0.85f * encoder.filtered_rpm);
}

/**
 * @brief Update trajectory setpoints for smooth movement
 */
static void Trajectory_Generator_Update(void)
{
    float error = trajectory.target_pos - trajectory.current_setpoint_pos;
    float stop_dist_deg = 360.0f * (trajectory.current_setpoint_vel * trajectory.current_setpoint_vel) / (2.0f * tuning.max_accel * 60.0f);
    
    float target_v = tuning.move_speed_coarse;
    if (fabsf(error) < stop_dist_deg) {
        target_v = sqrtf(2.0f * (tuning.max_accel / 60.0f) * (fabsf(error) / 360.0f)) * 60.0f;
    }
    
    if (error < 0) target_v = -target_v;
    
    float v_step = tuning.max_accel * control_dt;
    if (target_v > trajectory.current_setpoint_vel + v_step)
        trajectory.current_setpoint_vel += v_step;
    else if (target_v < trajectory.current_setpoint_vel - v_step)
        trajectory.current_setpoint_vel -= v_step;
    else
        trajectory.current_setpoint_vel = target_v;
        
    trajectory.current_setpoint_pos += (trajectory.current_setpoint_vel / 60.0f) * 360.0f * control_dt;
}

static float resolution_step = 10.0f; 
static bool step_executed = false;   

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

void Motor_Init(void)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    encoder.count_prev = 0; 
    encoder.absolute_counts = 0;
    
    // Initialize parameters from defaults
    tuning.speed_Kp = DEFAULT_SPEED_KP; 
    tuning.speed_Ki = DEFAULT_SPEED_KI; 
    tuning.speed_Kd = DEFAULT_SPEED_KD;
    tuning.pos_Kp = DEFAULT_POS_KP; 
    tuning.pos_Ki = DEFAULT_POS_KI; 
    tuning.pos_Kd = DEFAULT_POS_KD;
    tuning.speed_Kf = DEFAULT_SPEED_KF; 
    
    tuning.jog_speed_fine = JOG_SPEED_FINE; 
    tuning.move_speed_coarse = MOVE_SPEED_COARSE;
    tuning.step_size_coarse = STEP_SIZE_COARSE; 
    tuning.step_size_fine = STEP_SIZE_FINE;
    tuning.min_pwm = DEFAULT_MIN_PWM; 
    tuning.max_accel = DEFAULT_MAX_ACCEL;
    
    resolution_step = tuning.step_size_coarse;
    
    // Initialize PID instances
    pid_speed.Kp = tuning.speed_Kp; 
    pid_speed.Ki = tuning.speed_Ki; 
    pid_speed.Kd = tuning.speed_Kd;
    pid_speed.integral_max = PID_INTEGRAL_MAX; 
    pid_speed.output_max = 100.0f; 
    pid_speed.output_min = -100.0f;
    
    pid_position.Kp = tuning.pos_Kp; 
    pid_position.Ki = tuning.pos_Ki; 
    pid_position.Kd = tuning.pos_Kd;
    pid_position.integral_max = POS_INTEGRAL_MAX;
    
    // Start hardware peripherals
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    __HAL_TIM_MOE_ENABLE(&htim8);
    HAL_TIM_Base_Start_IT(&htim6);
}

void Motor_MoveToPosition(float target_degrees)
{
    if (emergency_stop) return;
    trajectory.target_pos = target_degrees;
    current_mode = MOTOR_MODE_POSITION;
}

void Motor_SetVoltageLimit(float max_voltage, float supply_voltage)
{
    if (supply_voltage <= 0.0f) return;
    float max_pwm = (max_voltage / supply_voltage) * 100.0f;
    pid_speed.output_max = max_pwm; 
    pid_speed.output_min = -max_pwm;
}

void Motor_SetMotionProfile(float max_rpm, float max_accel, float smoothing)
{
    tuning.move_speed_coarse = max_rpm; 
    tuning.max_accel = max_accel;
    // Smoothing not fully implemented in Trajectory_Generator_Update yet
}

void Motor_SetJogVelocity(float rpm)
{
    if (emergency_stop) return;
    trajectory.target_vel = rpm;
    current_mode = MOTOR_MODE_SPEED;
}

void Motor_UpdateSelectionButton(bool pressed)
{
    if (pressed) {
        if (!y_button_active) {
            y_button_hold_tick = HAL_GetTick();
            y_button_active = true;
        }
    } else {
        y_button_active = false;
    }
}

void Motor_UpdateModeButton(bool pressed)
{
    if (pressed) {
        if (!m_button_active) {
            m_button_hold_tick = HAL_GetTick();
            m_button_active = true;
        }
    } else {
        m_button_active = false;
    }
}

void Motor_UpdateControlModeButton(bool pressed)
{
    if (pressed) {
        if (!b_button_active) {
            b_button_hold_tick = HAL_GetTick();
            b_button_active = true;
        }
    } else {
        b_button_active = false;
    }
}

void Motor_SetConnectionStatus(bool connected) 
{ 
    is_joystick_connected = connected; 
}

void Motor_SendDataToMatlab(void)
{
    float ghost_pos = (current_mode == MOTOR_MODE_GHOST) ? buffered_target_pos : trajectory.target_pos;

    if (ghost_move_active) {
        // BUFFERING MODE: Store data silently
        if (ghost_buffer_idx < GHOST_BUFFER_MAX) {
            ghost_buffer[ghost_buffer_idx].tick = HAL_GetTick();
            ghost_buffer[ghost_buffer_idx].pos_x100 = (long)(encoder.current_position_deg * 100.0f);
            ghost_buffer[ghost_buffer_idx].target_x100 = (long)(trajectory.target_pos * 100.0f);
            ghost_buffer_idx++;
        }
    } else if (ghost_dump_requested) {
        // DUMPING MODE: Send the whole buffer as fast as possible
        for (uint32_t i = 0; i < ghost_buffer_idx; i++) {
            printf("DATA,%lu,%ld,%ld\r\n", 
                   ghost_buffer[i].tick, 
                   ghost_buffer[i].pos_x100, 
                   ghost_buffer[i].target_x100);
        }
        printf("END\r\n");
        ghost_dump_requested = false;
        ghost_buffer_idx = 0;
    } else {
        // PREVIEW MODE: Send only target and current pos for the live preview
        printf("PREVIEW,%ld,%ld\r\n", 
               (long)(encoder.current_position_deg * 100.0f),
               (long)(ghost_pos * 100.0f));
    }
}

void Motor_StartAutotune(void)
{
    if (emergency_stop) return;
    atune.relay_output = 25.0f; 
    atune.center_pos = encoder.current_position_deg; 
    atune.peak_max = -999.0f; 
    atune.peak_min = 999.0f; 
    atune.cycle_count = -1; 
    atune.amplitude_sum = 0.0f; 
    atune.period_sum = 0; 
    atune.last_flip_tick = HAL_GetTick();
    atune.direction = true; 
    atune.motor_sign = 1;
    tuning_progress = 0; 
    autotune_status = STATUS_RUNNING_POS;
    current_mode = MOTOR_MODE_AUTOTUNE;
}

void Motor_StartAutotuneSpeed(void)
{
    if (emergency_stop) return;
    atune.relay_output = 25.0f; 
    atune.center_pos = encoder.current_position_deg;
    atune.target_val = 5.0f; 
    atune.peak_max = -999.0f; 
    atune.peak_min = 999.0f;
    atune.cycle_count = -1; 
    atune.amplitude_sum = 0.0f; 
    atune.period_sum = 0;
    atune.last_flip_tick = HAL_GetTick(); 
    atune.direction = true; 
    atune.motor_sign = 1;
    tuning_progress = 0; 
    autotune_status = STATUS_RUNNING_SPEED;
    current_mode = MOTOR_MODE_AUTOTUNE_SPEED;
}

void Motor_ProcessCommand(char cmd)
{
    // RESET WATCHDOG: Receiving any character indicates the joystick is alive
    is_joystick_connected = true;
    joystick_watchdog_timer = HAL_GetTick();
    fault_code &= ~FAULT_JOYSTICK_LOST;

    // PRIORITY 1: Emergency Stop Logic
    if (cmd == 'P') {
        if (!emergency_stop) { 
            emergency_stop = true; 
            if (ghost_move_active) {
                ghost_move_active = false;
                ghost_dump_requested = true;
            }
        } else { 
            emergency_stop = false; 
            fault_code = FAULT_NONE; 
            current_mode = MOTOR_MODE_STOPPED; 
        }
        trajectory.current_setpoint_vel = 0.0f;
        trajectory.current_setpoint_pos = encoder.current_position_deg;
        trajectory.target_pos = encoder.current_position_deg;
        pid_speed.integral = 0.0f; 
        pid_position.integral = 0.0f;
        return;
    }

    // PRIORITY 2: Config Buttons (Allowed during E-Stop)
    if (cmd == 'A') {
        if (!a_button_is_held) { 
            a_press_tick = HAL_GetTick(); 
            a_button_is_held = true; 
            returning_home = false; 
        }
    }
    
    if (cmd == 'M') { 
        if (jog_mode == JOG_COARSE) { 
            jog_mode = JOG_FINE; 
            resolution_step = tuning.step_size_fine; 
        } else { 
            jog_mode = JOG_COARSE; 
            resolution_step = tuning.step_size_coarse; 
        }
        return; 
    }

    // PRIORITY 3: Base System Check
    if (control_system_mode == CONTROL_MODE_BASE_SYSTEM) return;

    // PRIORITY 4: Motion Commands (Blocked if E-Stop)
    if (emergency_stop) return;

    switch (cmd)
    {
    case 'U': Gripper_Up(); break;
    case 'D': Gripper_Down(); break;
    case 'F': Gripper_Toggle(); break;
    case 'S': Gripper_Sequence_Pick(); break;
    case 'G': Gripper_Sequence_Place(); break;
    case 'L': 
        if (current_mode == MOTOR_MODE_GHOST) {
            buffered_target_pos -= resolution_step;
        } else if (jog_mode == JOG_COARSE) {
            if (!step_executed) { 
                trajectory.target_pos -= tuning.step_size_coarse; 
                current_mode = MOTOR_MODE_POSITION; 
                step_executed = true; 
            }
        } else { 
            trajectory.target_vel = -tuning.jog_speed_fine; 
            current_mode = MOTOR_MODE_SPEED; 
        }
        break;
    case 'R': 
        if (current_mode == MOTOR_MODE_GHOST) {
            buffered_target_pos += resolution_step;
        } else if (jog_mode == JOG_COARSE) {
            if (!step_executed) { 
                trajectory.target_pos += tuning.step_size_coarse; 
                current_mode = MOTOR_MODE_POSITION; 
                step_executed = true; 
            }
        } else { 
            trajectory.target_vel = tuning.jog_speed_fine; 
            current_mode = MOTOR_MODE_SPEED; 
        }
        break;
    case 'O': 
        step_executed = false;
        if (a_button_is_held) {
            uint32_t duration = HAL_GetTick() - a_press_tick;
            if (duration < 3000 && !returning_home) {
                // RESET HOME POSITION
                __HAL_TIM_SET_COUNTER(&htim3, 0);
                encoder.count_prev = 0;
                encoder.absolute_counts = 0;
                encoder.current_position_deg = 0.0f;
                encoder.filtered_rpm = 0.0f;
                last_rpm_for_accel = 0.0f;
                
                trajectory.target_pos = 0.0f;
                trajectory.current_setpoint_pos = 0.0f;
                trajectory.current_setpoint_vel = 0.0f;
                buffered_target_pos = 0.0f;
                
                pid_speed.integral = 0.0f;
                pid_speed.error_prev = 0.0f;
                pid_speed.d_filt = 0.0f;
                
                pid_position.integral = 0.0f;
                pid_position.error_prev = 0.0f;
                pid_position.d_filt = 0.0f;
            }
            a_button_is_held = false; 
            returning_home = false;
        }
        if (current_mode == MOTOR_MODE_SPEED) {
            if (jog_mode == JOG_FINE) {
                current_mode = MOTOR_MODE_STOPPED;
            } else {
                current_mode = MOTOR_MODE_POSITION; 
                trajectory.target_pos = encoder.current_position_deg;
                trajectory.current_setpoint_pos = encoder.current_position_deg;
                trajectory.current_setpoint_vel = 0.0f; 
            }
        }
        break;
    case 'Y': 
        if (current_mode == MOTOR_MODE_GHOST) {
            float rel_target = buffered_target_pos - encoder.current_position_deg;
            printf("START,%.1f\r\n", fabsf(rel_target)); 
            
            trajectory.target_pos = buffered_target_pos;
            ghost_buffer_idx = 0; 
            ghost_move_active = true;
            ghost_settle_start_tick = 0;
        }
        break;
    case 'J': 
        current_mode = MOTOR_MODE_POSITION; 
        trajectory.target_pos = encoder.current_position_deg; 
        break;
    default: break;
    }
}

void Motor_ControlLoop(void)
{
    Encoder_Update();
    target_position_deg = trajectory.target_pos;
    
    // M-button long press (3s) -> Test Mode
    if (m_button_active && current_mode != MOTOR_MODE_TEST && !emergency_stop) {
        if (HAL_GetTick() - m_button_hold_tick >= 3000) {
            printf("START\r\n");
            open_loop_test_tick = HAL_GetTick();
            current_mode = MOTOR_MODE_TEST;
            m_button_active = false;
        }
    }

    // Y-button long press (1s) -> Toggle Ghost Mode
    if (y_button_active && !emergency_stop) {
        if (HAL_GetTick() - y_button_hold_tick >= 1000) {
            if (current_mode == MOTOR_MODE_GHOST) {
                current_mode = MOTOR_MODE_POSITION;
            } else {
                current_mode = MOTOR_MODE_GHOST;
                buffered_target_pos = trajectory.target_pos;
            }
            y_button_active = false;
        }
    }

    // B-button long press (1s) -> Toggle Control Mode
    if (b_button_active && !emergency_stop) {
        if (HAL_GetTick() - b_button_hold_tick >= 1000) {
            if (control_system_mode == CONTROL_MODE_JOYSTICK) {
                control_system_mode = CONTROL_MODE_BASE_SYSTEM;
                printf("CONTROL: BASE_SYSTEM\r\n");
            } else {
                control_system_mode = CONTROL_MODE_JOYSTICK;
                printf("CONTROL: JOYSTICK\r\n");
            }
            b_button_active = false;
        }
    }

    // Handle autotune triggers
    if (autotune_trigger == ATUNE_POS) { 
        Motor_StartAutotune(); 
        autotune_trigger = ATUNE_IDLE; 
    } else if (autotune_trigger == ATUNE_SPEED) { 
        Motor_StartAutotuneSpeed(); 
        autotune_trigger = ATUNE_IDLE; 
    }

    // Handle return home trigger
    if (a_button_is_held && !returning_home) {
        if (HAL_GetTick() - a_press_tick >= HOME_HOLD_TIME_MS) {
            returning_home = true; 
            trajectory.target_pos = 0.0f; 
            current_mode = MOTOR_MODE_POSITION;
        }
    }

    // Joystick safety check: Timeout if no data received for 500ms
    if (control_system_mode == CONTROL_MODE_JOYSTICK) {
        if (HAL_GetTick() - joystick_watchdog_timer > 500) {
            is_joystick_connected = false;
        }

        if (!is_joystick_connected) {
            fault_code |= FAULT_JOYSTICK_LOST;
            if (current_mode != MOTOR_MODE_STOPPED) {
                current_mode = MOTOR_MODE_STOPPED;
                PWM_Apply(0.0f);
            }
        } else {
            fault_code &= ~FAULT_JOYSTICK_LOST;
        }
    }

    // Stop motor if e-stop or stopped mode
    if (emergency_stop || current_mode == MOTOR_MODE_STOPPED) { 
        if (ghost_move_active) {
            ghost_move_active = false;
            ghost_dump_requested = true;
        }
        PWM_Apply(0.0f); 
        return; 
    }

    /* --- Autotune: Position --- */
    if (current_mode == MOTOR_MODE_AUTOTUNE) {
        if (fabsf(encoder.current_position_deg - atune.center_pos) > 25.0f) { 
            emergency_stop = true; 
            autotune_status = STATUS_ERROR_LIMIT_EXCEEDED; 
            return; 
        }
        if (atune.cycle_count == -1) { 
            PWM_Apply(15.0f); 
            if (HAL_GetTick() - atune.last_flip_tick > 300) { 
                atune.motor_sign = ((encoder.current_position_deg - atune.center_pos) >= 0) ? 1 : -1;
                atune.cycle_count = 0; 
                atune.last_flip_tick = HAL_GetTick(); 
            } 
            return; 
        }
        bool crossed = (atune.direction && (encoder.current_position_deg > atune.center_pos)) || 
                       (!atune.direction && (encoder.current_position_deg < atune.center_pos));
        if (crossed) { 
            atune.direction = !atune.direction; 
            if (atune.cycle_count > 2) { 
                atune.amplitude_sum += (atune.peak_max - atune.peak_min); 
                atune.period_sum += (HAL_GetTick() - atune.last_flip_tick); 
            }
            atune.cycle_count++; 
            atune.last_flip_tick = HAL_GetTick(); 
            tuning_progress = atune.cycle_count; 
            atune.peak_max = -999.0f; 
            atune.peak_min = 999.0f; 
        }
        if (encoder.current_position_deg > atune.peak_max) atune.peak_max = encoder.current_position_deg;
        if (encoder.current_position_deg < atune.peak_min) atune.peak_min = encoder.current_position_deg;
        PWM_Apply((atune.direction ? atune.relay_output : -atune.relay_output) * atune.motor_sign);
        if (atune.cycle_count >= 10) { 
            float avg_A = (atune.amplitude_sum / (atune.cycle_count - 2)) / 2.0f; 
            if (avg_A > 0.1f) { 
                float Ku = (4.0f * atune.relay_output) / (3.14159f * avg_A);
                tuning.pos_Kp = 0.20f * Ku; 
                tuning.pos_Ki = 0.10f * tuning.pos_Kp; 
                tuning.pos_Kd = 0.01f;
                if (tuning.pos_Kp > 2.5f) tuning.pos_Kp = 2.5f; 
                autotune_status = STATUS_SUCCESS;
            } else { 
                autotune_status = STATUS_ERROR_LIMIT_EXCEEDED; 
            }
            current_mode = MOTOR_MODE_STOPPED;
        }
        return;
    }

    /* --- Autotune: Speed --- */
    if (current_mode == MOTOR_MODE_AUTOTUNE_SPEED) {
        if (fabsf(encoder.current_position_deg - atune.center_pos) > 60.0f) { 
            emergency_stop = true; 
            autotune_status = STATUS_ERROR_LIMIT_EXCEEDED; 
            return; 
        }
        if (atune.cycle_count == -1) { 
            PWM_Apply(20.0f); 
            if (HAL_GetTick() - atune.last_flip_tick > 300) { 
                atune.motor_sign = (encoder.filtered_rpm >= 0) ? 1 : -1; 
                atune.cycle_count = 0; 
                atune.last_flip_tick = HAL_GetTick(); 
            } 
            return; 
        }
        bool crossed = (atune.direction && (encoder.filtered_rpm > atune.target_val)) || 
                       (!atune.direction && (encoder.filtered_rpm < -atune.target_val));
        if (crossed) { 
            atune.direction = !atune.direction; 
            if (atune.cycle_count > 2) { 
                atune.amplitude_sum += (atune.peak_max - atune.peak_min); 
                atune.period_sum += (HAL_GetTick() - atune.last_flip_tick); 
            }
            atune.cycle_count++; 
            atune.last_flip_tick = HAL_GetTick(); 
            tuning_progress = atune.cycle_count; 
            atune.peak_max = -999.0f; 
            atune.peak_min = 999.0f; 
        }
        if (encoder.filtered_rpm > atune.peak_max) atune.peak_max = encoder.filtered_rpm;
        if (encoder.filtered_rpm < atune.peak_min) atune.peak_min = encoder.filtered_rpm;
        PWM_Apply((atune.direction ? atune.relay_output : -atune.relay_output) * atune.motor_sign);
        if (atune.cycle_count >= 15) { 
            float avg_A = (atune.amplitude_sum / (atune.cycle_count - 2)) / 2.0f; 
            if (avg_A > 0.5f) {
                float Ku = (4.0f * atune.relay_output) / (3.14159f * avg_A);
                tuning.speed_Kp = 0.20f * Ku; 
                tuning.speed_Ki = 0.50f * tuning.speed_Kp;
                if (tuning.speed_Kp > 6.0f) tuning.speed_Kp = 6.0f; 
                autotune_status = STATUS_SUCCESS;
            } else { 
                autotune_status = STATUS_ERROR_LIMIT_EXCEEDED; 
            }
            current_mode = MOTOR_MODE_STOPPED;
        }
        return;
    }
    
    // Update active PID gains
    pid_speed.Kp = tuning.speed_Kp; 
    pid_speed.Ki = tuning.speed_Ki; 
    pid_speed.Kd = tuning.speed_Kd;
    pid_position.Kp = tuning.pos_Kp; 
    pid_position.Ki = tuning.pos_Ki; 
    pid_position.Kd = tuning.pos_Kd;

    /* --- Open Loop Test Mode --- */
    if (current_mode == MOTOR_MODE_TEST) {
        if (HAL_GetTick() - open_loop_test_tick < 1000) {
            PWM_Apply(25.0f);
        } else {
            PWM_Apply(0.0f);
            printf("FINISH\r\n");
            tuning.move_speed_coarse = 5.0f;
            trajectory.target_pos = 0.0f;
            current_mode = MOTOR_MODE_POSITION;
        }
        return;
    }

    float current_applied_pwm = 0.0f;

    /* --- Velocity Loop --- */
    if (current_mode == MOTOR_MODE_SPEED) {
        float ff = tuning.speed_Kf * trajectory.target_vel;
        current_applied_pwm = PID_Compute(&pid_speed, trajectory.target_vel, encoder.filtered_rpm) + ff; 
        PWM_Apply(current_applied_pwm);
        trajectory.target_pos = encoder.current_position_deg; 
        trajectory.current_setpoint_pos = encoder.current_position_deg;
    } 
    /* --- Position Loop --- */
    else if (current_mode == MOTOR_MODE_POSITION || current_mode == MOTOR_MODE_GHOST) {
        Trajectory_Generator_Update();
        
        // Dynamic speed recovery
        if (tuning.move_speed_coarse < 100.0f && fabsf(encoder.current_position_deg) < 1.0f) {
            tuning.move_speed_coarse = 100.0f;
        }

        float target_rpm = PID_Compute(&pid_position, trajectory.current_setpoint_pos, encoder.current_position_deg);
        float ff = tuning.speed_Kf * target_rpm;
        current_applied_pwm = PID_Compute(&pid_speed, target_rpm, encoder.filtered_rpm) + ff; 
        PWM_Apply(current_applied_pwm);

        // Ghost Mode settle check
        if (ghost_move_active) {
            float pos_error = fabsf(encoder.current_position_deg - trajectory.target_pos);
            if (pos_error <= 1.0f) {
                if (ghost_settle_start_tick == 0) {
                    ghost_settle_start_tick = HAL_GetTick();
                } else if (HAL_GetTick() - ghost_settle_start_tick >= 500) {
                    ghost_move_active = false;
                    ghost_dump_requested = true;
                    ghost_settle_start_tick = 0;
                }
            } else {
                ghost_settle_start_tick = 0;
            }
        }
    }

    /* --- Safety Monitoring --- */
    bool stall_condition = false;

    // 1. Stall Detection
    if (fabsf(current_applied_pwm) >= STALL_PWM_THRESHOLD && fabsf(encoder.filtered_rpm) < STALL_VELOCITY_THRESHOLD) {
        if (current_mode == MOTOR_MODE_POSITION || current_mode == MOTOR_MODE_GHOST) {
            float pos_error = fabsf(trajectory.target_pos - encoder.current_position_deg);
            if (pos_error > STALL_SETTLING_ERROR_DEG) stall_condition = true;
        } else if (current_mode == MOTOR_MODE_SPEED) {
            stall_condition = true;
        }
    }

    // 2. Encoder Phase Inversion
    if ((current_applied_pwm > ENCODER_FAULT_PWM_THRESHOLD && encoder.filtered_rpm < -ENCODER_INVERSION_RPM_LIMIT) ||
        (current_applied_pwm < -ENCODER_FAULT_PWM_THRESHOLD && encoder.filtered_rpm > ENCODER_INVERSION_RPM_LIMIT)) {
        fault_code |= FAULT_ENCODER_ERROR;
        if (safety_enabled) {
            emergency_stop = true;
            printf("CRITICAL: ENCODER INVERTED / PHASE ERROR\r\n");
            PWM_Apply(0.0f);
            return;
        }
    }

    // 3. Encoder Signal Loss
    if (fabsf(current_applied_pwm) > ENCODER_FAULT_PWM_THRESHOLD && 
        fabsf(encoder.filtered_rpm) < STALL_VELOCITY_THRESHOLD &&
        encoder.absolute_counts == last_absolute_counts) {
        
        if (encoder_fault_timer == 0) encoder_fault_timer = HAL_GetTick();
        else if (HAL_GetTick() - encoder_fault_timer >= 1000) {
            fault_code |= FAULT_ENCODER_ERROR;
            if (safety_enabled) {
                emergency_stop = true;
                printf("CRITICAL: ENCODER DISCONNECTED / NO SIGNAL\r\n");
                PWM_Apply(0.0f);
                return;
            }
        }
    } else {
        encoder_fault_timer = 0;
        last_absolute_counts = encoder.absolute_counts;
    }

    // Stall Timer
    if (stall_condition) {
        if (stall_timer == 0) stall_timer = HAL_GetTick();
        else if (HAL_GetTick() - stall_timer >= STALL_TIME_MS) {
            fault_code |= FAULT_MOTOR_STALLED;
            if (safety_enabled) {
                emergency_stop = true;
                printf("CRITICAL: MOTOR STALLED\r\n");
                PWM_Apply(0.0f);
            }
        }
    } else {
        stall_timer = 0;
    }

    // 4. Over-Rotation Protection
    if (fabsf(encoder.current_position_deg) > SOFT_LIMIT_DEG) {
        fault_code |= FAULT_OVER_ROTATION;
        if (safety_enabled) {
            emergency_stop = true;
            printf("CRITICAL: SOFT LIMIT EXCEEDED (WIRE SNAP PROTECT)\r\n");
            PWM_Apply(0.0f);
        }
    }
}

float Motor_GetPosition(void) { return encoder.current_position_deg; }
float Motor_GetSpeed(void) { return encoder.filtered_rpm; }

/* ============================================================================
 * Gripper & Sequence Functions
 * ============================================================================ */

void Gripper_Up(void) { printf("Gripper: UP\r\n"); }
void Gripper_Down(void) { printf("Gripper: DOWN\r\n"); }
void Gripper_Open(void) { printf("Gripper: OPEN\r\n"); }
void Gripper_Close(void) { printf("Gripper: CLOSE\r\n"); }

void Gripper_Toggle(void)
{
    static bool is_open = true;
    if (is_open) Gripper_Close();
    else Gripper_Open();
    is_open = !is_open;
}

void Gripper_Sequence_Pick(void)
{
    printf("Starting Sequence: PICK\r\n");
    Gripper_Open();
    /* HAL_Delay(500);  -- REMOVED: Blocking delay in ISR context causes deadlocks */
    Gripper_Down();
    /* HAL_Delay(1000); -- REMOVED */
    Gripper_Close();
    /* HAL_Delay(500);  -- REMOVED */
    Gripper_Up();
    printf("Sequence PICK: Done\r\n");
}

void Gripper_Sequence_Place(void)
{
    printf("Starting Sequence: PLACE\r\n");
    Gripper_Down();
    Gripper_Open();
    Gripper_Up();
    Gripper_Close();
    printf("Sequence PLACE: Done\r\n");
}
