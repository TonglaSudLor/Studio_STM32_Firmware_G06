/**
 * @file modbus_bridge.c
 * @brief Application-level Modbus register mapping and logic
 * 
 * @author Gemini CLI (Refactored)
 * @date May 2026
 */

#include "modbus_bridge.h"
#include "modbus_rtu.h"
#include "motor_controller.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

extern UART_HandleTypeDef hlpuart1;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim3;

/* --- Modbus Configuration --- */
#define MODBUS_SLAVE_ID     21
#define MODBUS_REG_COUNT    128

/* --- Global Modbus Handle --- */
static Modbus_Handle_t hmodbus;
static Modbus_Register_t register_frame[MODBUS_REG_COUNT];
static bool modbus_initialized = false;

/* --- Debug Log --- */
static uint8_t debug_rx_log[8] = {0};
static uint8_t debug_rx_idx = 0;

/* ============================================================================
 * Internal Register Handlers
 * ============================================================================ */

/**
 * @brief Process writes from Modbus Master (PC)
 */
static void ModbusBridge_HandleCommands(void)
{
    if (!modbus_initialized) return;

    // 0x01: Operating Mode Bits
    if (register_frame[0x01].U16 & 0x01) { // Home (Move to 0)
        Motor_MoveToPosition(0.0f);
        register_frame[0x01].U16 &= ~0x01;
    }
    
    // bit 1 (value 2) = Jog / Manual Gripper Trigger
    if (register_frame[0x01].U16 & 0x02) {
        uint16_t val = register_frame[0x02].U16;
        if      (val == 0) Gripper_Up();
        else if (val == 1) Gripper_Down();
        else if (val == 2) Gripper_Open();
        else if (val == 4) Gripper_Close();
        
        current_mode = MOTOR_MODE_POSITION;
        register_frame[0x01].U16 &= ~0x02;
        register_frame[0x02].U16 = 0;
    }

    if (register_frame[0x01].U16 & 0x04) { // Auto mode
        current_mode = MOTOR_MODE_POSITION;
        register_frame[0x01].U16 &= ~0x04;
    }
    if (register_frame[0x01].U16 & 0x08) { // Set Home (Reset Encoder)
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        encoder.absolute_counts = 0;
        encoder.current_position_deg = 0.0f;
        trajectory.target_pos = 0.0f;
        trajectory.current_setpoint_pos = 0.0f;
        register_frame[0x01].U16 &= ~0x08;
    }
    if (register_frame[0x01].U16 & 0x10) { // Test mode
        current_mode = MOTOR_MODE_TEST;
        register_frame[0x01].U16 &= ~0x10;
    }

    // 0x03: Gripper Sequence
    if (register_frame[0x03].U16 != 0) {
        uint16_t val = register_frame[0x03].U16;
        if (val == 1) Gripper_Sequence_Pick();
        else if (val == 2) Gripper_Sequence_Place();
        register_frame[0x03].U16 = 0;
    }

    // 0x05: Jog step
    if (register_frame[0x05].U16 != 0) {
        float step = (float)((int16_t)register_frame[0x05].U16);
        Motor_MoveToPosition(encoder.current_position_deg + step);
        register_frame[0x05].U16 = 0;
    }

    // 0x24: Point-to-Point Target
    if (register_frame[0x24].U16 != 0) {
        Motor_MoveToPosition((float)((int16_t)register_frame[0x24].U16));
        register_frame[0x24].U16 = 0;
    }

    // 0x25: Safety / Soft Stop
    if (register_frame[0x25].U16 & 0x01) {
        emergency_stop = true;
        register_frame[0x25].U16 &= ~0x01;
    } else if (register_frame[0x25].U16 & 0x02) {
        emergency_stop = false;
        register_frame[0x25].U16 &= ~0x02;
    }
}

/* ============================================================================
 * Public API
 * ============================================================================ */

void ModbusBridge_Init(void)
{
    memset(register_frame, 0, sizeof(register_frame));
    
    // 0x00: Device ID / Heartbeat ("YA")
    register_frame[0].U16 = 22881; 
    
    // Initialize RTU Handle
    hmodbus.huart = &hlpuart1;
    hmodbus.htim = &htim16;
    hmodbus.slave_address = MODBUS_SLAVE_ID;
    hmodbus.register_count = MODBUS_REG_COUNT;
    
    Modbus_Init(&hmodbus, register_frame);
    modbus_initialized = true;
}

void ModbusBridge_Process(void)
{
    // 1. Process Command Writes from Master
    ModbusBridge_HandleCommands();
    
    // 2. Run Modbus Protocol Engine
    Modbus_Process(&hmodbus);
}

void ModbusBridge_UpdateRegisters(void)
{
    // Sync internal state to Modbus Read-Only registers
    
    // 0x28: Current Position (scale 10.0 for UI)
    register_frame[0x28].U16 = (int16_t)(Motor_GetPosition() * 10.0f);
    
    // 0x29: Current Speed (scale 10.0 for UI)
    register_frame[0x29].U16 = (int16_t)(Motor_GetSpeed() * 10.0f);
    
    // 0x31: Safety Status
    register_frame[0x31].U16 = emergency_stop ? 1 : 0;
    
    // 0x27: Task Status Bits
    uint16_t task_bits = 0;
    if (current_mode == MOTOR_MODE_POSITION) task_bits |= (1 << 1);
    if (ghost_move_active) task_bits |= (1 << 2);
    register_frame[0x27].U16 = task_bits;

    // 0x26: Reed sensors — hardcode 0 for now (no reed sensor hardware yet)
    register_frame[0x26].U16 = 0;

    // 0x30: Acceleration — hardcode 0 for now (no acceleration measurement yet)
    register_frame[0x30].U16 = 0;
}

void ModbusBridge_RxCallback(uint8_t data)
{
    // Log for debugging
    debug_rx_log[debug_rx_idx % 8] = data;
    debug_rx_idx++;

    if (hmodbus.uart.rx_tail < MODBUS_BUFFER_SIZE) {
        hmodbus.uart.rx_buffer[hmodbus.uart.rx_tail++] = data;
    }
    
    hmodbus.state = MODBUS_STATE_RECEPTION;
    
    // Reset T3.5 Timer
    __HAL_TIM_SET_COUNTER(hmodbus.htim, 0);
    HAL_TIM_Base_Start_IT(hmodbus.htim);
}

void ModbusBridge_TimerCallback(void)
{
    hmodbus.flag_t35_timeout = 1;
    HAL_TIM_Base_Stop_IT(hmodbus.htim);
}