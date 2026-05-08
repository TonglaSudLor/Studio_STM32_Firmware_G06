/**
 * @file modbus_rtu.h
 * @brief Modbus RTU Slave Protocol Implementation
 * 
 * @author Gemini CLI (Refactored)
 * @date May 2026
 */

#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <string.h>

#define MODBUS_BUFFER_SIZE 300

/**
 * @brief Modbus register representation (16-bit union)
 */
typedef union {
    uint16_t U16;
    uint8_t U8[2];
} Modbus_Register_t;

/**
 * @brief Modbus state machine states
 */
typedef enum {
    MODBUS_STATE_INIT,
    MODBUS_STATE_IDLE,
    MODBUS_STATE_RECEPTION,
    MODBUS_STATE_PROCESSING,
    MODBUS_STATE_EMISSION
} Modbus_State_t;

/**
 * @brief Supported Modbus function codes
 */
typedef enum {
    MODBUS_FUNC_READ_HOLDING_REG = 0x03,
    MODBUS_FUNC_WRITE_SINGLE_REG = 0x06
} Modbus_FunctionCode_t;

/**
 * @brief Modbus frame status and exception codes
 */
typedef enum {
    MODBUS_STATUS_NULL = -2,
    MODBUS_STATUS_FRAME_ERROR = -1,
    MODBUS_STATUS_NORMAL = 0,
    MODBUS_STATUS_ILLEGAL_FUNCTION = 0x01,
    MODBUS_STATUS_ILLEGAL_DATA_ADDRESS = 0x02,
    MODBUS_STATUS_ILLEGAL_DATA_VALUE = 0x03,
    MODBUS_STATUS_SLAVE_DEVICE_FAILURE = 0x04
} Modbus_Status_t;

/**
 * @brief Internal UART buffer management
 */
typedef struct {
    uint8_t rx_buffer[MODBUS_BUFFER_SIZE];
    uint16_t rx_tail;
    uint8_t tx_buffer[MODBUS_BUFFER_SIZE];
    uint16_t tx_tail;
} Modbus_Uart_t;

/**
 * @brief Modbus Protocol Handle
 */
typedef struct {
    uint8_t slave_address;
    Modbus_Register_t *registers;
    uint32_t register_count;

    UART_HandleTypeDef* huart;
    TIM_HandleTypeDef* htim;

    uint8_t flag_t15_timeout;
    uint8_t flag_t35_timeout;

    Modbus_Status_t status;
    Modbus_State_t state;

    uint8_t rx_frame[MODBUS_BUFFER_SIZE];
    uint8_t tx_frame[MODBUS_BUFFER_SIZE];
    uint8_t tx_count;

    Modbus_Uart_t uart;
} Modbus_Handle_t;

/* --- Public API --- */

/**
 * @brief Initialize Modbus handle
 * @param hmodbus Pointer to handle
 * @param reg_start Pointer to register map start
 */
void Modbus_Init(Modbus_Handle_t* hmodbus, Modbus_Register_t* reg_start);

/**
 * @brief Main protocol processing worker
 * @param hmodbus Pointer to handle
 */
void Modbus_Process(Modbus_Handle_t* hmodbus);

#endif /* MODBUS_RTU_H */