/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file GGB.h
* @brief Gravity Gradient Boom Driver Header File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Josh C.
* @version           1.2.0
* @date              2022.03.12
*
* @details           Driver for GGB motor controller
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/* OFFSET VALUES */
/*
FSAFE_FAULTS (0x0E)
MOTOR_A_INVERT (0x12)
LOCAL_MASTER_LOCK (0x15)
MST_E_IN_FN (0x17)
FSAFE_CTRL (0x1F)
MA_DRIVE (0x20)
DRIVER_ENABLE (0x70)
FSAFE_TIME (0x76)
CONTROL_1 (0x78)
U_I2C_RD_ERR (0x04)
U_I2C_WR_ERR (0x05)
UPDATE_RATE (0x71)
*/

#ifndef GGB_H
#define GGB_H

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <ctype.h>
#include <stdint.h>

#define GGB_MAX_EXTENSION (10.0f) // TODO - to be refined!

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Read data to GGB I2C interface and address.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      uint8_t offset - address of data to read [0x00 - 0x7F]
* @param[input]      uint8_t *rxBuffer - address of buffer to store returned byte
* @return            HAL_StatusTypeDef containing result of I2C comms

*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef GGB_read_register(uint8_t offset, uint8_t *rxBuffer);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Write data to GGB I2C interface and address.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      uint8_t offset - address of data to write [0x00 - 0x7F]
* @param[input]      uint8_t data - data to write
* @return            HAL_StatusTypeDef containing result of I2C comms

*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef GGB_write_register(uint8_t offset, uint8_t data);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Write FSAFE_FAULTS (0x0E) register: reset failsafe ISR hit count.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @return            HAL_StatusTypeDef containing result of I2C comms

*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef GGB_RESET_FSAFE_FAULTS();

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Write MOTOR_A_INVERT (0x12) register: change motor A's polarity.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      uint8_t polarity - direction of motor [0 (default), 1 (inverted)]
* @return            HAL_StatusTypeDef containing result of I2C comms

*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef GGB_WRITE_MOTOR_A_INVERT(uint8_t polarity);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Lock LOCAL_MASTER_LOCK (0x15) register: disable writes to read-only registers.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @return            HAL_StatusTypeDef containing result of I2C comms

*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef GGB_LOCK_LOCAL_MASTER();

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Unlock LOCAL_MASTER_LOCK (0x15) register: enable writes to read-only registers.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @return            HAL_StatusTypeDef containing result of I2C comms

*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef GGB_UNLOCK_LOCAL_MASTER();

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Write MST_E_IN_FN (0x17) register: set behavior when master SCMD's config_in
*        line goes high on the expansion bus.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      uint8_t restart_op - restart operation [0 (do nothing), 1 (reboot), 2
*                                         (re-enumerate)]
* @param[input]      uint8_t user_prt - user port [0 (do nothing), 1 (reinit user port)]
* @param[input]      uint8_t exp_prt - expansion port [0 (do nothing), 1 (reinit expansion
*                                      port)]
* @return            HAL_StatusTypeDef containing result of I2C comms

*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef GGB_WRITE_MST_E_IN_FN(uint8_t restart_op, uint8_t user_prt, uint8_t exp_prt);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Write FSAFE_CTRL (0x1F) register: set behavior when failsafe occurs
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      uint8_t output - output [0 (maintain last motor drive levels), 1
*                                     (center output drive levels for 0 drive)]
* @param[input]      uint8_t restart_op - restart operation [0 (do nothing), 1 (reboot), 2
*                                         (re-enumerate)]
* @param[input]      uint8_t user_prt - user port [0 (do nothing), 1 (reinit user port)]
* @param[input]      uint8_t exp_prt - expansion port [0 (do nothing), 1 (reinit expansion
*                                      port)]
* @return            HAL_StatusTypeDef containing result of I2C comms

*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef GGB_WRITE_FSAFE_CTRL(uint8_t output, uint8_t restart_op, uint8_t user_prt, uint8_t exp_prt);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Write MA_DRIVE (0x20) register: set motor 0's (master A channel) drive direction
*        and level.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      uint8_t direction - motor direction [0 (backward), 1 (forward)]
* @param[input]      uint8_t level - motor level [0 - 255]
* @return            HAL_StatusTypeDef containing result of I2C comms

*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef GGB_WRITE_MA_DRIVE(uint8_t direction, uint8_t level);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Write DRIVER_ENABLE (0x70) register: enable all drivers.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @return            HAL_StatusTypeDef containing result of I2C comms

*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef GGB_ENABLE_DRIVERS();

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Write DRIVER_ENABLE (0x70) register: disable all drivers.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @return            HAL_StatusTypeDef containing result of I2C comms

*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef GGB_DISABLE_DRIVERS();

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Write CONTROL_1 (0x78) register: set low level system control.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      uint8_t restart - restart program [0 (disable), 1 (enable)]
* @param[input]      uint8_t reenum_slaves - re-enumerate slaves [0 (disable), 1 (enable)]
* @return            HAL_StatusTypeDef containing result of I2C comms

*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef GGB_WRITE_CONTROL_1(uint8_t restart, uint8_t reenum_slaves);

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Write U_I2C_RD_ERR (0x04) register: reset number of read errors.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @return            HAL_StatusTypeDef containing result of I2C comms

*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef GGB_RESET_U_I2C_RD_ERR();

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Write U_I2C_WR_ERR (0x05) register: reset number of write errors.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @return            HAL_StatusTypeDef containing result of I2C comms

*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef GGB_RESET_U_I2C_WR_ERR();

/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @brief Write UPDATE_RATE (0x71) register: set motor update rate (in ms)
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @param[input]      uint8_t time - time (ms) in between each motor update [0x00 (0ms) -
*                    0xFF (255ms)]
* @return            HAL_StatusTypeDef containing result of I2C comms

*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef GGB_WRITE_UPDATE_RATE(uint8_t time);

#endif
