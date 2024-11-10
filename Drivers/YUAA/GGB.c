/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file GGB.c
* @brief Gravity Gradient Boom Driver C File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Josh C.
* @version           1.2.0
* @date              2022.03.12
*
* @details           FSAFE_FAULTS (0x0E)
*                    MOTOR_A_INVERT (0x12)
*                    LOCAL_MASTER_LOCK (0x15)
*                    MST_E_IN_FN (0x17)
*                    FSAFE_CTRL (0x1F)
*                    MA_DRIVE (0x20)
*                    DRIVER_ENABLE (0x70)
*                    FSAFE_TIME (0x76)
*                    CONTROL_1 (0x78)
*                    U_I2C_RD_ERR (0x04)
*                    U_I2C_WR_ERR (0x05)
*                    UPDATE_RATE (0x71)
*
* @history
*
* @endhistory
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

#include "GGB.h"
#include "GGB_config.h"
#include "PeriphHelper.h"
#include "system_manager.h"
#include <stdio.h>
#include <ctype.h>
#include <stdint.h>

HAL_StatusTypeDef GGB_read_register(uint8_t offset, uint8_t *rxBuffer)
{
    assert(rxBuffer);
    Periph_BeginTransact(DEV_GGB_CTRL, 1 + GGB_I2C_RX_BUFFER_LENGTH, HAL_MAX_DELAY);
    // Transmit bytes
    HAL_StatusTypeDef result = Periph_Tx(DEV_GGB_CTRL, &offset, 1, GGB_I2C_TIMEOUT);
    if (result != HAL_OK) goto end;
    // Receive data
    result = Periph_Rx(DEV_GGB_CTRL, rxBuffer, GGB_I2C_RX_BUFFER_LENGTH, GGB_I2C_TIMEOUT);
end:
    Periph_EndTransact(DEV_GGB_CTRL);
    return result;
}

HAL_StatusTypeDef GGB_write_register(uint8_t offset, uint8_t data)
{
    // Write the byte
    uint8_t txBuffer[2];

    txBuffer[0] = offset;
    txBuffer[1] = data;

    // Transmit bytes
    Periph_BeginTransact(DEV_GGB_CTRL, 2, HAL_MAX_DELAY);
    HAL_StatusTypeDef retstat = Periph_Tx(DEV_GGB_CTRL, txBuffer, 2, GGB_I2C_TIMEOUT);
    Periph_EndTransact(DEV_GGB_CTRL);
    return retstat;
}

HAL_StatusTypeDef GGB_RESET_FSAFE_FAULTS()
{
    return GGB_write_register(GGB_FSAFE_FAULTS, 0x0);
}

HAL_StatusTypeDef GGB_WRITE_MOTOR_A_INVERT(uint8_t polarity)
{
    return GGB_write_register(GGB_MOTOR_A_INVERT, polarity);
}

HAL_StatusTypeDef GGB_LOCK_LOCAL_MASTER()
{
    return GGB_write_register(GGB_LOCAL_MASTER_LOCK, 0x0);
}

HAL_StatusTypeDef GGB_UNLOCK_LOCAL_MASTER()
{
    return GGB_write_register(GGB_LOCAL_MASTER_LOCK, 0x9B);
}

HAL_StatusTypeDef GGB_WRITE_MST_E_IN_FN(uint8_t restart_op, uint8_t user_prt, uint8_t exp_prt)
{
    // Combine flags into 8-bit package
    uint8_t package = ((exp_prt & 0x1) << 3) | ((user_prt & 0x1) << 2) | (restart_op & 0x3);
    return GGB_write_register(GGB_MST_E_IN_FN, package);
}

HAL_StatusTypeDef GGB_WRITE_FSAFE_CTRL(uint8_t output, uint8_t restart_op, uint8_t user_prt, uint8_t exp_prt)
{
    // Combine flags into 8-bit package
    uint8_t package = ((exp_prt & 0x1) << 4) | ((user_prt & 0x1) << 3) | ((restart_op & 0x3) << 1) | (output & 0x1);
    return GGB_write_register(GGB_FSAFE_CTRL, package);
}

HAL_StatusTypeDef GGB_WRITE_MA_DRIVE(uint8_t direction, uint8_t level)
{
    // Convert to 7 bit
	level = level >> 1;
    // Use to build value to actually write to register
	int16_t drive_value;
    // Set to 1/2 drive if direction = 1 or -1/2 drive if direction = 0
    drive_value = (level * direction) + ((int8_t)level * ((int8_t)direction - 1));
    // Add offset to center values
    drive_value += 128;

    return GGB_write_register(GGB_MA_DRIVE, drive_value);
}

HAL_StatusTypeDef GGB_ENABLE_DRIVERS()
{
    return GGB_write_register(GGB_DRIVER_ENABLE, 0x1);
}

HAL_StatusTypeDef GGB_DISABLE_DRIVERS()
{
    return GGB_write_register(GGB_DRIVER_ENABLE, 0x0);
}

HAL_StatusTypeDef GGB_WRITE_CONTROL_1(uint8_t restart, uint8_t reenum_slaves)
{
    // Combine flags into 8-bit package
    uint8_t package = ((reenum_slaves & 0x1) << 1) | (restart & 0x1);
    return GGB_write_register(GGB_CONTROL_1, package);
}

HAL_StatusTypeDef GGB_RESET_U_I2C_RD_ERR()
{
    return GGB_write_register(GGB_U_I2C_RD_ERR, 0x0);
}

HAL_StatusTypeDef GGB_RESET_U_I2C_WR_ERR()
{
    return GGB_write_register(GGB_U_I2C_WR_ERR, 0x0);
}

HAL_StatusTypeDef GGB_WRITE_UPDATE_RATE(uint8_t time)
{
    return GGB_write_register(GGB_UPDATE_RATE, time);
}
