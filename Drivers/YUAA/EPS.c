/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file EPS.c
* @brief EPS drivers
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            AJ N.
* @version           0.2.0
* @date              2022.08.31
*
* @details           Defines EPS drivers
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "EPS.h"
#include "PeriphHelper.h"
#include "system_manager.h"
#include "stm32f4xx_hal.h"

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#define EPS_DEFAULT_TIMEOUT 10 //Default I2C timeout in ms
#define errorcheck(x, lbl) if ((HAL_StatusTypeDef)(x) != HAL_OK) goto lbl

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef EPS_R_Cmd(uint8_t cmd, uint16_t* return_args)
{
    assert(return_args);
    uint8_t temp_reg_buff[2];
    Periph_BeginTransact(DEV_EPS, 3, HAL_MAX_DELAY);
    HAL_StatusTypeDef retstat = Periph_Tx(DEV_EPS, &cmd, 1, EPS_DEFAULT_TIMEOUT); errorcheck(retstat, end);
    retstat = Periph_Rx(DEV_EPS, temp_reg_buff, 2, EPS_DEFAULT_TIMEOUT); errorcheck(retstat, end);
    *return_args = (temp_reg_buff[0] << 8) | temp_reg_buff[1];
end:
    Periph_EndTransact(DEV_EPS);
    return retstat;
}

HAL_StatusTypeDef EPS_W_Cmd(uint8_t cmd, EPS_W_e val)
{
    uint8_t data[2]; data[0] = cmd; data[1] = (uint8_t)val;
    Periph_BeginTransact(DEV_EPS, 3, HAL_MAX_DELAY);
    HAL_StatusTypeDef retstat = Periph_Tx(DEV_EPS, data, 2, EPS_DEFAULT_TIMEOUT);
    Periph_EndTransact(DEV_EPS);
    return retstat;
}

HAL_StatusTypeDef EPS_R_Voltages(EPS_VoltageType* retval)
{
    assert(retval);
    HAL_StatusTypeDef retstat;
    
    uint16_t Vbatt = 0;
    retstat = EPS_R_Cmd(1, &Vbatt); if (retstat != HAL_OK) return retstat;

    uint16_t V_3v3 = 0;
    retstat = EPS_R_Cmd(49, &V_3v3); if (retstat != HAL_OK) return retstat;

    uint16_t V_5v = 0;
    retstat = EPS_R_Cmd(50, &V_5v); if (retstat != HAL_OK) return retstat;
    
    retval->V_Batt = Vbatt * (0.0023394775);
    retval->V_3V3 = V_3v3 * (0.0023394775);
    retval->V_5V = V_5v * (0.0023394775);
    return retstat;
}

HAL_StatusTypeDef EPS_R_Currents(EPS_CurrentType* retval)
{
    assert(retval);
    HAL_StatusTypeDef retstat;

    uint16_t Ibatt = 0;
    retstat = EPS_R_Cmd(2, &Ibatt); if (retstat != HAL_OK) return retstat;

    uint16_t I_3v3 = 0;
    retstat = EPS_R_Cmd(14, &I_3v3); if (retstat != HAL_OK) return retstat;

    uint16_t I_5v = 0;
    retstat = EPS_R_Cmd(15, &I_5v); if (retstat != HAL_OK) return retstat;

    retval->I_Batt = Ibatt * (0.0030517578);
    retval->I_3V3 = I_3v3 * (0.0020345052);
    retval->I_5V = I_5v * (0.0020345052);
    return retstat;
}

HAL_StatusTypeDef EPS_R_Panels(EPS_PanelType* retval)
{
    assert(retval);
    HAL_StatusTypeDef retstat;
    char cmdNo; unsigned char i;

    uint16_t voltages[3];
    for (cmdNo = 5, i=0; cmdNo < 12; cmdNo += 3, i++)
    {
        retstat = EPS_R_Cmd(cmdNo, &(voltages[i])); if (retstat != HAL_OK) return retstat;
    }

    uint16_t currents[6];
    for (cmdNo = 6, i = 0; cmdNo < 14; cmdNo += 3, i += 2)
    {
        retstat = EPS_R_Cmd(cmdNo, &(currents[i])); if (retstat != HAL_OK) return retstat;
        retstat = EPS_R_Cmd(cmdNo+1, &(currents[i+1])); if (retstat != HAL_OK) return retstat;
    }

    retval->X_V = (voltages[0] * (0.0024414063));
    retval->Y_V = (voltages[1] * (0.0024414063));
    retval->Z_V = (voltages[2] * (0.0024414063));

    retval->X_minusI = (currents[0] * (0.0006103516));
    retval->X_plusI = (currents[1] * (0.0006103516));
    retval->Y_minusI = (currents[2] * (0.0006103516));
    retval->Y_plusI = (currents[3] * (0.0006103516));
    retval->Z_minusI = (currents[4] * (0.0006103516));
    retval->Z_plusI = (currents[5] * (0.0006103516));
    return retstat;
}

HAL_StatusTypeDef EPS_R_Temps(EPS_TemperatureType* retval)
{
    assert(retval);
    HAL_StatusTypeDef retstat;

    uint16_t MCUt = 0;
    retstat = EPS_R_Cmd(18, &MCUt); if (retstat != HAL_OK) return retstat;

    uint16_t Batt_t[2];
    char cmd; unsigned char i;
    for (cmd = 19, i = 0; cmd <= 20; cmd++, i++)
    {
        retstat = EPS_R_Cmd(cmd, &(Batt_t[i])); if (retstat != HAL_OK) return retstat;
    }

    retval->MCU = ( MCUt * (2.5 / 4096) - 0.986 ) / 0.00355;
    for (i = 0; i < 2; i++)
    {
        //Positive temperature
        if (Batt_t[i] < 0x8000) retval->Batt[i] = (Batt_t[i] * 0.00390625);
        //Negative temperature
        else
        {
            Batt_t[i] = ~( (Batt_t[i] >> 4) - 1 );
            retval->Batt[i] = Batt_t[i] * (-0.0624);
        }
    }
    return retstat;
}

HAL_StatusTypeDef EPS_R_OUT(uint8_t* retval)
{
    assert(retval);
    HAL_StatusTypeDef retstat; uint16_t reg24;
    retstat = EPS_R_Cmd(24, &reg24); if (retstat != HAL_OK) return retstat;
    uint8_t result = reg24 >> 6; //Shift so OUT 1 becomes bit 1
    result = result & 0b01111110; //Force LSb and MSb to zero
    *retval = result; return retstat;
}

HAL_StatusTypeDef EPS_R_Heaters(uint8_t* retval)
{
    assert(retval);
    HAL_StatusTypeDef retstat; uint16_t reg24;
    retstat = EPS_R_Cmd(24, &reg24); if (retstat != HAL_OK) return retstat;
    uint8_t result = reg24 >> 12; //Shift so Heater 1 becomes bit 1 
    result = result & 0b00001110; //Force unnecessary bits to zero
    *retval = result; return retstat;
}

HAL_StatusTypeDef EPS_R_ChgMode(EPS_ChgMode_e* retval)
{
    assert(retval);
    HAL_StatusTypeDef retstat; uint16_t reg25;
    retstat = EPS_R_Cmd(25, &reg25); if (retstat != HAL_OK) return retstat;
    uint8_t result = (reg25 >> 3) & 0b11; //Shift so fast charge flags become LSbits and eliminate everything else
    if (result == 0b10) result = 0b01;
    *retval = result; return retstat;
}

HAL_StatusTypeDef EPS_R_ErrorCt(EPS_ErrorCtType* retval)
{
    assert(retval);
    HAL_StatusTypeDef retstat;
    retstat = EPS_R_Cmd(26, &(retval->pwrCycles)); if (retstat != HAL_OK) return retstat;
    retstat = EPS_R_Cmd(27, &(retval->undervolt)); if (retstat != HAL_OK) return retstat;
    retstat = EPS_R_Cmd(28, &(retval->shortCircuit)); if (retstat != HAL_OK) return retstat;
    retstat = EPS_R_Cmd(29, &(retval->overTemp)); if (retstat != HAL_OK) return retstat;
    return retstat;
}

