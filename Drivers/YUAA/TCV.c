/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file TCV.c
* @brief UHF transceiver drivers C File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Elijah B.
* @version           0.1.0
* @date              2023.01.18
*
* @details           Defines drivers for the EnduroSat UHF Transceiver Type II
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "TCV.h"
#include "es_crc32.h"
#include "main.h"
#include "MCU_Init.h"
#include "PeriphHelper.h"
#include "system_manager.h"
#include "FreeRTOS.h"
#include "task.h"
#include <ctype.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// Maximum number of times an ESTTC command with an invalid CRC will be retried before
// giving up
#define TCV_MAX_ATTEMPTS (3) 

// If defined, disable CRC in transmitted commands,
// and don't check CRC in received commands
#define TCV_NO_CRC

#ifndef TCV_NO_CRC
#define TCV_SUFFIX_LEN (1+8+1) // Length of [B][C..C]<CR>
#else
#define TCV_SUFFIX_LEN (1)
#endif
#define TCV_PREFIX_LEN (3+1+2+2) // Length of ES+[R/W][AA][ID]
#define ESTTC_MAX_ERR_LEN (13+TCV_SUFFIX_LEN) // Length of longest error response, which is `E_CRC_ERR_LEN[B][C..C]<CR>' 
// ^^^ The CRC is appended onto the intrinsic error responses like with normal responses although this is not explicitly stated in the manual

// Size of internal buffer for TCV replies. This size MUST be at least 
// ESTTC_MAX_ERR_LEN + 1 AND at least the maximum TCV reply size including 
// the [B][C..C]<CR> suffix + 1
#define TCV_RXBUF_SZ (16 + TCV_SUFFIX_LEN + 1)
// Size of internal buffer for transmissions to TCV. This size MUST be at least
// TCV_PREFIX_LEN + maximum W command data length + TCV_SUFFIX_LEN + 1
// Current maximum supported W command data length is 2 + TCV_MAX_BEACON_LEN
#define TCV_TXBUF_SZ (TCV_PREFIX_LEN + 2 + TCV_MAX_BEACON_LEN + TCV_SUFFIX_LEN + 1)

#define TCV_I2C_DELAY (200) //Manual says max delay is 150ms

#define BUF_APPEND_FORMAT(buf, buf_pt, buf_sz, format,...) buf_pt += snprintf(buf+buf_pt, buf_sz-buf_pt, format, __VA_ARGS__);
#define SCW_GET_HFXT(SCW) ((SCW >> 14) & 1)
#define SCW_GET_FRAM(SCW) ((SCW >> 1) & 1)
#define SCW_GET_RFTS(SCW) (SCW & 1)

#define errorcheck_goto(HAL_retstat, lbl) if ((HAL_StatusTypeDef)(HAL_retstat) != HAL_OK) goto lbl
#define errorcheck_ret(HAL_retstat) if ((HAL_StatusTypeDef)(HAL_retstat) != HAL_OK) return HAL_retstat

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL VARIABLES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
static char S_rxBuf[TCV_RXBUF_SZ];
static char S_txBuf[TCV_TXBUF_SZ];

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL ROUTINES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
// Helper method that handles ESTTC commands to TCV. 
// Assumes S_txBuf already contains the command body of length cmdBodyLen,
// and will append the [B][C..C]<CR> suffix before transmitting the command. 
// Will save the reply of length at most answerMaxLen + TCV_SUFFIX_LEN in S_rxBuf and perform CRC checking, 
// after which will cut off the suffix.
// When this method returns, S_rxBuf contains the null-terminated response without the suffix.
// Begin a transaction BEFORE calling this!
HAL_StatusTypeDef TCV_ESTTC(uint8_t cmdBodyLen, uint8_t answerMaxLen)
{
#ifdef DEBUG_ENABLED
    assert(cmdBodyLen + TCV_SUFFIX_LEN < TCV_TXBUF_SZ);
#endif
#if !defined(DEBUG_ENABLED) || !defined(TCV_NO_CRC)

    // EnduroSat did something very strange with the "read temperature" command 0x0A.
    // For some weird reason the CRC calculation on the response
    // is not consistent with how CRC is calculated for every other command.
    // So an exception is needed for this particular command.
    uint8_t expectTempResponse = 0;
    if (strncmp(S_txBuf + 6, "0A", 2) == 0) expectTempResponse = 1;
    // Exception for kill command - if the TCV's address is changed,
    // it will not acknowledge an I2C read request on the old address,
    // so an AF error on reading the response should be ignored

    // Append suffix
    sprintf(S_txBuf + cmdBodyLen, " %08lX", crc32(CRC32_INIT_REMAINDER, (BYTE*)S_txBuf, cmdBodyLen));
#endif
    sprintf(S_txBuf + cmdBodyLen + TCV_SUFFIX_LEN - 1, "\r");
    uint16_t reply_len = ESTTC_MAX_ERR_LEN >= answerMaxLen + TCV_SUFFIX_LEN ? ESTTC_MAX_ERR_LEN : answerMaxLen + TCV_SUFFIX_LEN;
#ifdef DEBUG_ENABLED
    assert(reply_len < TCV_RXBUF_SZ);
#endif // DEBUG_ENABLED

    for (uint8_t attempts = 1; attempts <= TCV_MAX_ATTEMPTS; attempts++)
    {
#ifdef DEBUG_ENABLED
        memset(S_rxBuf, 0, TCV_RXBUF_SZ);
#endif // DEBUG_ENABLED

        // Transmit command
        HAL_StatusTypeDef w_stat = Periph_Tx(DEV_TCV, (uint8_t*)S_txBuf, cmdBodyLen + TCV_SUFFIX_LEN, TCV_I2C_DELAY);
        errorcheck_ret(w_stat);

        // Receive reply

        // For command 0xf2 antenna release config, the manual says "In case the
        // command is sent via I2C the user may not get an appropriate reply as the UHF Transceiver will switch to
        // I2C master mode." I assume that
        // this only applies if 0xf2 is used to immediately deploy the antenna, 
        // but we're always setting a timer, so we don't care about this.
        // TODO: test if this is true. 

        HAL_StatusTypeDef r_stat = Periph_Rx(DEV_TCV, (uint8_t*)S_rxBuf, reply_len, TCV_I2C_DELAY);
        errorcheck_ret(r_stat);
        S_rxBuf[reply_len] = '\0';

        // Check reply CRC
        // First we need to find out how long the reply was. For this we find the final <CR> and backtrack from it.
         char* CR = strchr(S_rxBuf, '\r');
         if (CR == NULL)
        {
#ifdef DEBUG_ENABLED
            debug_L2("%s", "No CR found in TCV response");
#endif
            continue;
        }
		int16_t reply_body_len = (intptr_t)(CR - TCV_SUFFIX_LEN - S_rxBuf + 1);
        if (reply_body_len < 2)
        {
#ifdef DEBUG_ENABLED
            debug_L2("%s", "TCV response too short");
#endif
            continue;
		}
#if !defined(DEBUG_ENABLED) || !defined(TCV_NO_CRC)
        uint32_t my_reply_crc;
        if (!expectTempResponse) my_reply_crc = crc32(CRC32_INIT_REMAINDER, (BYTE*)S_rxBuf, reply_body_len);
        else my_reply_crc = crc32(CRC32_INIT_REMAINDER, (BYTE*)S_rxBuf + 4, 4);
        uint32_t reply_crc;
        if (sscanf(S_rxBuf + reply_body_len + 1, "%8lx", &reply_crc) != 1)
        {
#ifdef DEBUG_ENABLED
            debug_L2("%s", "No valid CRC found in TCV response");
#endif
            continue;
        }
        if (my_reply_crc != reply_crc)
        {
#ifdef DEBUG_ENABLED
            debug_L2("%s", "TCV CRC doesn't match");
#endif
            continue;
        }
#endif
        S_rxBuf[reply_body_len] = '\0'; // Erase suffix

        // Check for intrinsic reply
        if (strncmp(S_rxBuf, "ERR", 3) == 0 || strncmp(S_rxBuf, "E_", 2) == 0)
        {
            // CRC or command length error. 
#ifdef DEBUG_ENABLED
            debug_L2("%s", S_rxBuf);
#endif
            continue;
        }
        // Commented because this response should never be received by the OBC when communicating to the TCV over I2C
        /*else if (strncmp(reply, "I2C_NACK", 8) == 0)
        {

        }*/
#ifdef DEBUG_ENABLED
        else if (strncmp(S_rxBuf, "OK", 2) != 0)
        {
            // Reply doesn't match any valid reply type, but CRC was intact
            // This really shouldn't ever happen.
            panic("%s", "Invalid TCV reply with intact CRC");
        }
#endif // DEBUG_ENABLED

        else return HAL_OK;
    }
    // If you're ever out of the loop, it means something went wrong
    return HAL_ERROR;
}

// Helper to update the status control word using command 0x00.
// Will read the current SCW, 
// and then write the new SCW, updating the bits indicated in mask
// to the values indicated in newval.
// IMPORTANT: uses its own transaction
HAL_StatusTypeDef UpdateSCW(uint16_t mask, uint16_t newval)
{
    // First read the SCW, check for errors, and save the current parameters
    Periph_BeginTransact(DEV_TCV, 13 + 7, HAL_MAX_DELAY);
    char currentSCW_text[14];
    HAL_StatusTypeDef r_stat = TCV_ESTTC_R(0x00, 13, currentSCW_text); errorcheck_goto(r_stat, end);
    unsigned int RSSI; unsigned int addr; unsigned int resetCt;  uint16_t SCW;
    sscanf(currentSCW_text, "OK+%2d%2x%2x%4hx", &RSSI, &addr, &resetCt, &SCW);
    /*// If error flags are set in SCW, propagate the error so the calling thread reports this
    if (SCW_GET_HFXT(SCW) || !SCW_GET_FRAM(SCW) || !SCW_GET_RFTS(SCW)) return HAL_ERROR; */

    uint16_t newSCW = (SCW & 0b0111011111110011); // Take the old SCW and force the reserved bits to 0
    for (uint8_t i = 0; i < 16; i++) // Update the bits as indicated by the mask
    {
        if ((mask >> i) & 1) // If this bit is to be updated...
        {
            if ((newval >> i) & 1) newSCW |= (1 << i); // ... and the new value is a 1, force a 1 using bitwise OR with 1
            else newSCW &= (UINT16_MAX - (1 << i)); // ... and the new value is a 0, force a 0 using bitwise AND with 0
        }
    }
    char newSCW_text[5];
    sprintf(newSCW_text, "%.4X", newSCW);
    char reply[8];
    r_stat = TCV_ESTTC_W(0x00, newSCW_text, 7, reply); errorcheck_goto(r_stat, end);
    if (strncmp(newSCW_text, reply + 3, 4) != 0)
    {
#ifdef DEBUG_ENABLED
        panic("%s", "Failed to update SCW");
#endif
        r_stat = HAL_ERROR;
    }
end:
    Periph_EndTransact(DEV_TCV);
    return r_stat;
}

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
HAL_StatusTypeDef TCV_ESTTC_R(uint8_t cmd, uint8_t answerMaxLen, char* answerBuf) {
    assert(answerBuf && answerMaxLen);
    Periph_BeginTransact(DEV_TCV, 16+answerMaxLen, HAL_MAX_DELAY);
    sprintf(S_txBuf, "ES+R%2x%02X", UHF_I2C_ADDRESS, (unsigned int)cmd);
    HAL_StatusTypeDef retstat = TCV_ESTTC(TCV_PREFIX_LEN, answerMaxLen); 
    if (retstat == HAL_OK) {strncpy(answerBuf, S_rxBuf, answerMaxLen); answerBuf[answerMaxLen] = '\0';}
    Periph_EndTransact(DEV_TCV);
    return retstat;
}

HAL_StatusTypeDef TCV_ESTTC_W(uint8_t cmd, const char* writeData, uint8_t answerMaxLen, char* answerBuf) {    
    assert(writeData);
    uint8_t wBodySz = TCV_PREFIX_LEN + strlen(writeData);
    Periph_BeginTransact(DEV_TCV, wBodySz, HAL_MAX_DELAY);
    sprintf(S_txBuf, "ES+W%2x%02X", UHF_I2C_ADDRESS, (unsigned int)cmd);
    assert(strlen(writeData) < TCV_TXBUF_SZ);
    strncpy(S_txBuf + TCV_PREFIX_LEN, writeData, TCV_TXBUF_SZ - 1); S_txBuf[TCV_TXBUF_SZ - 1] = '\0';

    HAL_StatusTypeDef retstat = TCV_ESTTC(wBodySz, answerMaxLen); 
    if (answerBuf != NULL && retstat == HAL_OK) { strncpy(answerBuf, S_rxBuf, answerMaxLen); answerBuf[answerMaxLen] = '\0'; }
    Periph_EndTransact(DEV_TCV);
    return retstat;
}

HAL_StatusTypeDef TCV_ReadErrorStat(TCV_Err_t* result)
{
    assert(result);
    char reply[14];
    HAL_StatusTypeDef retstat = TCV_ESTTC_R(0x00, 13, reply); errorcheck_ret(retstat);
    unsigned int RSSI; unsigned int addr; unsigned int resetCt; uint16_t SCW;
    // TODO: is RSSI hex or decimal? Isn't specified in manual. Assuming hex for now.
    if (sscanf(reply, "OK+%2x%2x%2x%4hx", &RSSI, &addr, &resetCt, &SCW) < 4) return HAL_ERROR;
    result->resetCount = resetCt;
    result->HFXT = SCW_GET_HFXT(SCW);
    result->FRAM = !SCW_GET_FRAM(SCW);
    result->RFTS = !SCW_GET_RFTS(SCW);
    return retstat;
}

HAL_StatusTypeDef TCV_Reset(void)
{
    return UpdateSCW(1 << 11, 1 << 11);
}

HAL_StatusTypeDef TCV_SetRFMode(TCV_RFMode_e mode)
{
    return UpdateSCW(0b0000011100000000, mode << 8);
}

HAL_StatusTypeDef TCV_SetBeacon(char state)
{
    return UpdateSCW(0b0000000001000000, (!!state) << 6);
}

HAL_StatusTypeDef TCV_SetPipe(char state)
{
    return UpdateSCW(0b0000000000100000, (!!state) << 5);
}

HAL_StatusTypeDef TCV_GetRxPackets(TCV_RxPacket_t* result)
{
    assert(result);
    // Specifically don't start a transaction here 
    // because we don't care about an interruption between 0x04 and 0x05
    char reply[14];
    HAL_StatusTypeDef retstat = TCV_ESTTC_R(0x04, 13, reply); errorcheck_ret(retstat);
    uint32_t rxCount; unsigned int RSSI;
    if (sscanf(reply, "OK+%2d%8lx", &RSSI, &rxCount) < 2) return HAL_ERROR;
    retstat = TCV_ESTTC_R(0x05, 13, reply); errorcheck_ret(retstat);
    uint32_t rxErrCt;
    if (sscanf(reply, "OK+%2d%8lx", &RSSI, &rxErrCt) < 2) return HAL_ERROR;
    result->numRxPackets = rxCount; result->numRxPacketsDropped = rxErrCt;
    return HAL_OK;
}

HAL_StatusTypeDef TCV_GetTemp(float* result)
{
    assert(result);
    char reply[9];
    HAL_StatusTypeDef retstat = TCV_ESTTC_R(0x0a, 8, reply); errorcheck_ret(retstat);
    // The transceiver manual is straight up wrong as to how the temperature is formatted.
    // When it says [B][TTTT], it actually means [B][TTTTT]
    // where the first T is the sign, the next two Ts are digits,
    // the fourth T is the decimal point, and the last T is the tenths digit.

    // Also our toolchain doesn't support float scanning,
    // so since TCV temperature has exactly one digit after the decimal point,
    // doing a simple conversion here by hand.
    int tempInt, tempTenth;
    if (sscanf(reply, "OK %3d.%1d", &tempInt, &tempTenth) < 2) return HAL_ERROR;
    if (tempInt < 0) tempTenth = -tempTenth;
    *result = tempInt + (float)tempTenth / 10.0;
    return HAL_OK;
}

HAL_StatusTypeDef TCV_GetAntRelease(TCV_AntRelease_e* result)
{
    assert(result);
    char reply[8];
    HAL_StatusTypeDef retstat = TCV_ESTTC_R(0xf2, 7, reply); errorcheck_ret(retstat);
    uint16_t deplStatus;
    if (sscanf(reply, "OK+%4hx", &deplStatus) < 1) return HAL_ERROR;
    //uint8_t timer = deplStatus & ((1 << 8) - 1);
    *result = deplStatus >> 8;
    return HAL_OK;
}

HAL_StatusTypeDef TCV_SetAntRelease(TCV_AntRelease_e mode, uint8_t minutes)
{
    assert(
        (mode == TCV_AntReleaseDisabled || mode == TCV_AntReleaseEnabled || mode ==TCV_AntReleaseRobust) 
        && minutes >= 0x0a
    );
    uint16_t writeval = (mode << 8) | minutes;
    char writeval_text[5]; sprintf(writeval_text, "%.4X", writeval);
    char reply[8];
    HAL_StatusTypeDef retstat = TCV_ESTTC_W(0xf2, writeval_text, 7, reply); errorcheck_ret(retstat);
    if (strncmp(writeval_text, reply + 3, 4) != 0) return HAL_ERROR;
    return HAL_OK;
}

HAL_StatusTypeDef TCV_SetBeaconContent(const char* message)
{
    assert(message);
    char cmd[2 + TCV_MAX_BEACON_LEN + 1];
    size_t msg_len = strlen(message);
    sprintf(cmd, "%.2X", TCV_MAX_BEACON_LEN <= msg_len ? TCV_MAX_BEACON_LEN : msg_len);
    strncpy(cmd + 2, message, TCV_MAX_BEACON_LEN); cmd[2 + TCV_MAX_BEACON_LEN] = '\0';
    char reply[4];
    HAL_StatusTypeDef retstat = TCV_ESTTC_W(0xfb, cmd, 3, reply); errorcheck_ret(retstat);
    if (strncmp(reply, "OK", 2) != 0) return HAL_ERROR;
    return HAL_OK;
}

HAL_StatusTypeDef TCV_SetBeaconPeriod(uint16_t seconds)
{
    assert(seconds);
    char Wdata[8 + 1];
    sprintf(Wdata, "0000%.4hx", seconds);
    char answer[2+1];
    HAL_StatusTypeDef retstat = TCV_ESTTC_W(0x07, Wdata, 2, answer); errorcheck_ret(retstat);
    if (strncmp(answer, "OK", 2) != 0) return HAL_ERROR;
    return HAL_OK;
}

HAL_StatusTypeDef TCV_Kill(void)
{
    Periph_BeginTransact(DEV_TCV, 40, HAL_MAX_DELAY);
    HAL_StatusTypeDef retstat = TCV_SetBeacon(0); errorcheck_goto(retstat, end);
    retstat = TCV_SetPipe(0); errorcheck_goto(retstat, end);
    retstat = TCV_ESTTC_W(0xfc, "23", 0, NULL);
end:
    Periph_EndTransact(DEV_TCV);
    return retstat;
}


/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* PARKED CODE
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

// TCV tests begin
//#define TCV_TEST_LOGIC // Define this to enable logical testing suite
//#define TCV_TEST_PHYSICAL // Define this to enable testing with the real TCV
    // Logical tests - use without actual TCV
    // and with NO_PERIPHS defined.
    // Designed to test the logical operation of the TCV drivers.
//#ifdef TCV_TEST_LOGIC
//char answer[16];
//volatile HAL_StatusTypeDef retstat;
//char exReadSCW[] = "ES+R2200\r";
//char* expectedTx;
//#define I2C_SetEx I2C_ExpectTx(SYS_I2C, (uint8_t*)expectedTx, strlen(expectedTx))
//#define ass_ret_ok assert(retstat == HAL_OK)
//// Low-numbered command
//retstat = TCV_ESTTC_R(0x00, 8, answer);
//assert(retstat == HAL_OK);
//assert(strcmp(answer, "OK") == 0);
//// High-numbered command
//retstat = TCV_ESTTC_R(0xFE, 8, answer);
//assert(retstat == HAL_OK);
//assert(strcmp(answer, "OK") == 0);
//// Small answer size
//retstat = TCV_ESTTC_R(0x42, 1, answer);
//assert(retstat == HAL_OK);
//assert(strcmp(answer, "O") == 0);
//// Not copied on error response
//strcpy(answer, "Don't touch");
//retstat = TCV_ESTTC_R(0x42, 1, answer); // give error response
//assert(retstat == HAL_ERROR);
//assert(strcmp(answer, "Don't touch") == 0);
//// Answer longer than longest error response
//char longanswer[33];
//retstat = TCV_ESTTC_R(0x42, 32, longanswer);
//assert(retstat == HAL_OK);
//assert(strcmp(longanswer, "HelloHelloHelloHelloHelloHello42") == 0);
//// Raises logic error on null pass
//retstat = TCV_ESTTC_R(0, 0, 0);
//// Error on no CR at end of reply
//retstat = TCV_ESTTC_R(0, 1, answer); // give no CR
//assert(retstat == HAL_ERROR);
//// Error on answer not matching OK or E_
//retstat = TCV_ESTTC_R(0, 1, answer); // give weird answer with matching CRC
//assert(retstat == HAL_ERROR);
////Normal write behavior
//char writeData[] = "Hello";
//char expectedTx[] = "ES+W2213Hello 6060604A\r";
//I2C_ExpectTx(SYS_I2C, expectedTx, strlen(expectedTx));
//retstat = TCV_ESTTC_W(0x13, writeData, 4, answer);
//assert(retstat == HAL_OK);
//assert(strcmp(writeData, "Hello") == 0);
//// Large command no, short expected answer
//char writeData[] = "Hello";
//char expect0[] = "ES+W22FEHello 94D46033\r";
//I2C_ExpectTx(SYS_I2C, expect0, strlen(expect0));
//retstat = TCV_ESTTC_W(0xFE, writeData, 1, answer);
//assert(retstat == HAL_OK);
//assert(strlen(answer) == 1);
//// Long writeData
//char longWriteData[65] = "HelloHelloHelloHelloHelloHello42HelloHelloHelloHelloHelloHello42";
//char expectedLongTx[] = "ES+W2242HelloHelloHelloHelloHelloHello42HelloHelloHelloHelloHelloHello42 564597F3\r";
//I2C_ExpectTx(SYS_I2C, expectedLongTx, strlen(expectedLongTx));
//retstat = TCV_ESTTC_W(0x42, longWriteData, 4, answer);
//assert(retstat == HAL_OK);
//// No answer expected
//char expectedTx1[] = "ES+W2200H 777D32E7\r";
//I2C_ExpectTx(SYS_I2C, expectedTx1, strlen(expectedTx1));
//retstat = TCV_ESTTC_W(0, "H", 0, NULL);
//assert(retstat == HAL_OK);
//// Logic error on empty writeData
//TCV_ESTTC_W(0, 0, 0, 0); // noreturn
//// Reading error statuses
//TCV_Err_t err;
//char exErr[] = "ES+R2200 BD888E1F\r"; expectedTx = exErr;
//I2C_SetEx;
//retstat = TCV_ReadErrorStat(&err); // give OK+0022423382 90E2F440\r
//assert(retstat == HAL_OK);
//assert(err.HFXT == 0);
//assert(err.FRAM == 0);
//assert(err.RFTS == 1);
//assert(err.resetCount == 42);
//// Reset command
//char exReset[] = "ES+W22003883 AE2108E8\r"; expectedTx = exReset;
//I2C_SetEx;
//retstat = TCV_Reset();
//ass_ret_ok;
//// Set RF mode
//expectedTx = exReadSCW;
//I2C_SetEx;
//retstat = TCV_SetRFMode(TCV_4800bps); // need manual check of W SCW
//ass_ret_ok;
//// Set beacon on
//expectedTx = exReadSCW;
//I2C_SetEx;
//retstat = TCV_SetBeacon(2); // need manual check of W SCW
//ass_ret_ok;
//// Set beacon off
//expectedTx = exReadSCW;
//I2C_SetEx;
//retstat = TCV_SetBeacon(0); // need manual check of W SCW
//ass_ret_ok;
//// Set pipe on
//expectedTx = exReadSCW;
//I2C_SetEx;
//retstat = TCV_SetPipe(2); // need manual check of W SCW
//ass_ret_ok;
//// Set pipe on even when it is already on
//expectedTx = exReadSCW;
//I2C_SetEx;
//retstat = TCV_SetPipe(1); // give pipe on for initial SCW read
//ass_ret_ok;
//// Read Rx packets
//char exRPckt[] = "ES+R2204\r"; expectedTx = exRPckt;
//I2C_SetEx;
//TCV_RxPacket_t packets;
//retstat = TCV_GetRxPackets(&packets); // give OK+00FFFFFFFE 9B613F25\r, 
//// cmd 0x05 needs manual checking, give OK+4200000002 5B3EE021\r
//ass_ret_ok;
//assert(packets.numRxPackets == 0xFFFFFFFE);
//assert(packets.numRxPacketsDropped == 2);
//// Get temp
//char exTemp[] = "ES+R220A\r"; expectedTx = exTemp;
//I2C_SetEx;
//float temp;
//retstat = TCV_GetTemp(&temp); // give OK -4.4 D0F8C873\r
//ass_ret_ok;
//// manually compare floating-point val
//// Get ant release
//char exAntRel[] = "ES+R22F2\r"; expectedTx = exAntRel;
//I2C_SetEx;
//TCV_AntRelease_e rel_t;
//retstat = TCV_GetAntRelease(&rel_t); // give OK+1142 E2628C40\r
//ass_ret_ok;
//assert(rel_t == TCV_AntReleaseRobust);
//// Set ant release
//char exAntRelW[] = "ES+W22F20142\r"; expectedTx = exAntRelW;
//I2C_SetEx;
//retstat = TCV_SetAntRelease(TCV_AntReleaseEnabled, 0x42);
//ass_ret_ok;
//// Set beacon content, doesn't read more than allowed
//char testBeacon[95] = "HelloHelloHelloHelloHelloHelloHelloHelloHelloHelloHelloHelloHelloHelloHelloHelloHello42Haha no";
//char exBeacon[] = "ES+W22FB57HelloHelloHelloHelloHelloHelloHelloHelloHelloHelloHelloHelloHelloHelloHelloHelloHello42\r"; expectedTx = exBeacon;
//I2C_SetEx;
//retstat = TCV_SetBeaconContent(testBeacon);
//ass_ret_ok;
//// Set beacon period
//char exBeaconT[] = "ES+W220700000042\r"; expectedTx = exBeaconT;
//I2C_SetEx;
//retstat = TCV_SetBeaconPeriod(0x42);
//ass_ret_ok;
//// Kill
//retstat = TCV_Kill(); // manually check 
//ass_ret_ok;
//#endif
//// Tests with the real TCV
//#ifdef TCV_TEST_PHYSICAL
//char answer[16];
//volatile HAL_StatusTypeDef retstat;
//
//// Read SCW
//retstat = TCV_ESTTC_R(0x00, 16, answer);
//TCV_AntRelease_e ret;
//retstat = TCV_GetAntRelease(&ret);
////	// Read temperature
////	float temp;
////	//retstat = TCV_GetTemp(&temp);
////	// Set beacon content
////	retstat = TCV_SetBeaconContent("Test beacon message");
////	// Turn on beacon
////	retstat = TCV_SetBeacon(1);
////	// Set RF mode
////	retstat = TCV_SetRFMode(TCV_19200bps_Xwide);
////	// Turn off beacon
////	retstat = TCV_SetBeacon(0);
////	// Read antenna config
////	TCV_AntRelease_e antCfg;
////	retstat = TCV_GetAntRelease(&antCfg);
////	// Kill command
////	retstat = TCV_Kill();
//#endif


// Helper to calculate CRC32 from a string of arbitrary length inputSz.
// If inputSz is not a multiple of 4, will append zeroes during the calculation to make it a multiple of 4???
// Not explicitly mentioned in either the TCV or the OBC manuals but I don't know how else one would do this
/*uint32_t RoundAndCalcCRC(const char* input, uint8_t inputSz)
{
    uint32_t crc; uint8_t Mod4 = inputSz % 4;
    if (Mod4 == 0) crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)input, inputSz / 4);
    else
    {
        char* rounded = malloc(sizeof(char) * (inputSz + (4 - Mod4)));
        if (rounded == NULL) Error_Handler(); // TODO: some kind of better response to malloc failure
        memcpy(rounded, input, inputSz);
        for (int i = 0; i < 4 - Mod4; i++) rounded[inputSz + i] = 0;
        crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)rounded, inputSz + (4 - Mod4));
        free(rounded);
    }
    return crc;
}*/
