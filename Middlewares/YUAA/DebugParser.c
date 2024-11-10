/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file DebugParser.c
* @brief Debug Command Parser C File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Elijah B.
* @version           1.0.0
* @date              2022.05.07
*
* @details           Command parser for YUAA debug commands
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/


/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "MCU_Init.h"
#include "DebugParser.h"
#include "Svc_RTC.h"
#include "stm32f4xx_hal.h"

#ifdef DBPARSER_SD
#include "SD_Routines.h"
#include "fatfs.h"
// Buffer lengths for SD debug commands
#define DBPARSER_SD_MAX_DIR_FILES (8)
#define DBPARSER_SD_MAX_PATH_LEN (64)
#define DBPARSER_SD_MAX_FILNAME_LEN (8)
#define DBPARSER_SD_MAX_FILCONTENTS (128)
#endif

#ifdef DBPARSER_GGB
#include "GGB.h"
#endif

#ifdef DBPARSER_EPS
#include "EPS.h"
#endif

#ifdef DBPARSER_SS
#include "SS.h"
#endif

#ifdef DBPARSER_CRD
#include "CTR.h"
#include "MUX.h"
#include "DAC.h"
#include "RTD.h"
#endif

#include <string.h>
#include <stdint.h>
#include <stdio.h>

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/


#define BREAK_ON_OOB(val, minval, maxval, format,...) if (val < minval || val > maxval) \
 { fprintf(output, "ERROR: " format "\n", __VA_ARGS__); break; }

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL (STATIC) ROUTINES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#ifdef DBPARSER_SS
static void SS_parse(uint32_t cmd, const char* cmdBody, FILE* output)
{
    switch (cmd)
    {
    case 0x10: // Exec SS cmd
    {
        unsigned int SScmd = 0xFF;
        sscanf(cmdBody, "%2x", &SScmd);
        BREAK_ON_OOB(SScmd, 0, 0xA2, "Invalid SS command %X!", SScmd)

        unsigned int ansBytes = 0;
        sscanf(cmdBody + 2, "%u", &ansBytes);
        BREAK_ON_OOB(ansBytes, 0, 255, "Invalid SS answer length %d!", ansBytes)

        int32_t param = INT32_MAX;
        uint8_t paramLen = 0;
        uint8_t paramArr[2]; //All SS commands with parameters have 16-bit parameters
        uint8_t* paramPtr;
        char* paramBegin = strchr(cmdBody + 2, '+') + 1;
        if ((uintptr_t)paramBegin > 1)
        {
            sscanf(paramBegin, "%ld", &param);
        }

        if (param < INT32_MAX) //Valid param detected 
        {
            paramPtr = paramArr;
            //Store the 16-bit param least significant byte first in paramPtr
            for (uint8_t i = 0; i < 2; i++) paramArr[i] = (param >> i) % 256;
            paramLen = 2;
            fprintf(output, "Detected parameter %ld\n", param);
        }
        else paramPtr = NULL; //No valid param detected

        uint8_t answerArr[32]; for (uint8_t i = 0; i < 32; i++) answerArr[i] = 0;

        fprintf(output, "Transmitting command %X\n", SScmd);
        CmdRet_t retstat = ExecCmd(SScmd, paramPtr, answerArr, paramLen, ansBytes, NULL);

        fprintf(output, "HAL return status: %d, SS return status: %X\n", retstat.HAL_ret, retstat.SS_ret);
        if (retstat.HAL_ret == HAL_OK)
        {
            fprintf(output, "Answer: ");
            for (uint8_t i = 0; i < 32; i++) fprintf(output, "%.2X ", answerArr[i]);
            fprintf(output, "\n");
        }
        break;
    }
    case 0x11: // Init SPI intf
    {
        fprintf(output, "Initialized hspi1 with result %d\n", HAL_SPI_Init(&hspi1));
        break;
    }
    case 0x12: // De-init SPI intf
    {
        fprintf(output, "De-initialized hspi1 with result %d\n", HAL_SPI_DeInit(&hspi1));
        break;
    }
    case 0x13: // Print SPI handle
    {
        fprintf(output, "TxXferCount: %d\n", hspi1.TxXferCount);
        fprintf(output, "RxXferCount: %d\n", hspi1.RxXferCount);
        fprintf(output, "State: %d\n", hspi1.State);
        fprintf(output, "ErrorCode: %lu\n", hspi1.ErrorCode);
        break;
    }
    case 0x15: // Get vector from ADC
    {
        Vec3D_t result;
        fprintf(output, "Getting SS vector from ADCs.\n");
        HAL_StatusTypeDef retstat = SS_GetVector_FromADC(&result);
        fprintf(output, "Return status: %d\n", retstat);
        if (retstat == HAL_OK)
        {
            fprintf(output, "X: %f\n", result.X);
            fprintf(output, "Y: %f\n", result.Y);
            fprintf(output, "Z: %f\n", result.Z);
        }
        break;
    }
    default:
        fprintf(output, "ERROR: Unknown SS command %lX!\n", cmd);
        break;
    }
}
#endif
#ifdef DBPARSER_SD
static void SD_parse(uint32_t cmd, const char* cmdBody, FILE* output)
{
    switch (cmd)
    {
    case 0x20: //Print root
    case 0x21: //Print dir
    {
        FILINFO dir[DBPARSER_SD_MAX_DIR_FILES];
        char* dirName;
        if (cmd == 0x20)  dirName = "/";
        else dirName = (char*)cmdBody;
        fprintf(output, "Listing contents of %s:\n", dirName);
        FRESULT fres = SD_listDir(cmdBody, dir, DBPARSER_SD_MAX_DIR_FILES);
        BREAK_ON_OOB(fres, FR_OK, FR_OK, "ERROR: Operation returned error %d", fres)

        for (int i = 0; i < DBPARSER_SD_MAX_DIR_FILES; i++)
        {
            if (strcmp(dir[i].fname, "end") == 0) break;
            fprintf(output, "%s\n", dir[i].fname);
        }

        break;
    }
    case 0x22: // Print file
    case 0x23: // Create file
    case 0x24: // Delete file
    {
        const char* filename = cmdBody;

        switch (cmd)
        {
        case 0x22: // Print file
        {
            char SD_fileBuf[DBPARSER_SD_MAX_FILCONTENTS + 1]; SD_fileBuf[DBPARSER_SD_MAX_FILCONTENTS] = '\0';
            FRESULT fres = SD_getFileContents(filename, SD_fileBuf, DBPARSER_SD_MAX_FILCONTENTS);
            if (fres != FR_OK)
            {
                fprintf(output, "ERROR: Operation returned error %d\n", fres);
                break;
            }
            fprintf(output, "Printing contents of %s\n%s", filename, SD_fileBuf);
            fprintf(output, "\n");

            break;
        }
        case 0x23: //Create file
        {
            fprintf(output, "Creating %s\n", filename);

            FRESULT fres = SD_createFile(filename);
            if (fres != FR_OK) fprintf(output, "ERROR: Operation returned error %d\n", fres);
            else fprintf(output, "Created file.\n");

            break;
        }
        case 0x24: // Delete file
        {
            fprintf(output, "Deleting %s\n", filename);

            FRESULT fres = SD_delFile(filename);
            if (fres != FR_OK) fprintf(output, "ERROR: Operation returned error %d\n", fres);
            else fprintf(output, "Deleted file.\n");
            
            break;
        }
        }
        break;
    }
    case 0x25: //Append data to file
    case 0x26: //Overwrite file
    {
        char filename[DBPARSER_SD_MAX_FILNAME_LEN];
        const char* toWrite = strchr(cmdBody, '+') + 1;
        const char* filenameStart = cmdBody;

        BREAK_ON_OOB(toWrite - filenameStart - 1, 1, DBPARSER_SD_MAX_FILNAME_LEN, "Filename too long %td", toWrite - filenameStart - 1)
        strncpy(filename, filenameStart, toWrite - filenameStart - 1); filename[toWrite - filenameStart - 1] = '\0';

        fprintf(output, "Writing %s to %s\n", toWrite, filename);

        FRESULT fres;
        switch (cmd)
        {
        case 0x25: //Append data to file
            fres = SD_appendData(filename, toWrite, strlen(toWrite));
            break;
        case 0x26: //Overwrite file
            fres = SD_overwriteData(filename, toWrite, strlen(toWrite));
            break;
        }
        if (fres != FR_OK) fprintf(output, "ERROR: Writing returned error %d\n", fres);
        else fprintf(output, "Written to file.\n");
        break;
    }
    case 0x27: //Print free space
    {
        uint32_t freeKiB;
        FRESULT fres = SD_getFreeSpace(&freeKiB);
        if (fres != FR_OK) fprintf(output, "ERROR: Operation returned error %d\n", fres);
        else fprintf(output, "Free space on SD card: %lu KiB\n", freeKiB);
        break;
    }
    default:
        fprintf(output, "ERROR: Unknown SD command %lX!\n", cmd);
        break;
    }
}
#endif
#ifdef DBPARSER_GGB
static void GGB_parse(uint32_t cmd, const char* cmdBody, FILE* output)
{
    switch (cmd)
    {
    case 0x30: //GGB read reg
    case 0x31: //GGB write reg
    {
        unsigned int offset = -1;
        sscanf(cmdBody, "%x", &offset);
        BREAK_ON_OOB(offset, 0, 0x7E, "Invalid register address %X!", offset)

        switch (cmd)
        {
        case 0x30: //GGB read reg
        {
            uint8_t value = 0;
            uint8_t retstat = GGB_read_register(offset, &value);
            fprintf(output, "Read GGB register %X with result %d\n", offset, retstat);
            fprintf(output, "Register contents: 0x%X\n", value);
            break;
        }
        case 0x31: //GGB write reg
        {
            unsigned int value = -1;
            sscanf(cmdBody, "%2x", &value);
            BREAK_ON_OOB(value, 0, UINT8_MAX, "Invalid value 0x%X!", value)

            uint8_t retstat = GGB_write_register(offset, (uint8_t)value);
            fprintf(output, "Wrote %X to GGB register %X with result %d\n", value, offset, retstat);
            break;
        }
        }
        
        break;
    }
    case 0x32: //Motor A polarity
    {
        unsigned int polarity = -1;
        sscanf(cmdBody, "%1u", &polarity);
        BREAK_ON_OOB(polarity, 0, 1, "Invalid polarity %d!", polarity)

        uint8_t retstat = GGB_WRITE_MOTOR_A_INVERT((uint8_t)polarity);
        fprintf(output, "Wrote polarity %d to GGB with result %d\n", polarity, retstat);
        break;
    }
    case 0x33: //Set dir+lvl
    {
        unsigned int dir = -1;
        sscanf(cmdBody, "%1u", &dir);
        BREAK_ON_OOB(dir, 0, 1, "Invalid direction %d!", dir)

        unsigned int lvl = 0;
        sscanf(cmdBody + 2, "%3u", &lvl);
        BREAK_ON_OOB(lvl, 0, 255, "Invalid drive level %d!", lvl)

        uint8_t retstat = GGB_WRITE_MA_DRIVE((uint8_t)dir, (uint8_t)lvl);
        fprintf(output, "Wrote drive level %d and direction %d to GGB with result %d\n", lvl, dir, retstat);
        break;
    }
    case 0x34: //Enable driver
    {
        fprintf(output, "Enabled GGB drivers with result %hhu\n", GGB_ENABLE_DRIVERS());
        break;
    }
    case 0x35: //Disable driver
    {
        fprintf(output, "Disabled GGB drivers with result %hhu\n", GGB_DISABLE_DRIVERS());
        break;
    }
    default:
        fprintf(output, "ERROR: Unknown GDB command %lX!\n", cmd);
        break;
    }
}
#endif
#ifdef DBPARSER_EPS
static void EPS_parse(uint32_t cmd, const char* cmdBody, FILE* output)
{
    switch (cmd)
    {
    case 0x50: // EPS read
    {
        unsigned int readAddr = -1;
        sscanf(cmdBody, "%2u", &readAddr);
        BREAK_ON_OOB(readAddr, 0, 52, "Invalid EPS read address %u!", readAddr)

        uint16_t answer;
        HAL_StatusTypeDef retstat = EPS_R_Cmd(readAddr, &answer);
        if (retstat == HAL_OK) fprintf(output, "EPS read command executed and returned %hu\n", answer);
        else fprintf(output, "EPS read command error %u", retstat);
        break;
    }
    case 0x51: // EPS write
    {
        unsigned int writeAddr = -1;
        sscanf(cmdBody, "%2x", &writeAddr);
        BREAK_ON_OOB(writeAddr, 0, 20, "Invalid EPS write address %u!", writeAddr)

        unsigned int Wval = -1;
        sscanf(strchr(cmdBody, '+') + 1, "%1u", &Wval);
        BREAK_ON_OOB(Wval, 0, 3, "Invalid EPS write value %u!", Wval)

        HAL_StatusTypeDef retstat = EPS_W_Cmd(writeAddr, (EPS_W_e)Wval);
        if (retstat == HAL_OK) fprintf(output, "EPS write command executed successfully\n");
        else fprintf(output, "EPS write command error %u", retstat);
        break;
    }
    default:
        fprintf(output, "ERROR: Unknown EPS command %lX!\n", cmd);
        break;
    }
}
#endif
#ifdef DBPARSER_I2C
static void I2C_parse(uint32_t cmd, const char* cmdBody, FILE* output)
{
    uint16_t intNum = 0;
    sscanf(cmdBody, "%1hu", &intNum);
    if (intNum < 1 || intNum > 3) 
    { fprintf(output, "ERROR: Invalid interface number %hd!\n", intNum); return; }

    I2C_HandleTypeDef *hi2c;
    switch (intNum)
    {
    case 1:
        hi2c = &hi2c1;
        break;
    case 2:
        hi2c = &hi2c2;
        break;
    case 3:
        hi2c = &hi2c3;
        break;
    }

    switch (cmd)
    {
    case 0x60: //Init interface
    {
        switch (intNum)
        {
        case 1:
            MX_I2C1_Init();
            break;
        case 2:
            MX_I2C2_Init();
            break;
        case 3:
            MX_I2C3_Init();
            break;
        }
        fprintf(output, "I2C%d initialized.\n", intNum);
        break;
    }
    case 0x61: //De-init interface
    {
        HAL_StatusTypeDef retstat = HAL_I2C_DeInit(hi2c);
        fprintf(output, "I2C%d de-initialized with result %d.\n", intNum, retstat);
        break;
    }

    case 0x62: //Reset interface
    {
        I2C_Reset(hi2c);
        fprintf(output, "I2C%d reset.\n", intNum);
        break;
    }

    case 0x63: //Print handle
    {
        uint16_t XferCount = hi2c->XferCount;
        HAL_I2C_StateTypeDef State = hi2c->State;
        HAL_I2C_ModeTypeDef Mode = hi2c->Mode;
        uint32_t ErrorCode = hi2c->ErrorCode;
        uint32_t EventCount = hi2c->EventCount;
        fprintf(output, "XferCount: %hu\n State: %X\n Mode: %X\n ErrorCode: %lu\n EventCount: %lu\n", XferCount, State, Mode, ErrorCode, EventCount);
        break;
    }

    case 0x6A: //I2C W
    case 0x6B: //I2C R
    {
        uint16_t addr = -1;
        sscanf(cmdBody + 1, "%2hx", &addr);
        BREAK_ON_OOB(addr, 0, 127, "Invalid I2C address %hX", addr)

        uint8_t data[64]; memset(data, 0, 64*sizeof(uint8_t));
        switch (cmd)
        {
        case 0x6A: //I2C W
        {
        	uint16_t data_pt = 0;
            for (uint16_t str_pt = 0; str_pt < strlen(cmdBody + 3) && str_pt < 64; str_pt += 2, data_pt++)
            {
            	unsigned int tmp;
                sscanf(cmdBody + 3 + str_pt, "%2x", &tmp);
                data[data_pt] = (uint8_t)tmp;
            }
            //fprintf(output, "Bytewise contents of data: "); for (uint8_t i=0;i<8;i++) fprintf(output, "%X ", data[i]); fprintf(output, "\n");
            fprintf(output, "Size of data: %d\n", data_pt);
            HAL_StatusTypeDef retstat = HAL_I2C_Master_Transmit(hi2c, addr << 1, data, data_pt, HAL_MAX_DELAY);
            fprintf(output, "Transmitted Write command ");
            for (uint8_t i = 0; i < data_pt; i++) {unsigned int tmp = data[i]; fprintf(output, "%02X ", tmp);}
            fprintf(output, "to 0x%02x with result %d.\n", addr, retstat);
            break;
        }
        case 0x6B: //I2C R
        {
            unsigned int size = -1;
            sscanf(cmdBody + 3, "%u", &size);
            BREAK_ON_OOB(size, 1, 63, "Invalid size %d!", size)

            HAL_StatusTypeDef retstat = HAL_I2C_Master_Receive(hi2c, addr << 1, data, (uint16_t)size, HAL_MAX_DELAY);
            fprintf(output, "Transmitted Read command requesting %d bytes to 0x%02X with result %d.\n", size, addr, retstat);
            //fprintf(output, "Bytewise contents of data: "); for (uint8_t i=0;i<8;i++) fprintf(output, "%X ", data[i]); fprintf(output, "\n");

            if (retstat == HAL_OK)
            {
                fprintf(output, "Reply: ");
                for (uint8_t i = 0; i < 8; i++) {unsigned int tmp = data[i]; fprintf(output, "%02X ", tmp);}
                fprintf(output, "\n");
            }
            break;
        }
        }
        
        break;
    }
    default:
        fprintf(output, "ERROR: Unknown I2C command %lX!\n", cmd);
        break;
    }
}
#endif
#ifdef DBPARSER_CRD
static void CRD_parse(uint32_t cmd, const char* cmdBody, FILE* output)
{
    switch (cmd)
    {
    case 0x70:
    {
        int addr;
        if (sscanf(cmdBody, "%d", &addr) != 1) { fprintf(output, "ERROR: Invalid counter address %s!\n", cmdBody); return; }
        uint32_t data; uint8_t overflow = 0;
        unsigned int actualAddr;
        if (addr) actualAddr = CTR_ADDR_ODD; else actualAddr = CTR_ADDR_EVEN;
        HAL_StatusTypeDef retstat = CTR_ReadReset(actualAddr, &data, &overflow);
        fprintf(output, "Read counter @ 0x%.2X with result %d", actualAddr, retstat);
        if (retstat == HAL_OK)
        {
            fprintf(output, ", count %lu", data);
            if (overflow) fprintf(output, ", overflow!");
        }
        fprintf(output, "\n");
        break;
    }
    case 0x75:
    {
        unsigned int inputNo;
        if (sscanf(cmdBody, "%u", &inputNo) != 1 || inputNo > 7)
        { fprintf(output, "ERROR: Invalid counter number %s!\n", cmdBody); return; }
        fprintf(output, "Selected counter %u with result %d\n", inputNo, MUX_SelectCtrChip(inputNo));
        break;
    }
    case 0x76:
    {
        unsigned int addr;
        if (sscanf(cmdBody, "%1x", &addr) != 1) { fprintf(output, "ERROR: Invalid address %.2s!\n", cmdBody); return; }
        BREAK_ON_OOB(addr, 1, 2, "Invalid DAC selection %u!", addr);
        uint8_t actualAddr;
        if (addr == 1) actualAddr = DAC_1; else actualAddr = DAC_2;
        DAC_SetupType out;
        unsigned int tmp[4];
        for (int i=0; i<4; i++)
            if (sscanf(cmdBody + 2 + 2*i, "%2x", tmp + i) != 1)
            { fprintf(output, "ERROR: Invalid output %s!\n", cmdBody + 2); return;}
        fprintf(output, "Setting outputs ");
        for (int i = 0; i < 4; i++)
        {
            fprintf(output, "%.2X, ", tmp[i]);
            out.outputVals[i] = tmp[i];
        }
        fprintf(output, "to DAC_%hu with result %d.\n", addr, DAC_SetOutput(actualAddr, out));
        break;
    }
    case 0x77:
    {
        unsigned int addr;
        if (sscanf(cmdBody, "%1x", &addr) != 1) { fprintf(output, "ERROR: Invalid address %.2s!\n", cmdBody); return; }
        BREAK_ON_OOB(addr, 1, 2, "Invalid DAC selection %u!", addr);
        uint8_t actualAddr;
        if (addr == 1) actualAddr = DAC_1; else actualAddr = DAC_2;
        fprintf(output, "Reset outputs of DAC_%u with result %d.\n", addr, DAC_ResetOutput(actualAddr));
        break;
    }
    case 0x7A:
    {
        unsigned int addr;
        if (sscanf(cmdBody, "%1x", &addr) != 1) { fprintf(output, "ERROR: Invalid address %.2s!\n", cmdBody); return; }
        BREAK_ON_OOB(addr, 1, 2, "Invalid RTD selection %u!", addr);
        uint8_t actualAddr;
        if (addr == 1) actualAddr = RTD_1; else actualAddr = RTD_2;
        float temp;
        HAL_StatusTypeDef retstat = RTD_GetTemp(actualAddr, &temp);
        fprintf(output, "Read temperature from RTD_%u with result %d", addr, retstat);
        if (retstat == HAL_OK) fprintf(output, ", %f deg", temp);
        fprintf(output, "\n");
        break;
    }
    default:
        break;
    }
}
#endif

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL (NON-STATIC) ROUTINES DEFINITION
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
void debugCmdParse(char* input, FILE* output)
{
	char * begin = strstr(input, DB_CMD_PREFIX);

    if (begin != NULL) //if begin is null, no YUAA command identified in the input
    {
        fprintf(output, "INFO: Received debug command %s\n", begin);
        uint32_t cmd;
        if (sscanf(begin + strlen(DB_CMD_PREFIX), "%2lx", &cmd) != 1) //read two hex digits after the debug cmd prefix
            fprintf(output, "ERROR: Invalid debug command syntax!\n"); 
        else 
        {
            char* cmdBody = begin + strlen(DB_CMD_PREFIX) + 2; 
            void (*cmdHandler)(uint32_t cmd, const char* cmdBody, FILE * output) = NULL;
            switch (cmd >> 4)
            {
#ifdef DBPARSER_SS
            case 0x1:
                cmdHandler = SS_parse;
                break;
#endif
#ifdef DBPARSER_SD
            case 0x2:
                cmdHandler = SD_parse;
                break;
#endif
#ifdef DBPARSER_GGB
            case 0x3:
                cmdHandler = GGB_parse;
                break;
#endif
            case 0x4:
                fprintf(output, "Command %lX not yet implemented!\n", cmd);
                break;
#ifdef DBPARSER_EPS
            case 0x5:
                cmdHandler = EPS_parse;
                break;
#endif
#ifdef DBPARSER_I2C
            case 0x6:
                cmdHandler = I2C_parse;
                break;    
#endif
#ifdef DBPARSER_CRD
            case 0x7:
                cmdHandler = CRD_parse;
                break;
#endif

            case 0xf: //Native
            {
                switch (cmd)
                {
                case 0xff:
                    fprintf(output, "%s\n", begin + strlen(DB_CMD_PREFIX) + 2);
                    break;
                case 0xf0: //Add current time, uptime
                {
                    float temp;
                    status_t retstat = GetCpuTemperature(&temp);
                    fprintf(output, "Read CPU temperature with status %d", retstat);
                    if (retstat == SEN_SUCCESS) fprintf(output, ": %f", temp);
                    fprintf(output, "\n");
                    break;
                }
                default:
                    fprintf(output, "ERROR: Invalid debug command %lX !\n", cmd);
                    break;
                }
                break;
            }
            default:
            {
                fprintf(output, "ERROR: Invalid debug command %lX !\n", cmd);
                break;
            }
            }
            if (cmdHandler != NULL) cmdHandler(cmd, cmdBody, output);
        }
    }
}
