/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file DebugParser.h
* @brief Debug Command Parser Header File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Elijah B.
* @version           1.1.0
* @date              2022.05.07
*
* @details           Declares the command parser for YUAA debug commands
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

#ifndef DBGPRS_H
#define DBGPRS_H

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#define DB_CMD_PREFIX ("DB+")

/* Define these symbols as necessary to include debug command support for the respective components
NOTE: these are independent from the PeriphHelper DB_ symbols! */

//#define DBPARSER_SS
//#define DBPARSER_SD
//#define DBPARSER_GGB
//#define DBPARSER_EPS
//#define DBPARSER_I2C
//#define DBPARSER_CRD

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DECLARATIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/*
* Debug commands shall have the format DB+xxyyyy...
* where xx are two hex digits specifying the command, and
* yyyy... is the command-specific argument list whose length depends on the command.
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Reference of implemented commands:
* SS 0x1.
* 0x10, DB+10<Cmd><AnsBytes>+<RequestParam>: Transmit the command specified by the two hex digits Cmd to the sun sensor and receive
*                  							 AnsBytes bytes. At most one parameter can be specified in RequestParam (as a signed decimal number).
*                  							 Print reply from SS.
* 0x11, DB+11: Call HAL_SPI_Init() on the SS SPI interface 
* 0x12, DB+12: Call HAL_SPI_DeInit() on the SS SPI interface
* 0x13, DB+13: Print some fields of the SPI_HandleTypeDef for the SS SPI interface
* 0x15, DB+15: Call SS_GetVector_FromADC() and print the resulting vector
*
* SD 0x2.
* 0x20, DB+20: Print the contents of the root directory
* 0x21, DB+21<Path>: Print the contents of directory specified by Path
* 0x22, DB+22<Filename>: Print the contents of the file Filename
* 0x23, DB+23<Filename>: Create a file called Filename
* 0x24, DB+24<Filename>: Delete the file called Filename
* 0x25, DB+25<Filename>+<Data>: Append Data to the already existing file called Filename
* 0x26, DB+26<Filename>+<Data>: Overwrite the already existing file called Filename with Data
* 0x27, DB+27: Print free space remaining on SD card
*
* GGB 0x3.
* 0x30, DB+30<Addr>: Read the value stored in the register specified by Addr (given as hex integer) and print it
* 0x31, DB+31<Addr>+<Val>: Write Val (given as hex integer) to the register specified by Addr (given as hex integer)
* 0x32, DB+32<Pol>: Set motor A polarity to Pol (0 for default, 1 for inverted)
* 0x33, DB+33<Dir>+<Lvl>: Set motor A drive to direction Dir (0 - forward, 1 - reverse) at level Lvl (0-255)
* 0x34, DB+34: Enable GGB motor control
* 0x35, DB+35: Disable GGB motor control
*
* MTQ 0x4.
* 0x40, DB+40<Axis><Duty>: Send a square wave output to the MTQ specified by Axis at duty cycle Duty for 5sec
*
* EPS 0x5.
* 0x50, DB+50<Addr>: Read the value stored in the register specified by Addr (as DECIMAL digits) and print it
* 0x51, DB+51<Addr>+<Val>: Write Val (in range 0..3) to the register specified by Addr (as HEX digits)
* 
* I2C 0x6.
* 0x60, DB+60<x>: Call MX_I2Cx_Init()
* 0x61, DB+61<x>: Call HAL_I2C_DeInit() for the I2C interface numbered x
* 0x62, DB+62<x>: Call I2C_Reset() for the I2C interface numbered x
* 0x63, DB+63<x>: Print some fields of the __I2C_HandleTypeDef corresponding to interface numbered x
* 0x6A, DB+6A<x><Addr><Data>: On the I2C interface numbered x, transmit a Write command using I2C_W() to Addr 
                              (formatted as two hex digits) containing Data (will be interpreted as hex).
                              Prints the I2C return status.
* 0x6B, DB+6B<x><Addr><Size>: On the I2C interface numbered x, transmit a Read command using I2C_R() to Addr
                              (formatted as two hex digits) requesting Size bytes. Prints the I2C return status and any data received.
*
* CRD 0x7.
* 0x70, DB+70<Addr>: Call CTR_ReadReset with CTR_ADDR_EVEN if Addr is 0 and with CTR_ADDR_ODD otherwise, and print the count received.
* 0x75, DB+75<ctrChipNo>: Call MUX_SelectCtrChip with ctrChipNo
* 0x76, DB+76<Addr><out0><out1><out2><out3>: Call DAC_SetOutput for DAC_1 if Addr is 1 and DAC_2 if addr is 2,
*                                            with the given outputs (each must be two hex digits).
* 0x77, DB+77<Addr>: Call DAC_ResetOutput for DAC_1 if Addr is 1 and DAC_2 if addr is 2
* 0x7A, DB+7A<Addr>: Call RTD_GetTemp for RTD_1 if Addr is 1 and RTD_2 if addr is 2 and print the received temperature
* 
* EEPROM 0x8.
*
* NATIVE 0xF.
* 0xFF, DB+FF<StringToEcho>: Echo the argument StringToEcho
* 0xF0, DB+F0: Print the current CPU temperature as reported by GetCpuTemperature
*/
void debugCmdParse(char* input, FILE* output);

#endif
