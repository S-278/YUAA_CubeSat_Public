/*!
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @file PeriphHelper.c
* @brief Peripherals helper C File
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* @author            Elijah B.
* @details           Defines peripheral helper code
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INCLUDES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#include "PeriphHelper.h"
#include "system_manager.h"
#include "FreeRTOS.h"
#include "task.h"
#include "MCU_Init.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#ifdef DEBUG_ENABLED
#include <string.h>
#include <stdio.h>
#endif

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL DEFINES
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#define EPS_INTF hi2c1

#define GGB_I2C_ADDRESS 0x5D
#define GGB_I2C_INTERFACE hi2c2

#define CTR_I2C_INTF hi2c3
#define CTR_ADDR_EVEN (0x50) //I2C address of the 4 even-numbered counters
#define CTR_ADDR_ODD (0x51) //I2C address of the 4 odd-numbered counters

#define DAC_I2C_INTERFACE hi2c3
#define DAC_1_ADDR (0x28)
#define DAC_2_ADDR (0x29)

#define MUX_ADDR (0x70)
#define MUX_INTF hi2c3

#define RTD_1_ADDR (0x48)
#define RTD_2_ADDR (0x49)
#define RTD_intf hi2c3

#define SS_SPI_INTERFACE hspi1
#define CS_PIN_PERIPH GPIOA
#define CS_PIN SPI1_CS_Pin

#define TCV_I2C_INTERFACE hi2c1

#ifdef DEBUG_ENABLED
/* This constant is an awkward hack to allow code to access preprocessor symbols
as memory at runtime. This is worth it because it eliminates a LOT of code duplication. */
static const uint32_t S_debugDevsEnabled = 0
#ifdef DEBUG_EPS
| 1 << DEV_EPS
#endif 
#ifdef DEBUG_TCV
| 1 << DEV_TCV
#endif
#ifdef DEBUG_SS
| 1 << DEV_SS
#endif
#ifdef DEBUG_CRD
| 1 << DEV_DAC_1 | 1 << DEV_DAC_2 | 1 << DEV_MUX | 1 << DEV_CTR_0 | 1 << DEV_CTR_1 | 1 << DEV_RTD_1 | 1 << DEV_RTD_2
#endif
#ifdef DEBUG_GGB
| 1 << DEV_GGB_CTRL
#endif
;
#define DEBUG_isDevEnabled(x) S_debugDevsEnabled & (1 << x)
#endif

#define TRANSACT_PRIO (configMAX_PRIORITIES - 3) // Dynamic priority of any task executing a transaction

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL TYPES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/* Enum of only those devices which are supported by Peripherals Helper.
   This is kept internal to avoid crowding the public namespace */
typedef enum {
	PH_DEV_EPS = 0,

	PH_DEV_GGB_CTRL,

	PH_DEV_CTR_0,
	PH_DEV_CTR_1,

	PH_DEV_DAC_1,
	PH_DEV_DAC_2,

	PH_DEV_MUX,

	PH_DEV_RTD_1,
	PH_DEV_RTD_2,

	PH_DEV_SS,

	PH_DEV_TCV,

	PH_DEV_EEPROM,

	PH_DEV_RTC,

	NUM_PH_DEVS
} PeriphHelper_Dev_e;

typedef struct {
	UBaseType_t normalPrio;
	UBaseType_t nestCt;
} TransactData_t;

#ifdef DEBUG_ENABLED
typedef struct {
	const uint8_t* expectedMsg;
	uint16_t msgSz;
} ExpectedTx_t;
#endif 


/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL VARIABLES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/
#ifdef DEBUG_ENABLED
static ExpectedTx_t ExpectedTxData[NUM_PH_DEVS]; 
static uint8_t expectedTx_initialized = 0;
#endif

/* Static array to hold transaction metadata,
   one entry per device supported by Peripherals Helper. */
static TransactData_t S_TransactData[NUM_PH_DEVS] = {
	[PH_DEV_EPS] = { .nestCt = 0 },
	/* ... all other members are empty-initialized */
};

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* INTERNAL ROUTINES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

static PeriphHelper_Dev_e enumMap(DevName_e dev)
{
	switch (dev)
	{
	case DEV_EPS:
		return PH_DEV_EPS;

	case DEV_GGB_CTRL:
		return PH_DEV_GGB_CTRL;

	case DEV_CTR_0:
		return PH_DEV_CTR_0;
	case DEV_CTR_1:
		return PH_DEV_CTR_1;

	case DEV_DAC_1:
		return PH_DEV_DAC_1;
	case DEV_DAC_2:
		return PH_DEV_DAC_2;

	case DEV_MUX:
		return PH_DEV_MUX;

	case DEV_RTD_1:
		return PH_DEV_RTD_1;
	case DEV_RTD_2:
		return PH_DEV_RTD_2;

	case DEV_SS:
		return PH_DEV_SS;

	case DEV_TCV:
		return PH_DEV_TCV;

	case DEV_EEPROM:
		return PH_DEV_EEPROM;

	case DEV_RTC:
		return PH_DEV_RTC;

	default:
		assert(0);
	}
}

#ifdef DEBUG_ENABLED
static void debugPrintIntf(void* intfAddr)
{
	if (intfAddr == &hi2c1) fprintf(DEBUG_COMM, "SYS_I2C (hi2c1)");
	else if (intfAddr == &hi2c2) fprintf(DEBUG_COMM, "INT_I2C (hi2c2)");
	else if (intfAddr == &hi2c3) fprintf(DEBUG_COMM, "PAYLOAD_I2C (hi2c3)");
	else if (intfAddr == &hspi1) fprintf(DEBUG_COMM, "COMMS SPI (hspi1)");
}
static void debugPrint_I2C_R_Prompt(void* intfAddr, uint16_t devAddr, uint16_t size)
{
	debugPrintIntf(intfAddr);
	fprintf(DEBUG_COMM, " @ 0x%02X << R for %u bytes\n", devAddr, size);
	debugPrintIntf(intfAddr);
	fprintf(DEBUG_COMM, " @ 0x%02X >> ", devAddr);
}
static void debugPrint_I2C_W_Prompt(void* intfAddr, uint16_t devAddr)
{
	debugPrintIntf(intfAddr);
	fprintf(DEBUG_COMM, " @ 0x%02X << W ", devAddr);
}
#endif

/*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* EXTERNAL ROUTINES DEFINITIONS
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

ES_ReturnType Periph_BeginTransact(DevName_e dev, uint16_t minSz, uint32_t timeout)
{
	/* IMPORTANT: this implementation assumes
	   a task does not relinquish CPU control 
	   between beginning and ending a transaction! */

	// TODO: this is a naive implementation. Optimal implementation involves:
	// 1. Take mutex with timeout
	// 2. Elevate priority and cache previous priority

	PeriphHelper_Dev_e ph_dev = enumMap(dev);
	if (S_TransactData[ph_dev].nestCt == 0) // No transaction exists on this device yet
	{
		if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
		{
			// Important: elevate priority before modifying static data, otherwise not atomic!
			UBaseType_t normalPrio = uxTaskPriorityGet(NULL);
			vTaskPrioritySet(NULL, TRANSACT_PRIO);
			S_TransactData[ph_dev].normalPrio = normalPrio;
		}
		S_TransactData[ph_dev].nestCt = 1;
		if (dev == DEV_SS)
		{
			HAL_GPIO_WritePin(CS_PIN_PERIPH, CS_PIN, GPIO_PIN_RESET);
#if defined(DEBUG_ENABLED) && defined(DEBUG_L3)
			debugPrintIntf(&SS_SPI_INTERFACE);
			fprintf(DEBUG_COMM, ": chip select\n");
#endif
		}
	}
	else // Assume nested call 
	{
		S_TransactData[ph_dev].nestCt++;
	}
	return E_OK;
}

void Periph_EndTransact(DevName_e dev)
{
	// TODO: this is a naive implementation. Optimal implementation involves:
	// 1. Return mutex
	// 2. Restore priority from cache

	PeriphHelper_Dev_e ph_dev = enumMap(dev);
	assert(S_TransactData[ph_dev].nestCt > 0);

	if (--S_TransactData[ph_dev].nestCt == 0)
	{
		if (dev == DEV_SS)
		{
			HAL_GPIO_WritePin(CS_PIN_PERIPH, CS_PIN, GPIO_PIN_SET);
#if defined(DEBUG_ENABLED) && defined(DEBUG_L3)
			debugPrintIntf(&SS_SPI_INTERFACE);
			fprintf(DEBUG_COMM, ": chip unselect\n");
#endif
		}
		if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
		{
			vTaskPrioritySet(NULL, S_TransactData[ph_dev].normalPrio);
		}
	}
}

void SharedMem_BeginAccess()
{
	/* Shared memory accesses probably don't take longer than a couple ms.
	As such, using a scheduler suspend section probably doesn't run a big risk
	of missing a watchdog refresh, and scheduler suspends are a lot faster and simpler
	than dynamic priority elevation. However, priority elevation is definitely safer. */
	vTaskSuspendAll();
}

void SharedMem_EndAccess()
{
	xTaskResumeAll();
}

HAL_StatusTypeDef Periph_Comm(PeriphComms_e type, DevName_e dev, uint8_t* data, uint16_t size, uint32_t timeout)
{
	assert(data != NULL);
	assert(type == PERIPH_Rx || type == PERIPH_Tx);
	// TODO: this is a naive implementation. Optimal implementation involves using HAL interrupt/DMA mode.

	switch (type)
	{
	case PERIPH_Rx:
	{
		switch (dev)
		{
		case DEV_EPS:
		case DEV_GGB_CTRL:
		case DEV_CTR_0:
		case DEV_CTR_1:
		case DEV_DAC_1:
		case DEV_DAC_2:
		case DEV_MUX:
		case DEV_RTD_1:
		case DEV_RTD_2:
		case DEV_TCV:
		{
			I2C_HandleTypeDef* hi2c; uint16_t i2c_addr;
			switch (dev)
			{
			case DEV_EPS:
			{ hi2c = &EPS_INTF; i2c_addr = EPS_I2C_ADDRESS; break; }
			case DEV_GGB_CTRL:
			{ hi2c = &GGB_I2C_INTERFACE; i2c_addr = GGB_I2C_ADDRESS; break; }
			case DEV_CTR_0:
			{ hi2c = &CTR_I2C_INTF; i2c_addr = CTR_ADDR_EVEN; break; }
			case DEV_CTR_1:
			{ hi2c = &CTR_I2C_INTF; i2c_addr = CTR_ADDR_ODD; break; }
			case DEV_DAC_1:
			{ hi2c = &DAC_I2C_INTERFACE; i2c_addr = DAC_1_ADDR; break; }
			case DEV_DAC_2:
			{ hi2c = &DAC_I2C_INTERFACE; i2c_addr = DAC_2_ADDR; break; }
			case DEV_MUX:
			{ hi2c = &MUX_INTF; i2c_addr = MUX_ADDR; break; }
			case DEV_RTD_1:
			{ hi2c = &RTD_intf; i2c_addr = RTD_1_ADDR; break; }
			case DEV_RTD_2:
			{ hi2c = &RTD_intf; i2c_addr = RTD_2_ADDR; break; }
			case DEV_TCV:
			{ hi2c = &TCV_I2C_INTERFACE; i2c_addr = UHF_I2C_ADDRESS; break; }
			default: assert(0);
			}

#ifdef DEBUG_ENABLED
			if (DEBUG_isDevEnabled(dev))
#endif
				return HAL_I2C_Master_Receive(hi2c, i2c_addr << 1, data, size, timeout);
#ifdef DEBUG_ENABLED
			else 
			{
				debugPrint_I2C_R_Prompt(hi2c, i2c_addr, size);
				break;
			}
#endif
		}
		case DEV_SS:
		{
#ifdef DEBUG_ENABLED
			if (DEBUG_isDevEnabled(dev))
#endif
				return HAL_SPI_Receive(&SS_SPI_INTERFACE, data, size, timeout);
#ifdef DEBUG_ENABLED
			else
			{
				debugPrintIntf(&SS_SPI_INTERFACE);
				fprintf(DEBUG_COMM, " >> ");
				break;
			}
#endif
		}
		default:
			assert(0);
		}
#ifdef DEBUG_ENABLED
		ESTTC_STOP_RX;
		HAL_UART_Receive(DEBUG_UART, data, size, HAL_MAX_DELAY);
		fprintf(DEBUG_COMM, "\n");
		ESTTC_RESUME_RX;
		return HAL_OK;
#endif
		break;
	}
	case PERIPH_Tx:
	{
#ifdef DEBUG_ENABLED
		PeriphHelper_Dev_e ph_dev = enumMap(dev);
		if (ExpectedTxData[ph_dev].expectedMsg != NULL && ExpectedTxData[ph_dev].msgSz > 0)
		{
			HAL_StatusTypeDef ret = HAL_OK;
			if (ExpectedTxData[ph_dev].msgSz != size \
				|| memcmp(ExpectedTxData[ph_dev].expectedMsg, data, size) != 0)\
				ret = HAL_ERROR;
			ExpectedTxData[ph_dev].expectedMsg = NULL; ExpectedTxData[ph_dev].msgSz = 0;
			if (ret == HAL_ERROR) return ret;
		}
#endif // Expected Tx checking
		switch (dev)
		{
		case DEV_EPS:
		case DEV_GGB_CTRL:
		case DEV_CTR_0:
		case DEV_CTR_1:
		case DEV_DAC_1:
		case DEV_DAC_2:
		case DEV_MUX:
		case DEV_RTD_1:
		case DEV_RTD_2:
		case DEV_TCV:
		{
			I2C_HandleTypeDef* hi2c; uint16_t i2c_addr;
			switch (dev)
			{
			case DEV_EPS:
			{ hi2c = &EPS_INTF; i2c_addr = EPS_I2C_ADDRESS; break; }
			case DEV_GGB_CTRL:
			{ hi2c = &GGB_I2C_INTERFACE; i2c_addr = GGB_I2C_ADDRESS; break; }
			case DEV_CTR_0:
			{ hi2c = &CTR_I2C_INTF; i2c_addr = CTR_ADDR_EVEN; break; }
			case DEV_CTR_1:
			{ hi2c = &CTR_I2C_INTF; i2c_addr = CTR_ADDR_ODD; break; }
			case DEV_DAC_1:
			{ hi2c = &DAC_I2C_INTERFACE; i2c_addr = DAC_1_ADDR; break; }
			case DEV_DAC_2:
			{ hi2c = &DAC_I2C_INTERFACE; i2c_addr = DAC_2_ADDR; break; }
			case DEV_MUX:
			{ hi2c = &MUX_INTF; i2c_addr = MUX_ADDR; break; }
			case DEV_RTD_1:
			{ hi2c = &RTD_intf; i2c_addr = RTD_1_ADDR; break; }
			case DEV_RTD_2:
			{ hi2c = &RTD_intf; i2c_addr = RTD_2_ADDR; break; }
			case DEV_TCV:
			{ hi2c = &TCV_I2C_INTERFACE; i2c_addr = UHF_I2C_ADDRESS; break; }
			default: assert(0);
			}

#ifdef DEBUG_ENABLED
			if (DEBUG_isDevEnabled(dev))
#endif
				return HAL_I2C_Master_Transmit(hi2c, i2c_addr << 1, data, size, timeout);
#ifdef DEBUG_ENABLED
			else
			{
				debugPrint_I2C_W_Prompt(hi2c, i2c_addr);
				break;
			}
#endif
		}
		case DEV_SS:
		{
#ifdef DEBUG_ENABLED
			if (DEBUG_isDevEnabled(dev))
#endif
				return HAL_SPI_Transmit(&SS_SPI_INTERFACE, data, size, timeout);
#ifdef DEBUG_ENABLED
			else
			{
				debugPrintIntf(&SS_SPI_INTERFACE);
				fprintf(DEBUG_COMM, " << ");
				break;
			}
#endif
		}
		default:
			assert(0);
		}
#ifdef DEBUG_ENABLED
		for (uint16_t i = 0; i < size; i++)
		{
			unsigned int tmp = (unsigned int)data[i];
			fprintf(DEBUG_COMM, "%02X ", tmp);
		}
		fprintf(DEBUG_COMM, "\n");
		return HAL_OK;
#endif
		break;
	}
	default:
		assert(0);
	}
}

#ifdef DEBUG_ENABLED
void Periph_ExpectTx(DevName_e dev, const uint8_t* expectedMsg, uint16_t expectedSz)
{
	vTaskSuspendAll();
	if (!expectedTx_initialized) { memset(ExpectedTxData, 0, sizeof(ExpectedTxData)); expectedTx_initialized = 1; }
	PeriphHelper_Dev_e ph_dev = enumMap(dev);
	ExpectedTxData[ph_dev].expectedMsg = expectedMsg;
	ExpectedTxData[ph_dev].msgSz = expectedSz;
	xTaskResumeAll();
}
#endif // DEBUG_ENABLED
