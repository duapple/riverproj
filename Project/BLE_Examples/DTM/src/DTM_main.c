/******************** (C) COPYRIGHT 2018 STMicroelectronics ********************
 * File Name          : DTM_main.c
 * Author             : STMicroelectronics
 * Version            : 1.1.0
 * Date               : March 21th, 2019
 * Description        : DTM application which configures a BlueNRG-2 device as a network coprocessor (UART configuration) in order to be used with the BlueNRG GUI or other instruments as CBT
 ********************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *******************************************************************************/

/**
 * @file DTM_main.c
 * @brief This application configures a BlueNRG-2 device as a network coprocessor (UART) in order to be used with the BlueNRG GUI or other instruments as CBT. Full stack  modular configuration option is used.
 * 

 * \section ATOLLIC_project ATOLLIC project
 To use the project with ATOLLIC TrueSTUDIO for ARM, please follow the instructions below:
 -# Open the ATOLLIC TrueSTUDIO for ARM and select File->Import... Project menu.
 -# Select Existing Projects into Workspace.
 -# Select the ATOLLIC project
 -# Select desired configuration to build from Project->Manage Configurations
 -# Select Project->Rebuild Project. This will recompile and link the entire application

 * \section KEIL_project KEIL project
 To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
 -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu.
 -# Open the KEIL project
 <tt> ...\\Project\\BLE_Examples\\DTM\\MDK-ARM\\BlueNRG-2\\DTM.uvprojx </tt>
 -# Select desired configuration to build
 -# Select Project->Rebuild all target files. This will recompile and link the entire application
 -# To download the binary image, please connect STLink to JTAG connector in your board (if available).
 -# Select Project->Download to download the related binary image.
 -# Alternatively, open the BlueNRG1 Flasher utility and download the built binary image.

 * \section IAR_project IAR project
 To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
 -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu.
 -# Open the IAR project
 <tt> ...\\Project\\BLE_Examples\\DTM\\EWARM\\BlueNRG-2\\DTM.eww </tt>
 -# Select desired configuration to build
 -# Select Project->Rebuild All. This will recompile and link the entire application
 -# To download the binary image, please connect STLink to JTAG connector in your board (if available).
 -# Select Project->Download and Debug to download the related binary image.
 -# Alternatively, open the BlueNRG1 Flasher utility and download the built binary image.

 * \subsection Project_configurations Project configurations
 - \c UART - Network coprocessor configuration: UART mode


 * \section Board_supported Boards supported
 - \c STEVAL-BCN002V1

 * \section Serial_IO Serial I/O
 @table
 | Parameter name  | Value            | Unit      |
 --------------------------------------------------
 | Baudrate        | 115200 [default] | bit/sec   |
 | Data bits       | 8                | bit       |
 | Parity          | None             | bit       |
 | Stop bits       | 1                | bit       |
 @endtable


 * \section Usage Usage
 One network coprocessor configurations are available:
 - UART (DTM binary file: DTM_UART.hex)

 **/

/** @addtogroup BlueNRG1_demonstrations_applications
 *  BlueNRG-1 DTM application \see DTM_main.c for documentation.
 *
 *@{
 */
/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "BlueNRG1_conf.h"
#include "bluenrg1_stack.h"
#include "DTM_boot.h"
#include "sleep.h"
#include "transport_layer.h"
#include "hw_config.h"
#include "miscutil.h" 
#include "DTM_cmd_db.h"

#define RESET_REASON_WDG        ((uint8_t)0x05)
#define RESET_REASON_LOCKUP     ((uint8_t)0x06)
#define RESET_REASON_BOR        ((uint8_t)0x07)
#define RESET_REASON_CRASH      ((uint8_t)0x08)

/* Add aci_blue_initialized_event() prototype */
void aci_blue_initialized_event(uint8_t Reason_Code);

/* Add aci_blue_crash_info_event() prototype */
void aci_blue_crash_info_event(uint8_t Crash_Type, uint32_t SP, uint32_t R0, uint32_t R1, uint32_t R2, uint32_t R3, uint32_t R12, uint32_t LR, uint32_t PC, uint32_t xPSR, uint8_t Debug_Data_Length, uint8_t Debug_Data[]);

int main(void) {
	crash_info_t crash_info;

	/* System Init */
	DTM_SystemInit();

	/* Stack Initialization */
	DTM_StackInit();

	/* Transport Layer Init */
	transport_layer_init();

	/* Get crash info */
	HAL_GetCrashInfo(&crash_info);

#ifdef LL_ONLY
	uint8_t Value = 1;
	aci_hal_write_config_data(0x2C, 1, &Value);

#else
	RESET_REASON_Type xResetReason;

	uint8_t reset_reason = 0x01;

	/* EVT_BLUE_INITIALIZED */
	xResetReason = SysCtrl_GetWakeupResetReason();
	if (xResetReason == RESET_WDG) {
		reset_reason = RESET_REASON_WDG;
	} else if (xResetReason == RESET_LOCKUP) {
		reset_reason = RESET_REASON_LOCKUP;
	} else if (xResetReason == RESET_BLE_BOR) {
		reset_reason = RESET_REASON_BOR;
	}
	if ((crash_info.signature & 0xFFFF0000) == CRASH_SIGNATURE_BASE) {
		reset_reason = RESET_REASON_CRASH;
	}

	aci_blue_initialized_event(reset_reason);

#endif

	if ((crash_info.signature & 0xFFFF0000) == CRASH_SIGNATURE_BASE) {
		aci_blue_crash_info_event(crash_info.signature & 0xFF, crash_info.SP, crash_info.R0, crash_info.R1, crash_info.R2, crash_info.R3, crash_info.R12, crash_info.LR, crash_info.PC, crash_info.xPSR, 0,
		NULL);
	}

	while (1) {
		/* BlueNRG-1 stack tick */
		BTLE_StackTick();
		transport_layer_tick();
		BlueNRG_Sleep(SLEEPMODE_NOTIMER, IO_WAKEUP_PIN, 0); // 4: IO11 0: low level
	}
}
