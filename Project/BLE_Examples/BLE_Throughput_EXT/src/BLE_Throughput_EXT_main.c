/******************** (C) COPYRIGHT 2018 STMicroelectronics ********************
 * File Name          : BLE_Throughput_EXT_main.c
 * Author             : STMicroelectronics
 * Version            : 1.1.0
 * Date               : March 21th, 2019
 * Description        : Code demonstrating the BlueNRG-2 BLE Throughput with data length extension feature
 ********************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *******************************************************************************/

/**
 * @file BLE_Throughput_EXT_main.c
 * @brief This is a Throughput demo that shows how to implement some throughput tests  between two BlueNRG-2 devices using the ATT MTU size up to 247 bytes and data length extension feature
 * Several Throughput options are available: 
 * Server notification to client side with data length extension: 27, 100, 251 bytes
 * Server performs att mtu exchange to modify att mtu size
 * Bidirectional communication (server: notification and client: write without response) can be turned on /off
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
 <tt> ...\\Project\\BLE_Examples\\BLE_Throughput_EXT\\MDK-ARM\\BlueNRG-2\\BLE_Throughput_EXT.uvprojx </tt>
 -# Select desired configuration to build
 -# Select Project->Rebuild all target files. This will recompile and link the entire application
 -# To download the binary image, please connect STLink to JTAG connector in your board (if available).
 -# Select Project->Download to download the related binary image.
 -# Alternatively, open the BlueNRG1 GUI, put the board in bootloader mode and download the built binary image.

 * \section IAR_project IAR project
 To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
 -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu.
 -# Open the IAR project
 <tt> ...\\Project\\BLE_Examples\\BLE_Throughput_EXT\\EWARM\\BlueNRG-2\\BLE_Throughput_EXT.eww </tt>
 -# Select desired configuration to build
 -# Select Project->Rebuild All. This will recompile and link the entire application
 -# To download the binary image, please connect STLink to JTAG connector in your board (if available).
 -# Select Project->Download and Debug to download the related binary image.
 -# Alternatively, open the BlueNRG1 GUI, put the board in bootloader mode and download the built binary image.

 * \subsection Project_configurations Project configurations
 - \c Client - Client role configuration for throughput test
 - \c Server - Server role configuration for throughput test


 * \section Board_supported Boards supported
 - \c STEVAL-BCN002V1

 * \section Serial_IO Serial I/O
 
 @table
 | Parameter name  | Value               | Unit      |
 ------------------------------------------------------
 | Baudrate        | 921600              |  bit/sec  |
 | Data bits       | 8                   | bit       |
 | Parity          | None                | bit       |
 | Stop bits       | 1                   | bit       |
 @endtable


 * \section Usage Usage

 This Throughput demo has 2 roles:
 - The server that expose the Throughput service. It is the slave.
 - The client that uses the Throughput service. It is the master.

 The Throughput Service contains 2 Characteristics:
 -# The TX Characteristic: the client can enable notifications on this characteristic. When the server has data to be sent, it sends notifications which contains the value of the TX Characteristic
 -# The RX Characteristic: it is a writable characteristic. When the client has data to be sent to the server, it writes a value into this characteristic.

 The following interactive options are available on Server side (open hyperteminal with 921600, 8,None,1, None):
 - Press u to Send data len update request for 27 bytes
 - Press m to Send data len update request for 100 bytes
 - Press U to Send data len update request for 251 bytes
 - Press a to Send ATT_MTU exchange
 - Press c to Send connection parameter update request
 - Press f to Enable/disable flushable PDUs
 - Press p to Print APP flags
 - Press ? to Print help

 NOTES:
 - The <b>Client</b> and <b>Server</b> workspaces (by default) allow to target a unidirectional throughput test: server device sends characteristic notifications (20 bytes) to the client device. The required serial port baudrate is 921600.Test 
 - Program the client side on one BlueNRG-2 platform and reset it. The platform is seen on the PC as a virtual COM port. Open the port in a serial terminal emulator. The required serial port baudrate is 921600
 - Program the server side on a second BlueNRG-2 platform and reset it. The platform is seen on the PC as a virtual COM port. Open the port in a serial terminal emulator. The required serial port baudrate is 921600.
 -  The two platforms try to establish a connection. As soon as they get connected, the slave continuously sends notification of a characteristic to the client.
 - User can play with the provided commands in order to select the following link layer packet length: 27 (default),100 and 251 (maximum allowed) bytes. 
 - User can also performs an ATT MTU exchange command on server side in order to allow server to increase the ATT_MTU size (247 bytes)
 - He can also enable the bidirectional throughput on client side (client writes on RX characteristic)
 

 **/

/** @addtogroup BlueNRG1_demonstrations_applications
 * BlueNRG-1 extended packet length throughput demo \see BLE_Throughput_EXT_main.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "ble_const.h"
#include "bluenrg1_stack.h"
#include "gp_timer.h"
#include "app_state.h"
#include "throughput.h"
#include "SDK_EVAL_Config.h"
#include "Throughput_config.h"

#define BLE_THROUGHPUT_VERSION_STRING "x.y.z"

void Process_InputData(uint8_t* data_buffer, uint16_t Nb_bytes);

int main(void) {
	uint8_t ret;

	/* System Init */
	SystemInit();

	/* Identify BlueNRG1 platform */
	SdkEvalIdentification();

	/* Init Clock */
	Clock_Init();

	/* Init the UART peripheral */
	SdkEvalComUartInit(UART_BAUDRATE);

	/* Configure I/O communication channel:
	 It requires the void IO_Receive_Data(uint8_t * rx_data, uint16_t data_size) function
	 where user received data should be processed */
	SdkEvalComIOConfig(Process_InputData);

	/* BlueNRG-1 stack init */
	ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
	if (ret != BLE_STATUS_SUCCESS) {
		printf("Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
		while (1) {
		}
	}

#if SERVER
	printf("BlueNRG-1 BLE Throughput Server Application (version: %s)\r\n", BLE_THROUGHPUT_VERSION_STRING);
#else
	printf("BlueNRG-1 BLE Throughput Client Application (version: %s)\r\n", BLE_THROUGHPUT_VERSION_STRING);
#endif

	/* Init Throughput test */
	ret = DeviceInit();
	if (ret != BLE_STATUS_SUCCESS) {
		printf("DeviceInit()--> Failed 0x%02x\r\n", ret);
		while (1) {
		}
	}
	printf("BLE Stack Initialized \n");
	print_help();
	while (1) {
		NVIC_DisableIRQ(UART_IRQn);
		/* BlueNRG-1 stack tick */
		BTLE_StackTick();
		NVIC_EnableIRQ(UART_IRQn);

		/* Application tick */
		APP_Tick();
	}
} /* end main() */

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 * ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}

#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
