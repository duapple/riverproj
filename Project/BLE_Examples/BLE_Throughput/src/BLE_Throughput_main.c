/******************** (C) COPYRIGHT 2018 STMicroelectronics ********************
 * File Name          : BLE_Throughput_main.c
 * Author             : STMicroelectronics
 * Version            : 1.1.0
 * Date               : March 22th, 2019
 * Description        : Code demonstrating the BLE Throughput applications
 ********************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *******************************************************************************/

/**
 * @file BLE_Throughput_main.c
 * @brief This is a Throughput demo that shows how to implement a simple throughput test  between two BlueNRG,2 devices.
 * Two throughput test options are available: 
 * unidirectional (server notification to client side); 
 * bidirectional (server notification to client side and write without response from client to server side).
 * 

 * \section ATOLLIC_project ATOLLIC project
 To use the project with ATOLLIC TrueSTUDIO for ARM, please follow the instructions below:
 -# Open the ATOLLIC TrueSTUDIO for ARM and select File->Import... Project menu.
 -# Select Existing Projects into Workspace.
 -# Select the ATOLLIC project
 -# Select desired configuration to build from Project->Manage Configurations
 -# Select Project->Rebuild Project. This will recompile and link the entire application
 -# To download the binary image, please connect STLink to JTAG connector in your board (if available).
 -# Select Project->Download to download the related binary image.
 -# Alternatively, open the BlueNRG1 GUI, put the board in bootloader mode and download the built binary image.

 * \section KEIL_project KEIL project
 To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
 -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu.
 -# Open the KEIL project
 <tt> ...\\Project\\BLE_Examples\\BLE_Throughput\\MDK-ARM\\BlueNRG-2\\BLE_Throughput.uvprojx </tt>
 -# Select desired configuration to build
 -# Select Project->Rebuild all target files. This will recompile and link the entire application
 -# To download the binary image, please connect STLink to JTAG connector in your board (if available).
 -# Select Project->Download to download the related binary image.
 -# Alternatively, open the BlueNRG1 GUI, put the board in bootloader mode and download the built binary image.

 * \section IAR_project IAR project
 To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
 -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu.
 -# Open the IAR project
 <tt> ...\\Project\\BLE_Examples\\BLE_Throughput\\EWARM\\BlueNRG-2\\BLE_Throughput.eww </tt>
 -# Select desired configuration to build
 -# Select Project->Rebuild All. This will recompile and link the entire application
 -# To download the binary image, please connect STLink to JTAG connector in your board (if available).
 -# Select Project->Download and Debug to download the related binary image.
 -# Alternatively, open the BlueNRG1 GUI, put the board in bootloader mode and download the built binary image.

 * \subsection Project_configurations Project configurations
 - \c Client_bidirectional - Client role configuration for bidirectional throughput test
 - \c Client_unidirectional - Client role configuration for unidirectional throughput test
 - \c Server_bidirectional - Server role configuration for bidirectional throughput test
 - \c Server_unidirectional - Server role configuration for unidirectional throughput test


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

 The maximum length of the characteristic value is 20 bytes.

 NOTES:
 - The <b>Client_unidirectional</b> and <b>Server_unidirectional </b> workspaces allow to target a unidirectional throughput test: server device sends characteristic notifications to the client device(THROUGHPUT_TEST_SERVER define enabled on on both workspaces, on the preprocessor options). The required serial port baudrate is 921600.
 - Program the client side on one BlueNRG-1 platform and reset it. The platform is seen on the PC as a virtual COM port. Open the port in a serial terminal emulator. 
 - Program the server side on a second BlueNRG-1 platform and reset it. The two platforms try to establish a connection. As soon as they get connected, the slave continuously 
 sends notification of TX characteristic (20 bytes) to the client.
 - After every 500 packets, the measured application unidirectional throughput is displayed.
 
 - The <b>Client_bidirectional</b> and <b>Server_bidirectional</b> workspaces allow to target a bidirectional throughput test: server device sends characteristic notifications to the client device  and client device performs sends characteristic write without responses to the server device (THROUGHPUT_TEST_SERVER and THROUGHPUT_TEST_CLIENT define enabled on both workspaces, on the preprocessor options). The required serial port baudrate is 921600.
 - Program the client side on one BlueNRG-1 platform and reset it. The platform is seen on the PC as a virtual COM port. Open the port in a serial terminal emulator. 
 - Program the server side on a second BlueNRG-1 platform and reset it. The two platforms try to establish a connection. As soon as they get connected, the slave continuously 
 sends notifications of TX characteristic (20 bytes) to the client and the client continuously sends  write without response of RX characteristic (20 bytes) to the server.
 - After every 500 packets, the measured application bidirectional throughput is displayed.

 **/

/** @addtogroup BlueNRG1_demonstrations_applications
 * BlueNRG-1 throughput demo \see BLE_Throughput_main.c for documentation.
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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLE_THROUGHPUT_VERSION_STRING "1.0.0" 

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

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

	/* BlueNRG-1 stack init */
	ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
	if (ret != BLE_STATUS_SUCCESS) {
		printf("Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
		while (1)
			;
	}

#if SERVER
	printf("BlueNRG-1 BLE Throughput Server Application (version: %s, %d,%d,%d)\r\n", BLE_THROUGHPUT_VERSION_STRING, MAX_ATT_MTU, PREPARE_WRITE_LIST_SIZE, MBLOCKS_COUNT);
#else
	printf("BlueNRG-1 BLE Throughput Client Application (version: %s,%d,%d,%d)\r\n", BLE_THROUGHPUT_VERSION_STRING, MAX_ATT_MTU, PREPARE_WRITE_LIST_SIZE, MBLOCKS_COUNT);
#endif

	/* Init Throughput test */
	ret = THROUGHPUT_DeviceInit();
	if (ret != BLE_STATUS_SUCCESS) {
		printf("THROUGHPUT_DeviceInit()--> Failed 0x%02x\r\n", ret);
		while (1)
			;
	}

	printf("BLE Stack Initialized \n");

	while (1) {
		/* BlueNRG-1 stack tick */
		BTLE_StackTick();

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
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}

#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
