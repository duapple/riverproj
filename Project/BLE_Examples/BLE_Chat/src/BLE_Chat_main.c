/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
 * File Name          : BLE_Chat_main.c
 * Author             : STMicroelectronics
 * Version            : 1.1.0
 * Date               : March 22th, 2019
 * Description        : Code demonstrating the BLE Chat application
 ********************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *******************************************************************************/

/**
 * @file BLE_Chat_main.c
 * @brief This is a Chat demo that shows how to implement a simple 2-way communication between two BlueNRG devices.
 * It also provides a reference example about how using the 
 * BLE Over-The-Air (OTA) firmware upgrade capability with the BLE Chat Demo.
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
 -# Alternatively, open the BlueNRG1 Flasher utility and download the built binary image.

 * \section KEIL_project KEIL project
 To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
 -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu.
 -# Open the KEIL project
 <tt> ...\\Project\\BLE_Examples\\BLE_Chat\\MDK-ARM\\BlueNRG-2\\BLE_Chat.uvprojx </tt>
 -# Select desired configuration to build
 -# Select Project->Rebuild all target files. This will recompile and link the entire application
 -# To download the binary image, please connect STLink to JTAG connector in your board (if available).
 -# Select Project->Download to download the related binary image.
 -# Alternatively, open the BlueNRG1 Flasher utility and download the built binary image.

 * \section IAR_project IAR project
 To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
 -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu.
 -# Open the IAR project
 <tt> ...\\Project\\BLE_Examples\\BLE_Chat\\EWARM\\BlueNRG-2\\BLE_Chat.eww </tt>
 -# Select desired configuration to build
 -# Select Project->Rebuild All. This will recompile and link the entire application
 -# To download the binary image, please connect STLink to JTAG connector in your board (if available).
 -# Select Project->Download and Debug to download the related binary image.
 -# Alternatively, open the BlueNRG1 Flasher utility and download the built binary image.

 * \subsection Project_configurations Project configurations
 - \c Client - Client role configuration
 - \c Server - Server role configuration
 - \c Server_HigherApp_OTA - Server role configuration for Higher Application with OTA Service
 - \c Server_LowerApp_OTA - Server role configuration for Lower Application with OTA Service
 - \c Server_Use_OTA_ServiceManager - Server role configuration for Application using OTA Service Manager

 * \section Board_supported Boards supported
 - \c STEVAL-BCN002V1

 * \section Serial_IO Serial I/O
 The application will listen for keys typed in one node and, on return press, it will send them to the remote node.
 The application jumps to the OTA service manager if "OTAServiceManager" is typed and return is pressed.
 The remote node will listen for RF messages and it will output them in the serial port.

 @table
 | Parameter name  | Value               | Unit      |
 -----------------------------------------------------
 | Baudrate        | 115200 [default]    | bit/sec   |
 | Data bits       | 8                   | bit       |
 | Parity          | None                | bit       |
 | Stop bits       | 1                   | bit       |
 @endtable


 * \section Buttons_description Buttons description
 @table

 |  Configuration |                      Server_Use_OTA_ServiceManager                         |
 -----------------------------------------------------------------------------------------------
 |   BUTTON name  |                                  SW1                                       |
 -----------------------------------------------------------------------------------------------
 |    Function    |    Press 5 sec to jump to OTA service Manager *when battery operated*      |

 @endtable

 * \section Usage Usage

 This Chat demo has 2 roles:
 - The server that expose the Chat service. It is the slave.
 - The client that uses the Chat service. It is the master.

 The Chat Service contains 2 Characteristics:
 -# The TX Characteristic: the client can enable notifications on this characteristic. When the server has data to be sent, it will send notifications which will contains the value of the TX Characteristic
 -# The RX Characteristic: it is a writable characteristic. When the client has data to be sent to the server, it will write a value into this characteristic.

 The maximum length of the characteristic value is 20 bytes.

 NOTES:
 - OTA service support for lower or higher application is enabled, respectively,  through ST_OTA_LOWER_APPLICATION=1 or ST_OTA_HIGHER_APPLICATION=1(preprocessor, linker) options and files: OTA_btl.
 - OTA service manager support is enabled, respectively,  through ST_USE_OTA_SERVICE_MANAGER_APPLICATION (preprocessor, linker) options and files: OTA_btl.

 **/

/** @addtogroup BlueNRG1_demonstrations_applications
 * BlueNRG-1 Chat demo \see BLE_Chat_main.c for documentation.
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
#include "chat.h"
#include "SDK_EVAL_Config.h"
#include "Chat_config.h"

#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "OTA_btl.h"
#endif

#include "LPS22HH.h"				// Pressure
#include "LPS22HH_hal.h"
#include "LSM6DSO.h"				// Accelerometer and Gyroscope
#include "LSM6DSO_hal.h"
#include "HTS221.h"					// Humidity
#include "HTS221_hal.h"
#include "LIS2MDL.h"				// Magnetometer
#include "LIS2MDL_hal.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLE_CHAT_VERSION_STRING "1.0.0" 

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void ADC_DMA_Configuration(uint8_t xMode);
void GPIO_Configuration(void);
void Interrupts_EXT_IO_Config(void);
/* Private functions ---------------------------------------------------------*/

int main(void) {
	uint8_t ret;

	/* System Init */
	SystemInit();

	/* Identify BlueNRG1 platform */
	SdkEvalIdentification();

	/* Init Clock */
	Clock_Init();

	/* Configure I/O communication channel:
	 It requires the void IO_Receive_Data(uint8_t * rx_data, uint16_t data_size) function
	 where user received data should be processed */
	SdkEvalComIOConfig(Process_InputData);

	/* LED configuration */
	SdkEvalLedInit(LED1);
	SdkEvalLedInit(LED2);
	SdkEvalLedInit(LED3);

	/* BlueNRG-1 stack init */
	ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
	if (ret != BLE_STATUS_SUCCESS) {
		printf("Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
		SdkEvalLedOn(LED1);
		while (1)
			;
	}

#if SERVER
	printf("BlueNRG-1 BLE Chat Server Application (version: %s)\r\n", BLE_CHAT_VERSION_STRING);
#else
	printf("BlueNRG-1 BLE Chat Client Application (version: %s)\r\n", BLE_CHAT_VERSION_STRING);
#endif

	/* Init Chat Device */
	ret = CHAT_DeviceInit();
	if (ret != BLE_STATUS_SUCCESS) {
		printf("CHAT_DeviceInit()--> Failed 0x%02x\r\n", ret);
		SdkEvalLedOn(LED1);
		while (1)
			;
	}

	GPIO_Configuration();

	/* Configure I2C @ 400 kHz */
	SdkEvalI2CInit(400000);

	lsm6dso_xl_data_rate_set(0, LSM6DSO_XL_ODR_OFF);
	lsm6dso_gy_data_rate_set(0, LSM6DSO_GY_ODR_OFF);
	lsm6dso_pin_mode_set(0, LSM6DSO_OPEN_DRAIN);
	lsm6dso_pin_polarity_set(0, LSM6DSO_ACTIVE_LOW);
	lps22hh_data_rate_set(0, LPS22HH_POWER_DOWN);
	lis2mdl_operating_mode_set(0, LIS2MDL_POWER_DOWN);
	HTS221_Set_IrqOutputType(0, HTS221_OPENDRAIN);
	HTS221_Set_PowerDownMode(0, HTS221_SET);
	GPIO_WriteBit(GPIO_Pin_7, Bit_RESET);

	printf("BLE Stack Initialized \n");

	while (1) {

		/* BlueNRG-1 stack tick */
		BTLE_StackTick();

		/* Application tick */
		APP_Tick();

#if ST_OTA_FIRMWARE_UPGRADE_SUPPORT
		/* Check if the OTA firmware upgrade session has been completed */
		if (OTA_Tick() == 1) {
			/* Jump to the new application */
			OTA_Jump_To_New_Application();
		}
#endif  /* ST_OTA_FIRMWARE_UPGRADE_SUPPORT */

	}

} /* end main() */


 /*******************************************************************************
  * Function Name  : GPIO_Configuration.
  * Description    : Configure outputs GPIO pins
  * Input          : None
  * Return         : None
  *******************************************************************************/
 void GPIO_Configuration(void) {

 	GPIO_InitType GPIO_InitStructure;

 	/** GPIO Periph clock enable */
 	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);

 	/** Init Structure */
 	GPIO_StructInit(&GPIO_InitStructure);

 	/** Configure GPIO_Pin_7 for Proximity XSHUT*/
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Output;
 	GPIO_InitStructure.GPIO_Pull = DISABLE;
 	GPIO_InitStructure.GPIO_HighPwr = ENABLE;
 	GPIO_Init(&GPIO_InitStructure);

 	GPIO_WriteBit(GPIO_Pin_7, Bit_RESET);

 }

 /*******************************************************************************
  * Function Name  : Interrupts_EXT_IO_Config.
  * Description    : Configure interrupts on GPIO pins
  * Input          : None
  * Return         : None
  *******************************************************************************/
 void Interrupts_EXT_IO_Config(void) {

 	GPIO_InitType GPIO_InitStructure;
 	GPIO_EXTIConfigType exti_config;
 	NVIC_InitType NVIC_InitStructure;

 	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_GPIO, ENABLE);

 	NVIC_InitStructure.NVIC_IRQChannel = GPIO_IRQn;
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = MED_PRIORITY;
 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 	NVIC_Init(&NVIC_InitStructure);

 	/* Init Structure */
 	GPIO_StructInit(&GPIO_InitStructure);

 	/* Configure Wakeup IO pin */
 	GPIO_InitStructure.GPIO_Mode = GPIO_Input;
 	GPIO_InitStructure.GPIO_HighPwr = DISABLE;
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
 	GPIO_InitStructure.GPIO_Pull = DISABLE;
 	GPIO_Init(&GPIO_InitStructure);

 	/* Configure the Interrupt */
 	exti_config.GPIO_Pin = GPIO_Pin_11;
 	exti_config.GPIO_IrqSense = GPIO_IrqSense_Level;
 	exti_config.GPIO_Event = GPIO_Event_Low;
 	GPIO_EXTIConfig(&exti_config);

 	GPIO_EXTICmd(GPIO_Pin_11, ENABLE);

 }

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 */
void assert_failed(uint8_t* file, uint32_t line) {
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
