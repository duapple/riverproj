/******************** (C) COPYRIGHT 2018 STMicroelectronics ********************
 * File Name          : BLE_HID_Peripheral_main.c
 * Author             : STMicroelectronics
 * Version            : 1.1.0
 * Date               : March 22th, 2019
 * Description        : Code demonstrating the HID/HOGP Bluetooth Low Energy application profile.
 ********************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *******************************************************************************/

/**
 * @file BLE_HID_Peripheral_main.c
 * @brief This application contains an example which shows how implementing a device that uses the standard 
 * HID/HOGP Bluetooth Low Energy application profile. The device implemented to demo are keyboard and mouse. 
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
 <tt> ...\\Project\\BLE_Examples\\BLE_HID_Peripheral\\MDK-ARM\\BlueNRG-2\\HID_Peripheral.uvprojx </tt>
 -# Select desired configuration to build
 -# Select Project->Rebuild all target files. This will recompile and link the entire application
 -# To download the binary image, please connect STLink to JTAG connector in your board (if available).
 -# Select Project->Download to download the related binary image.
 -# Alternatively, open the BlueNRG1 Flasher utility and download the built binary image.

 * \section IAR_project IAR project
 To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
 -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu.
 -# Open the IAR project
 <tt> ...\\Project\\BLE_Examples\\BLE_HID_Peripheral\\EWARM\\BlueNRG-2\\HID_Peripheral.eww </tt>
 -# Select desired configuration to build
 -# Select Project->Rebuild All. This will recompile and link the entire application
 -# To download the binary image, please connect STLink to JTAG connector in your board (if available).
 -# Select Project->Download and Debug to download the related binary image.
 -# Alternatively, open the BlueNRG1 Flasher utility and download the built binary image.

 * \subsection Project_configurations Project configurations
 - \c HID_Keyboard - HID Keyboard device implementation
 - \c HID_Mouse - HID Mouse device implementation


 * \section Board_supported Boards supported
 - \c STEVAL-BCN002V1


 * \section Serial_IO Serial I/O
 @table
 | Parameter name  | Value            | Unit      |
 ----------------------------------------------------
 | Baudrate        | 115200 [default] | bit/sec   |
 | Data bits       | 8                | bit       |
 | Parity          | None             | bit       |
 | Stop bits       | 1                | bit       |
 @endtable


 * \section Buttons_description Buttons description
 @table
 |                |                                              HID_Keyboard                                               |||||                                                                                                                 HID_Mouse                                                                                                                 |||||
 --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 |   BUTTON name  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |                STEVAL-IDB007V1               |                STEVAL-IDB007V2               |                STEVAL-IDB008V1               |                STEVAL-IDB008V2               |                STEVAL-IDB009V1               |
 --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 |       SW1      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |  Left mouse button and exit from deep sleep  |  Left mouse button and exit from deep sleep  |  Left mouse button and exit from deep sleep  |  Left mouse button and exit from deep sleep  |  Left mouse button and exit from deep sleep  |
 |      RESET     |   Reset BlueNRG1   |   Reset BlueNRG1   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG1   |                Reset BlueNRG1                |                Reset BlueNRG1                |                Reset BlueNRG2                |                Reset BlueNRG2                |                Reset BlueNRG2                |

 @endtable

 * \section Usage Usage
 This application shows how to use the BlueNRG-1,2 HID/HOGP BLE application profile released.
 Two HID devices implemented are:

 - <b>HID Mouse</b>: 
 This demo FW implements a basic HID mouse with two buttons compliant with the standard HID/HOGP BLE application profile.
 At the first reset, the BlueNRG-1,2 starts an autocalibration of the gyroscope for a few sec.
 During the autocalibration the evaluation board shall be put on a table.
 During the autocalibraiton the LED DL1 will blink, at the end of the autocalibration procedure the LED DL1 will be OFF
 and the evaluation board is ready to use.
 The HID device is visible from the central devices with the name "STMouse".
 The mouse movement are provided by the 3D accelerometer and 3D gyroscope on the BlueNRG-1,2 evaluation board.
 The left button is the "PUSH1" button on the evaluation board.
 The right button is the "PUSH2" button on the evaluation board.
 If the mouse is not used for 2 minutes, it closes the connection and enters in deep sleep.
 This idle connection timeout can be changed from the application.
 To exit from deep sleep press the "PUSH1" button (left button) or reset the board.

 - <b> HID Keyboard</b>:
 This demo FW implements a basic HID qwerty keyboard compliant with the standard HID/HOGP BLE application profile.
 The HID device is visible from the central devices with the name "STKeyboard".
 To finish the bonding and pairing procedure with success insert the PIN: 123456
 To use the HID keyboard you need to connect the BlueNRG-1,2 evaluation board to a USB port of a PC, open an Hyperterminal using
 the configuration shows in the Serial I/O table and start to use the PC keyboard. Puts the cursor focus on the Hyperterminal window.
 On the Hyperterminal will be shown the keys that will be send to the central device using the HID/HOGP BLE application profile.
 If the keyboard is not used for 2 minutes, it closes the connection and enters in deep sleep.
 This idle connection timeout can be changed from the application.
 To exit from deep sleep press any key on the keyboard or reset the board.

 **/

/** @addtogroup BlueNRG1_demonstrations_applications
 *  BlueNRG-1 HID Peripheral demo \see main.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#ifdef __ICCARM__
#include <yfuns.h>
#endif
#include <string.h>
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "sleep.h"
#include "ble_const.h"
#include "bluenrg1_stack.h"
#include "SDK_EVAL_Com.h"
#include "hid_stack_config.h"
#include "clock.h"

/* Include the correct HID device header file */
#ifdef HID_KEYBOARD
#include "keyboard.h"
#else
#include "mouse.h"
#endif

//#define DEBUG 1
#ifdef DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern uint8_t leftButtonPressed;

int main(void) {
	uint8_t ret;

	/* System Init */
	SystemInit();

	/* Identify BlueNRG1 platform */
	SdkEvalIdentification();

	/* Application demo Led Init
	 LED1 = Sensor configuration led
	 LED3 = Connection led
	 */
	SdkEvalLedInit(LED1);
	SdkEvalLedInit(LED3);
	SdkEvalLedOn(LED1);
	SdkEvalLedOff(LED3);

	/* Serial Init 115200-8-N-1*/
	SdkEvalComIOConfig(SdkEvalComIOProcessInputData);

	/* Clock Init */
	Clock_Init();

	/* BlueNRG-1 stack init */
	ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
	if (ret != BLE_STATUS_SUCCESS) {
		PRINTF("Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
		while (1)
			;
	}

	/* Set HID Service params */
	setDefaultHidParams();
	/* Configure HID Peripheral and put the device in discoverable mode */
	ret = Configure_HidPeripheral();
	if (ret != BLE_STATUS_SUCCESS) {
		PRINTF("Error in configure_hid_peripheral() 0x%02x\n", ret);
	}

	while (1) {
		/* BLE Tick */
		BTLE_StackTick();

		/* Application Tick */
		APP_Tick();

		/* Device Power Save Management Procedures */
		DevicePowerSaveProcedure();
	}
}

/****************** BlueNRG-1 Sleep Management Callback ********************************/

SleepModes App_SleepMode_Check(SleepModes sleepMode) {
	if (SdkEvalComIOTxFifoNotEmpty() || SdkEvalComUARTBusy())
		return SLEEPMODE_RUNNING;

#ifdef HID_MOUSE
	if (leftButtonPressed)
		return SLEEPMODE_RUNNING;
#endif 

	return SLEEPMODE_NOTIMER;
}

/***************************************************************************************/

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
