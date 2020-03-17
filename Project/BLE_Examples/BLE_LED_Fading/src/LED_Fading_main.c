/******************** (C) COPYRIGHT 2019 STMicroelectronics ********************
 * File Name          : LED_Fading_main.c
 * Author             : STMicroelectronics
 * Version            : 1.1.0
 * Date               : March 27th, 2019
 * Description        : Code demonstrating a fading of RGB led with different color
 * 						based on the board orientation.
 ********************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *******************************************************************************/

/**
 * @file LED_Fading_main.c
 * @brief This application contains an example which shows how implementing a software dimming
 * of the RGB led
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
 <tt> ...\\Project\\BLE_Examples\\BLE_LED_Fading\\MDK-ARM\\BlueNRG-2\\BLE_LED_Fading.uvprojx </tt>
 -# Select desired configuration to build
 -# Select Project->Rebuild all target files. This will recompile and link the entire application
 -# To download the binary image, please connect STLink to JTAG connector in your board (if available).
 -# Select Project->Download to download the related binary image.
 -# Alternatively, open the BlueNRG1 Flasher utility and download the built binary image.

 * \section IAR_project IAR project
 To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
 -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu.
 -# Open the IAR project
 <tt> ...\\Project\\BLE_Examples\\BLE_LED_Fading\\EWARM\\BlueNRG-2\\BLE_LED_Fading.eww </tt>
 -# Select desired configuration to build
 -# Select Project->Rebuild All. This will recompile and link the entire application
 -# To download the binary image, please connect STLink to JTAG connector in your board (if available).
 -# Select Project->Download and Debug to download the related binary image.
 -# Alternatively, open the BlueNRG1 Flasher utility and download the built binary image.

 * \subsection Project_configurations Project configurations
 - \c Release - Release configuration

 * \section Board_supported Boards supported
 - \c STEVAL-BCN002V1

 @table
 | Parameter name  | Value               | Unit      |
 -----------------------------------------------------
 | Baudrate        | 115200 [default]    | bit/sec   |
 | Data bits       | 8                   | bit       |
 | Parity          | None                | bit       |
 | Stop bits       | 1                   | bit       |
 @endtable

 **/

/** @addtogroup BlueNRG1_demonstrations_applications
 * BlueNRG-2 LED_Fading \see LED_Fading_main.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */
/* Includes ------------------------------------------------------------------*/
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "SDK_EVAL_Config.h"
#include "sleep.h"
#include "sensor.h"
#include "hw_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#if ENABLE_DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint8_t direction = 0;
volatile int16_t cycle = 0;
volatile int16_t cycle_R = 0;
volatile int16_t cycle_G = 0;
volatile int16_t cycle_B = 0;

volatile uint16_t stepSpeed_R = 0;
volatile uint16_t stepSpeed_G = 0;
volatile uint16_t stepSpeed_B = 0;

/* Private function prototypes -----------------------------------------------*/
void LED_AppTick(void);
/* Private functions ---------------------------------------------------------*/

int main(void) {

	/* System Init */
	SystemInit();

	/* Identify BlueNRG1 platform */
	SdkEvalIdentification();

	/* Init the Hardware platform */
	Platform_Init();

	/* Init Sensors */
	Sensor_DeviceInit();

	while (1) {

		/* LED Soft PWM Tick*/
		LED_AppTick();

	}/* while (1) */
}

/****************** BlueNRG-1 Sleep Management Callback ********************************/

SleepModes App_SleepMode_Check(SleepModes sleepMode) {
	if (SdkEvalComIOTxFifoNotEmpty())
		return SLEEPMODE_RUNNING;
	return sleepMode;
}

void LED_AppTick(void) {

	cycle++;
	if (cycle > 1000)
		cycle = 0;

	if (cycle_R > cycle)
		SdkEvalLedOn(LED1);
	else
		SdkEvalLedOff(LED1);
	if (cycle_G > cycle)
		SdkEvalLedOn(LED2);
	else
		SdkEvalLedOff(LED2);
	if (cycle_B > cycle)
		SdkEvalLedOn(LED3);
	else
		SdkEvalLedOff(LED3);

}

/***************************************************************************************/

#ifdef USE_FULL_ASSERT
/*******************************************************************************
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line number
 *                  where the assert_param error has occurred.
 * Input          : - file: pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 *******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{}
}
#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
