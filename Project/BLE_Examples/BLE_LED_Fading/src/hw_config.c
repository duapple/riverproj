/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
 * File Name          : hw_config.c
 * Author             : IoT-EC Application team
 * Version            : V1.0.0
 * Date               : June 26, 2018
 * Description        : STEVAL-BCN002V1 Hardware configuration file
 ********************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *******************************************************************************/

#include "hw_config.h"
#include "SDK_EVAL_Config.h"
#include "sensor.h"
#include "clock.h"

#if ENABLE_DEBUG
	#include <stdio.h>
	#define PRINTF(...) printf(__VA_ARGS__)
#else
	#define PRINTF(...)
#endif

/* Private variables ---------------------------------------------------------*/
/* Imported Variables -------------------------------------------------------------*/

void Platform_Init(void) {

	/* Configure I/O communication channel */
	SdkEvalComUartInit(UART_BAUDRATE);

	// Init the systick
	Clock_Init();

	/* GPIO Configuration */
	GPIO_Configuration();

}

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
	/** Configure GPIO_Pin_7 for Proximity XSHUT */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Output;
	GPIO_InitStructure.GPIO_Pull = DISABLE;
	GPIO_InitStructure.GPIO_HighPwr = ENABLE;
	GPIO_Init(&GPIO_InitStructure);

	GPIO_WriteBit(GPIO_Pin_7, Bit_RESET);

}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
