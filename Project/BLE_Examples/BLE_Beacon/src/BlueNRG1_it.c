/**
 ******************************************************************************
 * @file    BlueNRG1_it.c
 * @author  VMA RF Application Team
 * @version V1.0.0
 * @date    September-2015
 * @brief   Main Interrupt Service Routines.
 *          This file provides template for all exceptions handler and
 *          peripherals interrupt service routine.
 ******************************************************************************
 * @attention
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "ble_const.h"
#include "bluenrg1_stack.h"
#include "clock.h"
#include "SDK_EVAL_Led.h"

#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "OTA_btl.h"
#endif

/** @addtogroup BlueNRG1_StdPeriph_Examples
 * @{
 */

/** @addtogroup GPIO_Examples
 * @{
 */

/** @addtogroup GPIO_IOToggle
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
volatile uint8_t receivedString[20];
volatile uint8_t receivedStringIDX = 0;
volatile uint32_t pushed_time = 0;
volatile uint32_t delta_time = 0;
#endif /* ST_USE_OTA_SERVICE_MANAGER_APPLICATION */
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief  This function handles NMI exception.
 */
void NMI_Handler(void) {
}

/**
 * @brief  This function handles Hard Fault exception.
 */
void HardFault_Handler(void) {
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1) {
	}
}

/**
 * @brief  This function handles SVCall exception.
 */
void SVC_Handler(void) {
}

/**
 * @brief  This function handles PendSV_Handler exception.
 */
//void PendSV_Handler(void)
//{
//}
/**
 * @brief  This function handles SysTick Handler.
 */
void SysTick_Handler(void) {
	SysCount_Handler();
}

void GPIO_Handler(void) {

#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
	GPIO_EXTIConfigType exti_config;
	// Interrupt from the button
	if (GPIO_GetITPendingBit(GPIO_Pin_11)) {
		GPIO_ClearITPendingBit(GPIO_Pin_11);
		GPIO_EXTICmd(GPIO_Pin_11, DISABLE);
		if (GPIO_ReadBit(GPIO_Pin_11) == Bit_RESET) {
			pushed_time = HAL_VTimerGetCurrentTime_sysT32();
			exti_config.GPIO_Pin = GPIO_Pin_11;
			exti_config.GPIO_IrqSense = GPIO_IrqSense_Level;
			exti_config.GPIO_Event = GPIO_Event_High;
			GPIO_EXTIConfig(&exti_config);
			GPIO_EXTICmd(GPIO_Pin_11, ENABLE);
			SdkEvalLedOn(LED1);
		} else {
			exti_config.GPIO_Pin = GPIO_Pin_11;
			exti_config.GPIO_IrqSense = GPIO_IrqSense_Level;
			exti_config.GPIO_Event = GPIO_Event_Low;
			GPIO_EXTIConfig(&exti_config);
			SdkEvalLedOff(LED1);
			SdkEvalLedOff(LED3);
			GPIO_EXTICmd(GPIO_Pin_11, ENABLE);
			delta_time = HAL_VTimerDiff_ms_sysT32(HAL_VTimerGetCurrentTime_sysT32(), pushed_time);
			printf("Pressed for %d\n\r", (int) delta_time);
			if (delta_time > 5000) {
				OTA_Jump_To_Service_Manager_Application();
			} else {
				// Reset
				NVIC_SystemReset();
			}
		}
	}
#endif

}
/******************************************************************************/
/*                 BlueNRG-1 Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (system_bluenrg1.c).                                               */
/******************************************************************************/
/**
 * @brief  This function handles UART interrupt request.
 * @param  None
 * @retval None
 */
void UART_Handler(void) {
#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION

	if (UART_GetITStatus(UART_IT_RX) != RESET) {
		/* Clear the interrupt */
		UART_ClearITPendingBit(UART_IT_RX);

		receivedString[receivedStringIDX] = (UART_ReceiveData() & 0xFF);
		/* Echo of received data */
		UART_SendData(receivedString[receivedStringIDX]);
		if (receivedString[receivedStringIDX] == 0x0D) {
			receivedStringIDX = 0;
			if (!strncmp("OTAServiceManager", (char *) (receivedString), 17)) {
				OTA_Jump_To_Service_Manager_Application();
			}
			memset((void*) &receivedString, 0, sizeof receivedString);
		} else {
			if (receivedStringIDX <= 18)
			receivedStringIDX++;
			else {
				receivedStringIDX = 0;
				memset((void*) &receivedString, 0, sizeof receivedString);
			}
		}

	}
#endif /* ST_USE_OTA_SERVICE_MANAGER_APPLICATION */

}

void Blue_Handler(void) {
	// Call RAL_Isr
	RAL_Isr();
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
