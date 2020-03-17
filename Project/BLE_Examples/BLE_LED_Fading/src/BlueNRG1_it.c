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
#include "SDK_EVAL_Com.h"
#include "sensor.h"

#include "clock.h"

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
volatile uint32_t SystemClockTick;

extern volatile uint8_t direction;
extern volatile int16_t cycle;
extern volatile int16_t cycle_R;
extern volatile int16_t cycle_G;
extern volatile int16_t cycle_B;

extern volatile uint16_t stepSpeed_R;
extern volatile uint16_t stepSpeed_G;
extern volatile uint16_t stepSpeed_B;

axis3bit16_t data_raw_acceleration;

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
 * @brief  This function handles SysTick Handler.
 */
void SysTick_Handler(void) {
	SystemClockTick++;
	if (SystemClockTick >= 0xFFFFFFF0) {
		SystemClockTick = 0;
	}

	if ((SystemClockTick % 100) == 0) {

		memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
		lsm6dso_acceleration_raw_get(0, data_raw_acceleration.u8bit);

		if (direction == 0) {
			if (data_raw_acceleration.i16bit[0] > 0) {
				stepSpeed_R = LSM6DSO_FROM_FS_2g_TO_mg(data_raw_acceleration.i16bit[0]) / 10;
			} else {
				stepSpeed_R = -LSM6DSO_FROM_FS_2g_TO_mg(data_raw_acceleration.i16bit[0]) / 10;
			}

			if (data_raw_acceleration.i16bit[1] > 0) {
				stepSpeed_G = LSM6DSO_FROM_FS_2g_TO_mg(data_raw_acceleration.i16bit[1]) / 10;
			} else {
				stepSpeed_G = -LSM6DSO_FROM_FS_2g_TO_mg(data_raw_acceleration.i16bit[1]) / 10;
			}

			if (data_raw_acceleration.i16bit[2] > 0) {
				stepSpeed_B = LSM6DSO_FROM_FS_2g_TO_mg(data_raw_acceleration.i16bit[2]) / 10;
			} else {
				stepSpeed_B = -LSM6DSO_FROM_FS_2g_TO_mg(data_raw_acceleration.i16bit[2]) / 10;
			}
		}

		if (direction == 0) {
			cycle_R += stepSpeed_R;
			cycle_G += stepSpeed_G;
			cycle_B += stepSpeed_B;
			if ((cycle_R > 1000) || (cycle_G > 1000) || (cycle_B > 1000)) {
				cycle_R -= stepSpeed_R;
				cycle_G -= stepSpeed_G;
				cycle_B -= stepSpeed_B;

				stepSpeed_R = cycle_R / 10 + 1;
				stepSpeed_G = cycle_G / 10 + 1;
				stepSpeed_B = cycle_B / 10 + 1;

				direction = 1;
			}
		} else if (direction == 1) {
			cycle_R -= stepSpeed_R;
			cycle_G -= stepSpeed_G;
			cycle_B -= stepSpeed_B;
			if ((cycle_R < 0) && (cycle_G < 0) & (cycle_B < 0)) {
				cycle_R = 0;
				cycle_G = 0;
				cycle_B = 0;
				direction = 0;
			} else {
				if ((cycle_R < 0))
					cycle_R = 0;
				if ((cycle_G < 0))
					cycle_G = 0;
				if ((cycle_B < 0))
					cycle_B = 0;
			}
		}
	}

	if ((SystemClockTick % 400) == 0) {
		PRINTF("Red:%d%%, Green:%d%%, Blue:%d%%\n\r", cycle_R / 10, cycle_G / 10, cycle_B / 10);
	}

}

void GPIO_Handler(void) {

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

}

void Blue_Handler(void) {

}

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
