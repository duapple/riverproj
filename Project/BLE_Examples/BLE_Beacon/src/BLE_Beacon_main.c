/******************** (C) COPYRIGHT 2018 STMicroelectronics ********************
 * File Name          : BLE_Beacon_main.c
 * Author             : STMicroelectronics
 * Version            : 1.1.0
 * Date               : March 22th, 2019
 * Description        : Code demonstrating the BLE Beacon application
 ********************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *******************************************************************************/

/**
 * @file BLE_Beacon_main.c
 * @brief This is a BLE beacon demo that shows how to configure a BlueNRG-1,2 device 
 * in order to advertise specific manufacturing data and allow another BLE device to
 * know if it is in the range of the BlueNRG-1 beacon device. 
 * It also provides a reference example about how using the 
 * BLE Over-The-Air (OTA) Service Manager firmware upgrade capability.
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
 <tt> ...\\Project\\BLE_Examples\\BLE_Beacon\\MDK-ARM\\BlueNRG-2\\BLE_Beacon.uvprojx </tt>
 -# Select desired configuration to build
 -# Select Project->Rebuild all target files. This will recompile and link the entire application
 -# To download the binary image, please connect STLink to JTAG connector in your board (if available).
 -# Select Project->Download to download the related binary image.
 -# Alternatively, open the BlueNRG1 Flasher utility and download the built binary image.

 * \section IAR_project IAR project
 To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
 -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu.
 -# Open the IAR project
 <tt> ...\\Project\\BLE_Examples\\BLE_Beacon\\EWARM\\BlueNRG-2\\BLE_Beacon.eww </tt>
 -# Select desired configuration to build
 -# Select Project->Rebuild All. This will recompile and link the entire application
 -# To download the binary image, please connect STLink to JTAG connector in your board (if available).
 -# Select Project->Download and Debug to download the related binary image.
 -# Alternatively, open the BlueNRG1 Flasher utility and download the built binary image.

 * \subsection Project_configurations Project configurations
 - \c Release - Release configuration
 - \c Use_OTA_ServiceManager - Configuration for Application using OTA Service Manager

 * \section Serial_IO Serial I/O
 The application jumps to the OTA service manager if "OTAServiceManager" is typed and return is pressed.

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

 |  Configuration |                      Server_Use_OTA_ServiceManager                         |
 -----------------------------------------------------------------------------------------------
 |   BUTTON name  |                                  SW1                                       |
 -----------------------------------------------------------------------------------------------
 |    Function    |    Press 5 sec to jump to OTA service Manager *when battery operated*      |

 @endtable

 The Beacon demo configures a BlueNRG-2 device in advertising mode (non-connectable mode) with specific manufacturing data.
 It transmits advertisement packets at regular intervals which contain the following manufacturing data:
 @table
 ------------------------------------------------------------------------------------------------------------------------
 | Data field              | Description                       | Notes                                                  |
 ------------------------------------------------------------------------------------------------------------------------
 | Company identifier code | SIG company identifier (1)        | Default is 0x0030 (STMicroelectronics)                 |
 | ID                      | Beacon ID                         | Fixed value                                            |
 | Length                  | Length of the remaining payload   | NA                                                     |
 | Location UUID           | Beacons UUID                      | It is used to distinguish specific beacons from others |
 | Major number            | Identifier for a group of beacons | It is used to group a related set of beacons           |
 | Minor number            | Identifier for a single beacon    | It is used to identify a single beacon                 |
 | Tx Power                | 2's complement of the Tx power    | It is used to establish how far you are from device    |
 @endtable

 - (1): SIG company identifiers are available on https://www.bluetooth.org/en-us/specification/assigned-numbers/company-identifiers
 - NA : Not Applicable;
 NOTEs:
 - OTA Service Manager support requires to build application by enabling only ST_USE_OTA_SERVICE_MANAGER_APPLICATION=1 (preprocessor, linker) options and through files: OTA_btl.
 - OTA FW upgrade feature is supported only on BlueNRG-2, BLE stack v2.x.

 **/

/** @addtogroup BlueNRG1_demonstrations_applications
 *  BlueNRG-1 Beacon demo \see BLE_Beacon_main.c for documentation.
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
#include "sleep.h"
#include "SDK_EVAL_Config.h"
#include "Beacon_config.h"

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
#define BLE_BEACON_VERSION_STRING "1.1.0"

/* Set to 1 for enabling Flags AD Type position at the beginning 
 of the advertising packet */
#define ENABLE_FLAGS_AD_TYPE_AT_BEGINNING 1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
float batteryVoltage = 0.0;
volatile bool isPlugged = false;
#endif

/* Private function prototypes -----------------------------------------------*/
void ADC_DMA_Configuration(uint8_t xMode);
void GPIO_Configuration(void);
void Interrupts_EXT_IO_Config(void);
/* Private functions ---------------------------------------------------------*/

void Device_Init(void) {
	uint8_t ret;
	uint16_t service_handle;
	uint16_t dev_name_char_handle;
	uint16_t appearance_char_handle;

	/* Set the TX Power to -2 dBm */
	ret = aci_hal_set_tx_power_level(1, 4);
	if (ret != 0) {
		printf("Error in aci_hal_set_tx_power_level() 0x%04xr\n", ret);
		while (1)
			;
	}

	/* Init the GATT */
	ret = aci_gatt_init();
	if (ret != 0)
		printf("Error in aci_gatt_init() 0x%04xr\n", ret);
	else
		printf("aci_gatt_init() --> SUCCESS\r\n");

	/* Init the GAP */
	ret = aci_gap_init(0x01, 0x00, 0x08, &service_handle, &dev_name_char_handle, &appearance_char_handle);
	if (ret != 0)
		printf("Error in aci_gap_init() 0x%04x\r\n", ret);
	else
		printf("aci_gap_init() --> SUCCESS\r\n");
}

/**
 * @brief  Start beaconing
 * @param  None
 * @retval None
 */
static void Start_Beaconing(void) {
	uint8_t ret = BLE_STATUS_SUCCESS;

#if ENABLE_FLAGS_AD_TYPE_AT_BEGINNING
	/* Set AD Type Flags at beginning on Advertising packet  */
	uint8_t adv_data[] = {
	/* Advertising data: Flags AD Type */
	0x02, 0x01, 0x06,
	/* Advertising data: manufacturer specific data */
	26, //len
			AD_TYPE_MANUFACTURER_SPECIFIC_DATA,  //manufacturer type
			0x30, 0x00, //Company identifier code (Default is 0x0030 - STMicroelectronics: To be customized for specific identifier)
			0x02,       // ID
			0x15,       //Length of the remaining payload
			0xE2, 0x0A, 0x39, 0xF4, 0x73, 0xF5, 0x4B, 0xC4, //Location UUID
			0xA1, 0x2F, 0x17, 0xD1, 0xAD, 0x07, 0xA9, 0x61, 0x00, 0x00, // Major number
			0x00, 0x00, // Minor number
			0xC8        //2's complement of the Tx power (-56dB)};
			};
#else
	uint8_t manuf_data[] = {
		26, //len
		AD_TYPE_MANUFACTURER_SPECIFIC_DATA,//manufacturer type
		0x30, 0x00,//Company identifier code (Default is 0x0030 - STMicroelectronics: To be customized for specific identifier)
		0x02,// ID
		0x15,//Length of the remaining payload
		0xE2, 0x0A, 0x39, 0xF4, 0x73, 0xF5, 0x4B, 0xC4,//Location UUID
		0xA1, 0x2F, 0x17, 0xD1, 0xAD, 0x07, 0xA9, 0x61,
		0x00, 0x00,// Major number
		0x00, 0x00,// Minor number
		0xC8//2's complement of the Tx power (-56dB)};
	};
#endif

	/* disable scan response */
	ret = hci_le_set_scan_response_data(0, NULL);
	if (ret != BLE_STATUS_SUCCESS) {
		printf("Error in hci_le_set_scan_resp_data() 0x%04x\r\n", ret);
		return;
	} else
		printf("hci_le_set_scan_resp_data() --> SUCCESS\r\n");

	/* put device in non connectable mode */
	ret = aci_gap_set_discoverable(ADV_NONCONN_IND, 160, 160, PUBLIC_ADDR, NO_WHITE_LIST_USE, 0, NULL, 0, NULL, 0, 0);
	if (ret != BLE_STATUS_SUCCESS) {
		printf("Error in aci_gap_set_discoverable() 0x%04x\r\n", ret);
		return;
	} else
		printf("aci_gap_set_discoverable() --> SUCCESS\r\n");

#if ENABLE_FLAGS_AD_TYPE_AT_BEGINNING
	/* Set the  ADV data with the Flags AD Type at beginning of the
	 advertsing packet,  followed by the beacon manufacturer specific data */
	ret = hci_le_set_advertising_data(sizeof(adv_data), adv_data);
	if (ret != BLE_STATUS_SUCCESS) {
		printf("Error in hci_le_set_advertising_data() 0x%04x\r\n", ret);
		return;
	} else
		printf("hci_le_set_advertising_data() --> SUCCESS\r\n");
#else
	/* Delete the TX power level information */
	ret = aci_gap_delete_ad_type(AD_TYPE_TX_POWER_LEVEL);
	if (ret != BLE_STATUS_SUCCESS)
	{
		printf ("Error in aci_gap_delete_ad_type() 0x%04x\r\n", ret);
		return;
	}
	else
	printf ("aci_gap_delete_ad_type() --> SUCCESS\r\n");

	/* Update the ADV data with the BEACON manufacturing data */
	ret = aci_gap_update_adv_data(27, manuf_data);
	if (ret != BLE_STATUS_SUCCESS)
	{
		printf ("Error in aci_gap_update_adv_data() 0x%04x\r\n", ret);
		return;
	}
	else
	printf ("aci_gap_update_adv_data() --> SUCCESS\r\n");
#endif
}

int main(void) {
	uint8_t ret;

	/* System Init */
	SystemInit();

	/* Identify BlueNRG-1 platform */
	SdkEvalIdentification();

	/* Init the UART peripheral */
	SdkEvalComUartInit(UART_BAUDRATE);
#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
	SdkEvalComUartIrqConfig(ENABLE);
#endif

	/* BlueNRG-1 stack init */
	ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
	if (ret != BLE_STATUS_SUCCESS) {
		printf("Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
		while (1)
			;
	}

#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
	ADC_DMA_Configuration(ADC_Input_BattSensor);
#else
	ADC_DMA_Configuration(ADC_Input_None);
#endif

	SdkEvalLedInit(LED1);
	SdkEvalLedInit(LED2);
	SdkEvalLedInit(LED3);

	/* Init the BlueNRG-1 device */
	Device_Init();

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

#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
	for (int var = 0; var <= 10; ++var) {
		ADC_Cmd(ENABLE);
		while (ADC_GetFlagStatus(ADC_FLAG_EOC) == RESET)
			;
		batteryVoltage = ADC_ConvertBatterySensor(ADC_GetRawData(), ADC_ReferenceVoltage_0V6);
	}

	if (batteryVoltage > 3.25) {
		printf("Device is powered at %dmV: ", (int)(batteryVoltage*1000));
		isPlugged = true;
		SdkEvalLedOn(LED3);
	} else {
		Interrupts_EXT_IO_Config();
	}
#endif

	/* Start Beacon Non Connectable Mode*/
	Start_Beaconing();

	printf("BlueNRG-1 BLE Beacon Application (version: %s)\r\n", BLE_BEACON_VERSION_STRING);

	while (1) {
		/* BlueNRG-1 stack tick */
		BTLE_StackTick();

		/* Enable Power Save according the Advertising Interval */

#if ST_USE_OTA_SERVICE_MANAGER_APPLICATION
		if (!isPlugged) {
			if (GPIO_ReadBit(GPIO_Pin_11) == Bit_SET)
				BlueNRG_Sleep(SLEEPMODE_NOTIMER, WAKEUP_IO11, (WAKEUP_IOx_LOW << WAKEUP_IO11_SHIFT_MASK));
			else
				BlueNRG_Sleep(SLEEPMODE_NOTIMER, WAKEUP_IO11, (WAKEUP_IOx_HIGH << WAKEUP_IO11_SHIFT_MASK));
		}
#else

		BlueNRG_Sleep(SLEEPMODE_NOTIMER, 0, 0);

#endif

	}
}

/*******************************************************************************
 * Function Name  : ADC_DMA_Configuration.
 * Description    : Init the ADC and the DMA.
 * Input          : Mode for Battery (No DMA) or Microphone (with DMA).
 * Return         : None.
 *******************************************************************************/
void ADC_DMA_Configuration(uint8_t xMode) {
	ADC_InitType ADC_InitStructure;

	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_ADC, ENABLE);
	SysCtrl_PeripheralClockCmd(CLOCK_PERIPH_DMA, ENABLE);

	ADC_DmaCmd(DISABLE);
	DMA_Cmd(DMA_CH0, DISABLE);
	ADC_Cmd(DISABLE);
	NVIC_DisableIRQ(DMA_IRQn);
	DMA_ClearFlag(DMA_FLAG_HT0);
	DMA_ClearFlag(DMA_FLAG_TC0);
	ADC_DeInit();
	DMA_DeInit(DMA_CH0);
	DMA_SelectAdcChannel(DMA_ADC_CHANNEL0, DISABLE);

	/* Enable auto offset correction */
	ADC_Calibration(ENABLE);
	ADC_AutoOffsetUpdate(ENABLE);

	switch (xMode) {
	case ADC_Input_None:
		break;
	case ADC_Input_BattSensor:
		ADC_InitStructure.ADC_OSR = ADC_OSR_200;
		ADC_InitStructure.ADC_Input = ADC_Input_BattSensor;
		ADC_InitStructure.ADC_ReferenceVoltage = ADC_ReferenceVoltage_0V6;
		ADC_InitStructure.ADC_Attenuation = ADC_Attenuation_9dB54;
		ADC_InitStructure.ADC_ConversionMode = ADC_ConversionMode_Single;
		ADC_Init(&ADC_InitStructure);

		/* Start conversion */
		ADC_Cmd(ENABLE);
		break;

	default:
		break;
	}

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

/****************** BlueNRG-1 Sleep Management Callback ********************************/

SleepModes App_SleepMode_Check(SleepModes sleepMode) {
	if (SdkEvalComIOTxFifoNotEmpty() || SdkEvalComUARTBusy())
		return SLEEPMODE_RUNNING;

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

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
