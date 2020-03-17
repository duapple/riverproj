/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
 * File Name          : sensor.c
 * Author             : IoT-EC Application team
 * Version            : V1.0.0
 * Date               : June 26, 2018
 * Description        : STEVAL-BCN002V1 Sensor init and state machines
 ********************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "ble_const.h" 
#include "bluenrg1_stack.h"
#include "gp_timer.h"
#include "SDK_EVAL_Config.h"

#include "sensor.h"
#include "hw_config.h"
#include "sleep.h"
#include "LED_Fading_main.h"

#if ENABLE_DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern uint16_t cycle;
extern uint16_t cycle_R;
extern uint16_t cycle_G;
extern uint16_t cycle_B;

volatile FeaturePresence xFeaturePresence;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
 * Function Name  : Init_Accelerometer_Gyroscope.
 * Description    : Init LSM6DSO accelerometer/gyroscope.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Init_Accelerometer_Gyroscope(void) {

	uint8_t rst;

	lsm6dso_i3c_disable_set(0, LSM6DSO_I3C_DISABLE);

	rst = lsm6dso_reset_set(0, PROPERTY_ENABLE);
	do {
		lsm6dso_reset_get(0, &rst);
	} while (rst);

	lsm6dso_pin_mode_set(0, LSM6DSO_PUSH_PULL);
	lsm6dso_pin_polarity_set(0, LSM6DSO_ACTIVE_LOW);
	lsm6dso_all_on_int1_set(0, PROPERTY_ENABLE);
	lsm6dso_int_notification_set(0, LSM6DSO_ALL_INT_LATCHED);

	lsm6dso_block_data_update_set(0, PROPERTY_ENABLE);
	lsm6dso_xl_power_mode_set(0, LSM6DSO_LOW_NORMAL_POWER_MD);
	lsm6dso_gy_power_mode_set(0, LSM6DSO_GY_NORMAL);
	lsm6dso_xl_data_rate_set(0, LSM6DSO_XL_ODR_52Hz);
	lsm6dso_gy_data_rate_set(0, LSM6DSO_GY_ODR_52Hz);
	lsm6dso_xl_full_scale_set(0, LSM6DSO_2g);
	lsm6dso_gy_full_scale_set(0, LSM6DSO_2000dps);

	lsm6dso_auto_increment_set(0, PROPERTY_ENABLE);

}

/*******************************************************************************
 * Function Name  : Init_Magnetometer.
 * Description    : Init LIS2MDL magnetometer.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Init_Magnetometer(void) {

	lis2mdl_data_rate_set(0, LIS2MDL_ODR_50Hz);
	lis2mdl_block_data_update_set(0, PROPERTY_ENABLE);
	lis2mdl_set_rst_mode_set(0, LIS2MDL_SET_SENS_ODR_DIV_63);
	lis2mdl_operating_mode_set(0, LIS2MDL_CONTINUOUS_MODE);
	lis2mdl_offset_temp_comp_set(0, PROPERTY_ENABLE);

}

/*******************************************************************************
 * Function Name  : Init_Pressure_Temperature_Sensor.
 * Description    : Init LPS22HB pressure and temperature sensor.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Init_Pressure_Temperature_Sensor(void) {

	lps22hh_i3c_interface_set(0, LPS22HH_I3C_DISABLE);
	lps22hh_block_data_update_set(0, PROPERTY_ENABLE);
	lps22hh_data_rate_set(0, LPS22HH_1_Hz);
	lps22hh_pin_mode_set(0, LPS22HH_OPEN_DRAIN);
	lps22hh_pin_polarity_set(0, LPS22HH_ACTIVE_LOW);
	lps22hh_int_notification_set(0, LPS22HH_INT_PULSED);
	lps22hh_int_pd_set(0, LPS22HH_PULL_DOWN_DISCONNECT);

}

/*******************************************************************************
 * Function Name  : Init_Humidity_Sensor.
 * Description    : Init HTS221 temperature sensor.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Init_Humidity_Sensor(void) {

	HTS221_Set_AvgH(0, HTS221_AVGH_4);
	HTS221_Set_AvgT(0, HTS221_AVGT_2);
	HTS221_Set_BduMode(0, HTS221_ENABLE);
	HTS221_Set_Odr(0, HTS221_ODR_1HZ);
	HTS221_Set_HeaterState(0, HTS221_DISABLE);

	HTS221_Set_IrqOutputType(0, HTS221_OPENDRAIN);
	HTS221_Set_IrqActiveLevel(0, HTS221_LOW_LVL);

	HTS221_Set_PowerDownMode(0, HTS221_RESET);
	HTS221_Activate(0);

}

/*******************************************************************************
 * Function Name  : Sensor_DeviceInit.
 * Description    : Init the device sensors.
 * Input          : None.
 * Return         : None
 *******************************************************************************/
void Sensor_DeviceInit() {

	/* Configure LED RGB and turn it off */
	SdkEvalLedInit(LED1); // Red LED
	SdkEvalLedInit(LED2); // Green LED
	SdkEvalLedInit(LED3); // Blue LED

	/* Configure I2C @ 400 kHz */
	SdkEvalI2CInit(400000);

	// Check sensor list
	SensorsScan();

	// Configure discovered sensors
	if (xFeaturePresence.PressurePresence)
		Init_Pressure_Temperature_Sensor();
	if (xFeaturePresence.HumidityTemperaturePresence)
		Init_Humidity_Sensor();
	if (xFeaturePresence.MagnetometerPresence)
		Init_Magnetometer();
	if (xFeaturePresence.AccelerometerGyroscopePresence)
		Init_Accelerometer_Gyroscope();

}

/*******************************************************************************
 * Function Name  : SensorsScan
 * Description    : Configure the sensors in active or low-power mode.
 * Input          : None
 * Return         : None
 *******************************************************************************/
void SensorsScan(void) {
	uint8_t who_am_I_8 = 0x00;
	uint16_t ModelID;

	PRINTF("Scan for sensors:\n\r");

	lsm6dso_device_id_get(0, &who_am_I_8);
	if (who_am_I_8 == LSM6DSO_ID) {
		PRINTF("- Accelerometer and Gyroscope: OK\n\r");
		xFeaturePresence.AccelerometerGyroscopePresence = true;
		xFeaturePresence.Pedometer = true;

	} else {
		PRINTF("- Accelerometer and Gyroscope: FAIL\n\r");
		xFeaturePresence.AccelerometerGyroscopePresence = false;
		xFeaturePresence.Pedometer = false;
	}

	lps22hh_device_id_get(0, &who_am_I_8);
	if (who_am_I_8 == LPS22HH_ID) {
		PRINTF("- Pressure and Temperature: OK\n\r");
		xFeaturePresence.PressurePresence = true;
	} else {
		PRINTF("- Pressure and Temperature: FAIL\n\r");
		xFeaturePresence.PressurePresence = false;
	}

	HTS221_Get_DeviceID(0, &who_am_I_8);
	if (who_am_I_8 == 0xBC) {
		PRINTF("- Humidity and Temperature: OK\n\r");
		xFeaturePresence.HumidityTemperaturePresence = true;
	} else {
		PRINTF("- Humidity and Temperature: FAIL\n\r");
		xFeaturePresence.HumidityTemperaturePresence = false;
	}

	lis2mdl_device_id_get(0, &who_am_I_8);
	if (who_am_I_8 == LIS2MDL_ID) {
		xFeaturePresence.MagnetometerPresence = true;
		PRINTF("- Magnetometer: OK\n\r- Proximity Sensor: ");
	} else {
		PRINTF("- Magnetometer: FAIL\n\r- Proximity Sensor: ");
		xFeaturePresence.MagnetometerPresence = false;
	}
	GPIO_WriteBit(GPIO_Pin_7, Bit_RESET);
	VL53L1_WaitMs(VL53L1_I2C_SLAVE_ADDR, 10);
	GPIO_WriteBit(GPIO_Pin_7, Bit_SET);

	for (uint8_t var = 0; var < 100; ++var) {
		VL53L1_WaitMs(VL53L1_I2C_SLAVE_ADDR, 10);

		VL53L1X_GetSensorId(VL53L1_I2C_SLAVE_ADDR, &ModelID);
		if (ModelID == 0xEACC)
			break;
	}

	GPIO_WriteBit(GPIO_Pin_7, Bit_RESET);
	if (ModelID == 0xEACC) {
		PRINTF("OK\n\r");
		xFeaturePresence.ProximityLightPresence = true;
	} else {
		PRINTF("FAIL\n\r");
		xFeaturePresence.ProximityLightPresence = false;
	}

}

/*******************************************************************************
 * Function Name  : SensorsConfiguration
 * Description    : Configure the sensors low-power mode.
 * Input          : None
 * Return         : None
 *******************************************************************************/
void SensorsLowPower(void) {

	if (xFeaturePresence.AccelerometerGyroscopePresence) {
		lsm6dso_xl_data_rate_set(0, LSM6DSO_XL_ODR_OFF);
		lsm6dso_gy_data_rate_set(0, LSM6DSO_GY_ODR_OFF);
	}
	if (xFeaturePresence.PressurePresence) {
		lps22hh_data_rate_set(0, LPS22HH_POWER_DOWN);
	}
	if (xFeaturePresence.MagnetometerPresence) {
		lis2mdl_operating_mode_set(0, LIS2MDL_POWER_DOWN);
	}
	if (xFeaturePresence.HumidityTemperaturePresence) {
		HTS221_Set_PowerDownMode(0, HTS221_SET);
	}
	if (xFeaturePresence.ProximityLightPresence) {
		/* Turn OFF Proximity */
		GPIO_WriteBit(GPIO_Pin_7, Bit_RESET);
	}
	PRINTF("Sensor in low-power: OK\n\r");

}
