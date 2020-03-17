/******************** (C) COPYRIGHT 2015 STMicroelectronics ********************
 * File Name          : sensor.h
 * Author             : IoT-EC Application team
 * Version            : V1.0.0
 * Date               : June 26, 2018
 * Description        : STEVAL-BCN002V1 Sensor init and state machines header file
 ********************************************************************************
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *******************************************************************************/

#include <stdbool.h>
#include "LPS22HH.h"				// Pressure
#include "LPS22HH_hal.h"
#include "LSM6DSO.h"				// Accelerometer and Gyroscope
#include "LSM6DSO_hal.h"
#include "HTS221.h"					// Humidity
#include "HTS221_hal.h"
#include "LIS2MDL.h"				// Magnetometer
#include "LIS2MDL_hal.h"

#ifndef _SENSOR_H_
#define _SENSOR_H_

#include "vl53l1_platform.h"	// Proximity
#include "VL53L1X_api.h"

#if ENABLE_OTA
#define ENABLE_BLUEVOICE 1
#endif

enum {
	PROXY_OFF = 0, PROXY_RANGING = 1, PROXY_AVAILABLE = 2, PROXY_ON = 3, PROXY_SILENT = 4
};

void Sensor_DeviceInit(void);
void APP_Tick(void);

extern uint8_t Application_Max_Attribute_Records[];

#define FROM_MGAUSS_TO_UT50 (0.1f/50.0f)
#define FROM_UT50_TO_MGAUSS 500.0f

/* Exported macro ------------------------------------------------------------*/
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};

extern uint8_t BufferToWrite[512];
extern int32_t BytesToWrite;

/* BLE Characteristic connection control */
/* Environmental Data */
#define W2ST_CONNECT_ENV           (1   )
/* LED status */
#define W2ST_CONNECT_LED           (1<<1)
/* Acceleration/Gyroscope/Magneto */
#define W2ST_CONNECT_ACC_GYRO_MAG  (1<<2)
/* Mic */
#define W2ST_CONNECT_AUDIO_LEVEL   (1<<3)
/* Standard Terminal */
#define W2ST_CONNECT_STD_TERM      (1<<4)
/* Standard Error */
#define W2ST_CONNECT_STD_ERR       (1<<5)
/* HW Advance Features */
#define W2ST_CONNECT_ACC_EVENT     (1<<6)
/* Gas Gouge Feature */
#define W2ST_CONNECT_GG_EVENT      (1<<7)

typedef struct {
	bool AccelerometerGyroscopePresence;
	bool MagnetometerPresence;
	bool HumidityTemperaturePresence;
	bool PressurePresence;
	bool ProximityLightPresence;
	bool iNemoEngine;
	bool Pedometer;
} FeaturePresence;

typedef struct {
	bool HwFreeFall;
	bool HwPedometer;
	bool HwDoubleTAP;
	bool HwSingleTAP;
	bool HwWakeUp;
	bool HwTilt;
	bool HwOrientation6D;
	bool MultipleEvent;
} HardwareFeaturePresence;

typedef struct {
	bool MotionNotification;
	bool EnvironmentalNotification;
	bool ProximityNotification;
	bool iNemoEngineNotification;
	bool BatteryMonitorNotification;
	bool LedNotification;
	bool eCompassNotification;
	bool AudioNotification;
	bool SyncNotification;
	bool AccEventNotification;
	bool Advertising;
} FeatureNotification;

typedef struct {
	int32_t AXIS_X;
	int32_t AXIS_Y;
	int32_t AXIS_Z;
} QuaternionAxes_t;

/* Feature mask for LED */
#define FEATURE_MASK_LED 0x20000000
/* Feature mask for Sensor fusion short precision */
#define FEATURE_MASK_SENSORFUSION_SHORT 0x00000100
/* Feature mask for e-compass */
#define FEATURE_MASK_ECOMPASS 0x00000040
/* Feature mask for hardware events */
#define FEATURE_MASK_ACC_EVENTS 0x00000400
/* Feature mask for Temperature1 */
#define FEATURE_MASK_TEMP1 0x00040000
/* Feature mask for Temperature2 */
#define FEATURE_MASK_TEMP2 0x00010000
/* Feature mask for Pressure */
#define FEATURE_MASK_PRESS 0x00100000
/* Feature mask for Humidity */
#define FEATURE_MASK_HUM   0x00080000
/* Feature mask for Accelerometer */
#define FEATURE_MASK_ACC   0x00800000
/* Feature mask for Gyroscope */
#define FEATURE_MASK_GRYO  0x00400000
/* Feature mask for Magnetometer */
#define FEATURE_MASK_MAG   0x00200000
/* Feature mask for Microphone */
#define FEATURE_MASK_MIC   0x04000000
/* Feature mask for BlueVoice */
#define FEATURE_MASK_BLUEVOICE   0x08000000

/* W2ST command for asking the calibration status */
#define W2ST_COMMAND_CAL_STATUS 0xFF
/* W2ST command for resetting the calibration */
#define W2ST_COMMAND_CAL_RESET  0x00
/* W2ST command for stopping the calibration process */
#define W2ST_COMMAND_CAL_STOP   0x01

#define W2ST_CHECK_CONNECTION(BleChar) ((ConnectionBleStatus&(BleChar)) ? 1 : 0)
#define W2ST_ON_CONNECTION(BleChar)    (ConnectionBleStatus|=(BleChar))
#define W2ST_OFF_CONNECTION(BleChar)   (ConnectionBleStatus&=(~BleChar))

#define SENSOR_TIMER 		0		// Fixed ODR @ 25 Hz
#define ADVERTISING_TIMER 	1		// Fixed @ 20 seconds
#define RESET_TIMER			2		// Fixed 5 seconds

#define ADVERTISING_TIME	20000	// Time to advertise before enter standby [ms]
#define RESET_TIME 			5000	// Time to wait before reset [ms]

#define BATTERY_UPDATE_RATE			1000    // Fixed ODR @  1 Hz
#define ENV_SENSOR_UPDATE_RATE		100     // Fixed ODR @  10 Hz
#define MOTION_SENSOR_UPDATE_RATE 	40		// Fixed ODR @ 25 Hz

void Set_DeviceConnectable(void);

void Init_SensorFusion(bool onOff);
void Init_BlueNRG_Custom_Services(void);
void Init_Accelerometer_Gyroscope(void);
void Init_Pressure_Temperature_Sensor(void);
void Init_Humidity_Sensor(void);
void Init_Magnetometer(void);
void SensorsScan(void);
void SensorsLowPower(void);
uint32_t ConfigCommandParsing(uint8_t * att_data, uint8_t data_length);
uint32_t DebugConsoleCommandParsing(uint8_t * att_data, uint8_t data_length);
void MEMSCallback(void);


#endif /* _SENSOR_H_ */
