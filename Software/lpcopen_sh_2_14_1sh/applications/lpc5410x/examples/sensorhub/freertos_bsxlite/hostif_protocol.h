/*
 * @brief LPC Sensor Hub communication protocol definition
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#if !defined (__HOSTIF_PROTOCOL_H__)
#define   __HOSTIF_PROTOCOL_H__

#ifndef __KERNEL__
#include <stdint.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup SH_HOSTIF Sensor Hub: Host Interface
 * @ingroup SENSOR_HUB
 * @{
 */
#ifdef __KERNEL__
#   define LPCSH_SUSPEND_DELAY 100        /* msec suspend delay*/
#endif

/** Maximum data payload to be transferred to host */
#define HOSTIF_MAX_DATA_SIZE 128
/** Maximum custom sensor data payload	*/
#define HOSTIF_MAX_SENSOR_DATA	64	

/** Virtual Sensor IDs */
enum LPCSH_SENSOR_ID {
    LPCSH_SENSOR_ID_FIRST = 0,

    LPCSH_SENSOR_ID_ACCELEROMETER = LPCSH_SENSOR_ID_FIRST,  /*!< Calibrated Accelerometer data */
    LPCSH_SENSOR_ID_ACCELEROMETER_UNCALIBRATED,  		    		/*!< Uncalibrated Accelerometer data */
    LPCSH_SENSOR_ID_MAGNETIC_FIELD, 				        				/*!< Calibrated magnetometer data */
    LPCSH_SENSOR_ID_MAGNETIC_FIELD_UNCALIBRATED,            /*!< Uncalibrated magnetometer data */
    LPCSH_SENSOR_ID_GYROSCOPE,   				            				/*!< Calibrated gyroscope data */

    LPCSH_SENSOR_ID_GYROSCOPE_UNCALIBRATED,      						/*!< Uncalibrated gyroscope data */
    LPCSH_SENSOR_ID_LIGHT,   							    							/*!< Light data */
    LPCSH_SENSOR_ID_PRESSURE,    														/*!< Barometer pressure data */
    LPCSH_SENSOR_ID_PROXIMITY   , 						    					/*!< Proximity data */
    LPCSH_SENSOR_ID_RELATIVE_HUMIDITY,   				    				/*!< Relative humidity data */

    LPCSH_SENSOR_ID_AMBIENT_TEMPERATURE,     								/*!< Ambient temperature data */
    LPCSH_SENSOR_ID_GRAVITY,     														/*!< Gravity part of acceleration in body frame */
    LPCSH_SENSOR_ID_LINEAR_ACCELERATION,     								/*!< Dynamic acceleration */
    LPCSH_SENSOR_ID_ORIENTATION,     												/*!< yaw, pitch, roll (also use this for Win8 Inclinometer) */
    LPCSH_SENSOR_ID_ROTATION_VECTOR,     										/*!< accel+mag+gyro quaternion */
    
    LPCSH_SENSOR_ID_GEOMAGNETIC_ROTATION_VECTOR,    				/*!< accel+mag quaternion */
    LPCSH_SENSOR_ID_GAME_ROTATION_VECTOR,   			 					/*!< accel+gyro quaternion */
    LPCSH_SENSOR_ID_STEP_DETECTOR,       										/*!< Precise time a step occured */
    LPCSH_SENSOR_ID_STEP_COUNTER,    					    					/*!< Count of sensitive steps */
    LPCSH_SENSOR_ID_SIGNIFICANT_MOTION,											/*!< Significant motion detection */
    
		LPCSH_SENSOR_ID_VOICECOMMAND,														/*!< Voice Command */
    LPCSH_SENSOR_ID_ACTIVITY_CLASSIFICATION_STATE,					/*!< Activity Classification */
    LPCSH_SENSOR_ID_AUG_REALITY_COMPASS,    								/*!< heading which switches to aug-reality mode when camera towards horizon (Win8 compass) */
    LPCSH_SENSOR_ID_TAP_DETECTOR,       										/*!< precise time a TAP occured */
    LPCSH_SENSOR_ID_SHAKE_DETECTOR,       									/*!< precise time a SHAKE occured */
    LPCSH_SENSOR_ID_STEP_SEGMENT_DETECTOR,    							/*!< identifys a possible step segment: first step, last step, mid step */
	LPCSH_SENSOR_ID_COUNT
};

#define ACCEL_DATA_NAME "SensorHub Accelerometer"
#define UNCALIBRATED_ACCEL_DATA_NAME "SensorHub Uncalibrated Accelerometer"
#define MAGNETIC_DATA_NAME "SensorHub Magnetic Sensor"
#define UNCAL_MAGNETIC_DATA_NAME "SensorHub Uncalibrated Magnetic Sensor"
#define ORIENTATION_DATA_NAME "SensorHub Orientation Sensor"
#define GYROSCOPE_DATA_NAME "SensorHub Gyroscope"
#define UNCAL_GYROSCOPE_DATA_NAME "SensorHub Uncalibrated Gyroscope"
#define ROTATIONVECTOR_DATA_NAME "SensorHub Rotation Vector"
#define GAME_ROTATIONVECTOR_DATA_NAME "SensorHub Game Rotation Vector"
#define GEO_MAGNETIC_ROTATION_VECTOR_DATA_NAME "SensorHub Geo Magnetic Rotation Vector"
#define LIGHT_DATA_NAME "SensorHub Light Sensor"
#define PROXIMITY_DATA_NAME "SensorHub Proximity Sensor"
#define PRESSURE_DATA_NAME "SensorHub Pressure Sensor"
#define STEP_DETECTOR_NAME "SensorHub Step Detector"
#define STEP_COUNTER_NAME "SensorHub Step Counter"
#define LINEAR_ACCELERATION_DATA_NAME "SensorHub Linear Acceleration Sensor"
#define GRAVITY_DATA_NAME "SensorHub Gravity"
#define SIGNIFICANT_MOTION_DATA_NAME "SensorHub Significant Motion Detector"
#define ACTIVITY_CLASSIFICATION_STATE_DATA_NAME "SensorHub Activity Classification State Detector"
#define TEMPERATURE_DATA_NAME "SensorHub Temperature Sensor"
#define RELATIVE_HUMIDITY_DATA_NAME "SensorHub Relative Humidity Sensor"
#define TAP_DETECTOR_DATA_NAME "SensorHub TAP Detector"
#define SHAKE_DETECTOR_DATA_NAME "SensorHub SHAKE Detector"

#define SENSOR_MANUFACTURER "NXP Semiconductor"

#define packed_struct  struct
#define packed_union  union
#pragma pack(push)
#pragma pack(1)

/**
 * @brief Generic sensor structure for Accel, Mag, Gyro etc
 */
packed_struct lpcsh_motion_sensor_node {
	int16_t Data[3];	/*!< Sensor data for X, Y, Z axis */
};

/**
 * @brief Generic uncalibrated sensor structure with Bias
 */
packed_struct lpcsh_motion_uncal_sensor_node {
	int16_t Data[3];	/*!< Sensor data for X, Y, Z axis */
	int16_t Bias[3];	/*!< Bias data for X, Y, Z axis */
};

/**
 * @brief Proximity data structure
 */
packed_struct lpcsh_proximity_sensor_node {
	int16_t Data;	/*!< Proximity distance data */
};

/**
 * @brief Ambient Light data structure
 */
packed_struct lpcsh_light_sensor_node {
	int16_t Data;	/*!< Light intensity data */
};

/**
 * @brief Temperature data structure
 */
packed_struct lpcsh_temperature_sensor_node {
	int16_t Data;	/*!< Temperature */
};

/**
 * @brief Pressure data structure
 */
packed_struct lpcsh_pressure_sensor_node {
	int16_t Data;	/*!< Pressure */
};

/**
 * @brief Humidity data structure
 */
packed_struct lpcsh_humidity_sensor_node {
	int16_t Data;	/*!< Humidity */
};

/**
 * @brief Significant motion data structure
 */
packed_struct lpcsh_significant_motion_node {
	unsigned char  significantMotionDetected;	/*!< Boolean to indicate if significant motion happened */
};

/**
 * @brief Activity Classification data structure
 */
packed_struct lpcsh_activity_classification_state_node {
	uint8_t Data;	/*!< Activity data */
};

/**
 * @brief Orientation structure with pitch, roll and yaw
 */
packed_struct lpcsh_orientation_node {
	int32_t Data[3];  /*!< Orientation vector data (Pitch, Roll and Yaw) */
};

/**
 * @brief Rotation Vector structure
 */
packed_struct lpcsh_quaternion_node {
    int32_t W;	/*!< Rotation Vector data for W axis */
    int32_t X;  /*!< Rotation Vector data for X axis */
    int32_t Y;	/*!< Rotation Vector data for Y axis */
    int32_t Z;	/*!< Rotation Vector data for Z axis */
    int32_t E_EST;
} ;

/**
 * @brief Virtual Sensor Data Header
 */
packed_struct lpcsh_sensor_node_header {
	uint8_t sensorId;	/*!< enum LPCSH_SENSOR_ID */
	uint32_t timeStamp; /*!< raw time stamp */
};

/**
 * @brief Step Counter data structure
 */
packed_struct lpcsh_step_sensor_node {
	uint32_t numTotalsteps;	/*!< Total number of steps counted */
};

/**
 * @brief Tap Detector data structure
 */
packed_struct lpcsh_tap_sensor_node {
	uint8_t state;	/*!< Tap state */
};

/**
 * @brief Shake detector data structure
 */
packed_struct lpcsh_shake_sensor_node {
	uint8_t detected;	/*!< Shake state */
};

/**
 * @brief Virtual Sensor Output Data for Host Interface
 */
 packed_struct lpcsh_sensor_node {
	packed_struct lpcsh_sensor_node_header header;	/*!< Header for Sensor Data */
	packed_union {
		packed_struct lpcsh_motion_sensor_node          sensorData;	/*!< Generic sensor structure for Accel, Mag, Gyro etc */
		packed_struct lpcsh_motion_uncal_sensor_node    uncal_sensorData;	/*!< Generic uncalibrated sensor structure with Bias */
        packed_struct lpcsh_quaternion_node             quaternionData;	/*!< Rotation Vector structure */
        packed_struct lpcsh_orientation_node            orientationData;	/*!< Orientation structure with pitch, roll and yaw */
        packed_struct lpcsh_step_sensor_node            stepData;	/*!< Step Counter data structure */
		packed_struct lpcsh_significant_motion_node     significantMotionData;	/*!< Significant motion data structure */
		packed_struct lpcsh_light_sensor_node           lightData;	/*!< Ambient Light data structure */
		packed_struct lpcsh_pressure_sensor_node        pressureData;	/*!< Pressure data structure */
		packed_struct lpcsh_proximity_sensor_node       proximityData;	/*!< Proximity data structure */
		packed_struct lpcsh_humidity_sensor_node        humidityData;	/*!< Humidity data structure */
        packed_struct lpcsh_temperature_sensor_node     temperature_sensor_node;	/*!< Temperature data structure */
        packed_struct lpcsh_activity_classification_state_node activityClassificationStateData;	/*!< Activity Classification data structure */
		packed_struct lpcsh_tap_sensor_node				tapDetectorData;	/*!< Tap Detector data structure */
		packed_struct lpcsh_shake_sensor_node 			shakeDetectorData;	/*!< Shake detector data structure */
		uint8_t											custom[HOSTIF_MAX_SENSOR_DATA];		/*!< Custom data to accomodate fusion library specific sensors */
	} data;
};


packed_struct ShCmdGetHeader_get_8bits_param_t {
    uint8_t param;
} ;

packed_struct ShCmdGetHeader_get_16bits_param_t {
    uint16_t param;
} ;

#define LPCSH_MAX_CMD_LENGTH            16	/*!< Maximum command packet size */
#define LPCSH_MAX_SENSOR_ID				30
	
/** Host I/F command ID enums */
enum LPCSH_CMD_ID_T {
	LPCSH_CMD_WHO_AM_I = 0x00,			/*!< Provide 8 bits Device ID */
	LPCSH_CMD_GET_VERSION,				/*!< 2 byte response. Byte 0 major version, byte 1 minor version */
	LPCSH_CMD_RESET,					/*!< Resets host interface */
	LPCSH_CMD_GET_DATA_LENGTH,			/*!< Command used by I2C host interface only to get notification data length */
	LPCSH_CMD_GET_DATA,					/*!< Command used by I2C host interface only to get notification data. */

	LPCSH_CMD_SENSOR_ENABLE = 0x20,		/*!< Command to enable virtual sensors. */
	LPCSH_CMD_GET_SENSOR_STATE,			/*!< Command to get virtual sensors state. */
	LPCSH_CMD_SET_DELAY,				/*!< Command to set sensor sample rate. */
	LPCSH_CMD_GET_DELAY,				/*!< Command to get sensor sample rate. */
};	

/**
 * @brief Host Interface Command Structure
 */
packed_struct LPCSH_CMD_t {
	uint8_t id;	/*!< Command Id from enum LPCSH_CMD_ID_T */
	uint8_t params[LPCSH_MAX_CMD_LENGTH - 1];	/*!< Command Parameters */
};

packed_struct ShHubCmdHeader_t {
    uint8_t command;	/* enum LPCSH_CMD_ID_T */
};

packed_struct ShHubCmdHeader_8bits_param_t {
    uint8_t command;	/* enum LPCSH_CMD_ID_T */
    uint8_t param;
} ;



packed_struct ShSensorCmdHeader_t {
    uint8_t command;	/* enum LPCSH_CMD_ID_T */
    uint8_t sensorId;	/* enum LPCSH_SENSOR_ID */
} ;

packed_struct ShSensorSetCmdHeader_8bits_param_t {
    uint8_t command;	/* enum LPCSH_CMD_ID_T */
    uint8_t sensorId;	/* enum LPCSH_SENSOR_ID */
    uint8_t param;
} ;

packed_struct ShSensorSetCmdHeader_16bits_param_t {
    uint8_t command;	/* enum LPCSH_CMD_ID_T */
    uint8_t sensorId;	/* enum LPCSH_SENSOR_ID */
    uint16_t param;
} ;


packed_struct ShSensorSetCmdHeader_32bits_param_t {
    uint8_t command;	/* enum LPCSH_CMD_ID_T */
    uint8_t sensorId;	/* enum LPCSH_SENSOR_ID */
    uint32_t param;
} ;


packed_union ShCmdHeaderUnion{
    struct ShSensorCmdHeader_t command;
    struct ShSensorSetCmdHeader_8bits_param_t sensor_cmd_8bits_param;
    struct ShSensorSetCmdHeader_16bits_param_t sensor_cmd_16bits_param;
    struct ShSensorSetCmdHeader_32bits_param_t sensor_cmd_32bits_param;
    struct ShHubCmdHeader_t hubCmdHeader;
    struct ShHubCmdHeader_8bits_param_t hub_cmd_8bits_param;
} ;

#pragma pack(pop)

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __HOSTIF_PROTOCOL_H__ */
