#ifndef __BSXFUSIONLIBRARY_H__
#define __BSXFUSIONLIBRARY_H__
/*!
* @section LICENCE
* $license$
*
* @file      BsxLiteFusionLibrary.h
* @date      2013/02/12 created
*
* @brief
* This file provides an interface to use the functionality of nine degree of freedom software library
*
* @detail
* bsxfusionlibrary - nine degree of freedom software library for the processing of the
* accelerometer, magnetometer and gyroscope sensor data. The library supports different
* operational modes of the sensors. Additionally, the library provides the virtual sensors
* like compass, imu, 9dof sensors for orientation processing, which uses the sensor fusion
* algorithm for estimation of the orientation processing or sensor calibration methods.
*
*/

/************************************************************************************************************/
/*												INCLUDES													*/
/************************************************************************************************************/

#include "BsxLibraryDataTypes.h"

/************************************************************************************************************/
/*											 GENERAL INTERFACES												*/
/************************************************************************************************************/

/*!
* @brief		Get version of the bsx Fusion Library.
* 				Version is a structure of four element which
* 				consists of major, minor, minorbugfix and majorbugfix.
* 			 	e.g. major=3, minor=0, minorbugfix =1 and
* 			  	majorbugfix = 0 implies its bsx 3.0.1.0
* @param 	* version: Pointer to version structure
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
* @Usage Guide	Call this API to get the version of
* 				the fusion library after initialization is done
*/
BSX_S8 bsx_get_version(ts_version*);

/*!
* @brief		This API initializes the main library process.
* 				If the input pointer is NULL then it will
* 				initialize with default values defined in the library.
*
* @param  		Inputparams->accelspec : Pointer to accelspec char array that holds settings for particular accel sensor
*				Inputparams->magspec : Pointer to magspec char array that holds settings for particular mag sensor
*				Inputparams->gyrospec : Pointer to gyrospec char array that holds settings for particular gyro sensor
*				Inputparams->usecaseconfig : Pointer to usecase char array that holds settings for particular usecase
*
*				Inputparams->accelspec_status holds the status if the spec is error free and not modified from original
*				0 imply error in accel spec char array
*				1 imply No error in accel spec array and spec corresponds to bma250
*				2 imply No error in accel spec array and spec corresponds to bma255
*				3 imply No error in accel spec array and spec corresponds to bma280
*
*				Inputparams->magspec_status holds the status if the spec is error free and not modified from original
*				0 imply error in mag spec char array
*				1 imply No error in mag spec array and spec corresponds to bmm050
*				2 imply No error in mag spec array and spec corresponds to bmm150
*
*				Inputparams->gyrospec_status holds the status if the spec is error free and not modified from original
*				0 imply error in gyro spec char array
*				1 imply No error in gyro spec array and spec corresponds to bmg160
*
*				Inputparams->usecase_status holds the status if the spec is error free and not modified from original
*				0 imply error in usecase char array
*				1 imply No error in usecase char array
*
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_init(initParam_t*);

/*!
* @brief    reset dynamic state of the library
* @param    none
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_reset(void);

/*!
* @brief    Main library process - divided into three process running at different frequency - dopreprocess , docalibration and dousecase
*             dopreprocess - preprocessing sensor, calibration and usecase
*             docalibration - calibration of accel,mag,gyro,fmc
*             dousecase - COMPASS/M4G , IMU , NDOF processing
* @param    libraryinput_p -> pointer to sensor data
*       	structure which includes sensor data S32 type and
*         	 time stamp in microseconds of U64 type
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_dostep(libraryinput_t* );

/*!
* @brief    Pre Process Library (includes preprocessing of accel, mag, gyro,fmc, compass)
* @param    libraryinput_p -> pointer to sensor data
*       	structure which includes sensor data S32 type and
*         	 time stamp in microseconds of U64 type
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_dopreprocess(libraryinput_t*);

/*!
* @brief    Sensor Calibration Layer - (includes calibration of accel, mag, gyro, fmc when corresponding tick.calib is enabled)
* @param    None
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_docalibration(void);


/*!
* @brief    Sensor Usecase Layer - responsible for all use-case processing (COMPASS,M4G,IMU,NDOF)
* @param    None
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_dousecase(void);

/*
* @brief 	Set the working opmode. Input is a structure of one
* 			element of BSX_U32 type and contains encoded opmode.
*			  e.g.
*        		workingModeMagOnly   =98308;
*        		workingModeAccMag    =98309;
*        		workingModeAccGyro   =524305;
*        		workingModeImuPlus   =525329;
* @param 	ts_workingModesStruct* - Pointer to working mode
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_set_workingmode(ts_workingModes *);

/*!
* @brief 	Get the sensor switch list for the current working mode.
* 			HWsensorSwitchList  is a structure of three elements
* 			BSX_U8 acc,BSX_U8 mag,BSX_U8 gyro.This function gets
* 			the status of these three alements for the
* 			given working mode.e.g. if working mode
* 			is BSX_WORKINGMODE_MAGONLY then acc = 0,mag = 1,gyro = 0;
* @param[in] workingModes Pointer to working mode constants
* @param[out] HWsensorSwitchList Pointer to hardware switch list
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
* @Usage Guide	Call this API to get sensor state based on working modes
*/
BSX_S8 bsx_get_hwdependency(ts_workingModes workingModes, ts_HWsensorSwitchList* HWsensorSwitchList);

/*!
* @brief 	calibration tick
* @params	Status:
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_get_calibrationcalltick(BSX_U8* );

/*!
* @brief 	Usecase tick
* @params	Status:
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_get_usecasecalltick(BSX_U8* );


/*!
* @brief	Set the magnetomter calibration profile(calib offset and status)
* @param	*calibprofile-> pointer to magnetometer calibration profile
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_set_magcalibprofile(ts_calibprofile*);

/*!
* @brief	Get the magnetomter calibration profile
* @param	*calibprofile-> magnetometer calibration profile
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_get_magcalibprofile(ts_calibprofile*);

/*!
* @brief	Set the gyroscope calibration profile
* @param	calibprofile-> gyroscope calibration profile
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_set_gyrocalibprofile(ts_calibprofile*);

/*!
* @brief	Get the gyroscope calibration profile
* @param	*calibprofile-> gyroscope calibration profile
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_get_gyrocalibprofile(ts_calibprofile*);

/************************************************************************************************************/
/*										DATA INTERFACES														*/
/************************************************************************************************************/

/*!
* @brief  	Get the raw accelerometer data(x,y and z direction) in m/s^2
* @param  	*rawaccdata: pointer to accelerometer raw data structure
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_get_accrawdata(ts_dataxyzf32*);

/*!
* @brief 	Get the raw 3-axis magnetometer data in microtesla
* @param  	*rawmagdata	-> mag data in microtesla
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_get_magrawdata(ts_dataxyzf32*);

/*!
* @brief  	Get the 3-axis corrected magnetometer data in MicroTesla
*			Corrected = raw – offset.
* @param  	*corMagData	-> mag data in microtesla
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_get_magcordata(ts_dataxyzf32*);

/*!
* @brief  	Get the estimated parameter of the magnetometer
* @param 	offsets -> estimated offsets in microtesla
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_get_magoffsets(ts_dataxyzf32*);

/*!
* @brief  	Get the magnetometer calibration accuracy status
* @param  	accuracyStatus 	-> current magnetometer calibration accuracy status
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_get_magcalibaccuracy(BSX_U8 *);

/*!
* @brief  	Get the raw 3-axis gryoscope data in radians/sec
* @param  	gyroData 	-> gyro data in radians/sec
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_get_gyrorawdata_rps(ts_dataxyzf32*);

/*!
* @brief  	Get the raw 3-axis gryoscope data in radians/sec
* @param  	gyroData 	-> gyro data in radians/sec
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_get_gyrocordata_rps(ts_dataxyzf32*);

/*!
* @brief  	Get the gyroscope calibration accuracy status
* @param  	*gyrocalibaccuracy 	-> current gyroscope calibration accuracy status
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_get_gyrocalibaccuracy(BSX_U8*);

/*!
* @brief  	Get the orientation quaternion data
* @param  	*quatData -> quaternion data (w,x,y,z)
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_get_orientdata_quat(ts_dataquatf32 *);

/*!
* @brief  	 Get the geo magnetic rotationvector
* @param  	*georotationquat -> Orientation quaternion data (w,x,y,z)
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_get_georotationvector_quat(ts_dataquatf32*);

/*!
* @brief  	Get the orientation euler data(heading, pitch and roll) in radians
* @param  	eulerData 	-> euler data (h,p,r)
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_get_orientdata_euler_rad(ts_dataeulerf32 *);

/*!
* @brief  	Get the heading data accuracy in radians
* @brief    Heading status refers to accuracy level of orientation data(from library) from
* @brief	true heading calculation. This comparison is done with magnetic data coupling.
* @param 	*headingaccrad -> Pointer to read the error in heading in radians
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_get_headingaccuracy_rad( BSX_F32* );

/*!
* @brief  	Get the geo magnetic rotation vector heading data accuracy in radians
* @brief    Heading status refers to accuracy level of orientation data(from library) from
* @brief	true heading calculation. This comparison is done with magnetic data coupling.
* @param 	*headingaccrad -> Pointer to read the error in heading in radians
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_get_geoheadingaccuracy_rad( BSX_F32*);

/*!
* @brief  	Get the orientation data accuracy status
* @param  	accuracyStatus 	-> orientation accuracy status
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8 bsx_get_orient_datastatus(BSX_U8*);

/*!
 * @brief   This API is used to get the step detection status.
 *          This step detection status will Update for every step get detected from algorithm.
 *          The status will retain from the last step detection based on algo calling rate.
 * @param   activity (unsigned  char)
            activity = 1 new step get detected
            activity = 0 no new step get detected
 * @return  zero for success, non-zero failed
 * @retval  0 -> stationary
 * @retval	1 -> step detected
 */
BSX_S8  bsx_get_stepdetectionstatus(BSX_U8*);

/*!
 * @brief  This API is used to get the steps count.
 *          The  steps will update for every change in valid movements of a user.
 *          This steps count will be updated till the next time the system gets reboot.
 * @param   steps(long  double) the range lies 2^(64)
 *          it will return the number of steps get detected by algorithm.
 * @return  zero for success, non-zero failed
 * @retval  0 -> Success
 * @retval	1 -> Error
 */
BSX_S8  bsx_get_stepcount(BSX_U64*);

/*!
* @brief 	This API is used to get the significant motion detection status
* 			once the motion is detected the significant motion
* 			is deactivated. algorithm can be activated by calling set
* 			opmode API.
* @param	*status(pointer to unsigned short)
*				status = 1 motion detected
*				status >=0 no motion detected
* @return   zero for success, non-zero failed
* @retval   0 -> Success
* @retval	1 -> Error
*/
BSX_S8  bsx_get_significantmotiondetectionstatus(BSX_U16*);

#endif

