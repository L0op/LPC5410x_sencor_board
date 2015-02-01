#ifndef __BSXLIBRARYCALIBCONSTANTS_H__
#define __BSXLIBRARYCALIBCONSTANTS_H__
/*!
* @section LICENCE
* $license$
*
* @file      bsxlibrarycalibconstants.h
* @date      2013/02/12 created
*
* @brief
* This file provides constants definition for calibration modules
*
* @detail
* bsxlibrarycalibconstants - file provides constants definition used for calibration modules
*
*/

/************************************************************************************************************/
/*												INCLUDES													*/
/************************************************************************************************************/

#include "BsxLibraryDataTypes.h"

/************************************************************************************************************/
/*											CONSTANT DEFINITIONS										 	*/
/************************************************************************************************************/

/** \def calibration modes */
#define BSX_CALIB_SLEEP     			(0U)
#define BSX_CALIB_MONITORING  			(1U)
#define BSX_CALIB_MODEOFFSETACTIVE		(2U)

/** \def data correction modes */
#define BSX_DATACORRECTION_SLEEP		(0U)       /** \def 0 = sleep */
#define BSX_DATACORRECTION_OFFSET		(1U)		/** \def 1 = offset correction */
#define BSX_DATACORRECTION_SENSDOFFSET	(2U)       /** \def 2 = offset + diagonal sensitivity correction */
#define BSX_DATACORRECTION_SENSFOFFSET	(3U)       /** \def 3 = offset + full sensitivity correction */

/** \def calibration modes */
#define BSX_CALIBSOURCE_NONE     		(0U)
#define BSX_CALIBSOURCE_CLASSIC  		(1U)
#define BSX_CALIBSOURCE_FAST			(2U)

/** \def gyro calibration modes */
#define BSX_GYROCALIB_SLEEP     		(0U)      /** \def 0= Not Active */
#define BSX_GYROCALIB_GYRODATA			(1U)      /** \ def 1=Gyroscope data only */
#define BSX_GYROCALIB_GYROACC  			(2U)      /** \ def 2=Gyroscope+Accel Data */
#define BSX_GYROCALIB_GYROACCMAG		(3U)      /** \ def 3= Gyroscope+Accel+Mag Data */

#endif






































