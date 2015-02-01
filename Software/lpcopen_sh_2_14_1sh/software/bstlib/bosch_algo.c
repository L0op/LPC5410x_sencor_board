/*
 * @brief Implements BOSCH algorithm glue logic
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
#include "board.h"
#include <string.h>
#include "sensorhub.h"
#include "BsxLibraryCalibConstants.h"
#include "BsxLibraryConstants.h"
#include "BsxLibraryDataTypes.h"
#include "BsxLibraryErrorConstants.h"
#if defined(BSX_LITE)
#include "BsxLiteFusionLibrary.h"
#else
#include "BsxFusionLibrary.h"
#endif

#define INCLUDE_BMA255API
#define INCLUDE_BMM050API
#define INCLUDE_USECASE_NDOF


/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/
uint8_t ACC_Spec[37] =
#if defined(BSX_LITE)
#if defined(INCLUDE_BMA255API)
{37,0,2,1,0,9,12,150,0,12,180,0,5,0,1,0,176,4,82,3,0,0,64,65,1,1,1,1,2,2,2,3,3,1,1,63,231}; /* BMA255 */
#elif defined(INCLUDE_BMI160API)
{37,0,3,1,0,9,12,150,0,16,60,0,1,0,1,0,176,4,82,3,0,0,64,65,1,1,1,1,2,2,2,3,3,1,1,180,115}; /* BMI160 */
#endif
#else
#if defined(INCLUDE_BMA250API)
{37,0,1,1,0,7,9,150,0,10,180,0,5,0,1,0,176,4,32,3,0,0,64,65,1,1,1,2,2,2,3,4,4,1,1,25,213}; /* BMA250 */
#elif defined(INCLUDE_BMA255API)
{37,0,2,1,0,7,9,150,0,12,180,0,5,0,1,0,176,4,82,3,0,0,64,65,1,1,1,1,2,2,2,3,3,1,1,172,167}; /* BMA255 */
#elif defined(INCLUDE_BMA280API)
{37,0,3,1,0,7,9,150,0,14,180,0,5,0,1,0,176,4,82,3,0,0,64,65,1,1,1,1,2,2,2,3,3,1,1,1,169}; /* BMA280 */
#endif
#endif

uint8_t MAG_Spec[39] =
#if defined(BSX_LITE)
{39,0,2,1,20,5,20,5,196,9,6,9,112,23,0,0,128,61,205,204,76,63,0,0,224,64,1,1,1,1,1,1,1,1,1,1,1,134,84}; /* BMM150 */
#else
#if defined(INCLUDE_BMM050API)
{39,0,1,1,232,3,232,3,232,3,20,100,112,23,0,0,128,61,205,204,76,63,0,0,224,64,1,1,1,1,1,1,1,1,1,1,1,11,213}; /* BMM050 */
#elif defined(INCLUDE_BMM150API)
{39,0,2,1,20,5,20,5,196,9,20,100,112,23,0,0,128,61,205,204,76,63,0,0,224,64,1,1,1,1,1,1,1,1,1,1,1,62,103}; /* BMM150 */
#endif
#endif

#if defined(BSX_LITE)
uint8_t GYRO_Spec[14] = {14,0,1,1,3,9,12,136,19,16,1,1,129,46}; /* bmg160 */
#else
uint8_t GYRO_Spec[14] = {14,0,1,1,1,7,200,136,19,16,1,1,203,8}; /* bmg160 */
#endif

const uint8_t USE_Case[] =
#if defined(BSX_LITE)

#if defined(INCLUDE_USECASE_NDOF)
{
/* BSXLite Usecase configuration string */
116,6,1,1,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,51,51,
179,62,205,204,12,63,205,204,12,63,51,51,51,63,51,51,51,63,205,204,76,63,1,0,9,4,2,23,183,209,56,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,23,183,209,56,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,23,183,
209,56,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,189,55,134,53,189,55,134,53,189,55,134,53,0,0,0,0,0,0,16,66,232,3,5,0,45,0,132,3,176,4,150,0,8,150,0,13,1,1,0,0,0,0,0,0,0,0,0,0,0,0,128,
63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,51,51,179,62,205,204,12,63,205,204,12,63,51,
51,51,63,51,51,51,63,205,204,76,62,1,6,4,1,0,5,0,65,1,64,1,36,0,120,0,4,1,20,20,2,2,0,4,0,0,128,63,205,204,204,61,154,153,153,63,205,204,204,62,205,204,204,61,1,0,20,0,16,4,120,0,8,0,
0,5,154,153,25,63,154,153,25,63,80,0,9,0,30,0,232,3,80,0,65,0,4,0,4,0,0,128,62,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,62,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,62,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,128,64,181,254,22,55,181,254,22,55,181,254,22,55,139,222,169,56,0,0,224,64,13,1,1,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,205,204,204,61,1,9,9,3,19,50,163,4,205,12,100,40,4,13,0,1,154,
153,153,62,154,153,153,62,205,204,204,62,154,153,25,63,154,153,153,62,0,0,128,62,154,153,153,62,236,81,184,62,205,204,76,63,205,204,76,63,205,204,76,63,205,204,76,63,205,204,76,62,205,
204,76,62,205,204,76,62,205,204,76,62,0,194,184,178,62,53,250,142,60,10,0,10,0,0,2,0,10,0,80,119,86,61,13,0,0,128,62,143,194,245,60,10,215,163,60,100,128,52,45,70,1,10,0,80,0,0,0,192,
63,0,0,0,64,9,2,0,0,200,65,0,0,128,66,0,0,128,65,0,0,192,63,205,204,76,61,194,184,178,61,50,37,59,24,71,0,0,160,64,154,153,25,63,80,119,86,61,0,1,205,204,76,63,0,0,96,64,0,0,32,64,205,
204,204,61,4,143,194,245,60,2,1,2,3,4,1,10,176,4,88,2,10,215,35,60,10,0,10,0,0,0,250,67,0,0,122,68,0,0,160,63,0,0,72,66,0,0,128,63,0,0,128,62,205,204,204,61,0,0,32,66,0,0,128,62,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,128,62,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,62,0,36,116,73,0,0,0,0,0,0,0,0,0,0,0,0,0,36,116,73,0,0,0,0,0,0,0,0,0,0,0,0,0,36,116,73,0,0,192,64,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,192,64,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,192,64,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,64,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,128,64,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,64,0,0,128,64,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,64,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,128,64,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,64,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,64,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,128,64,10,215,35,60,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10,215,35,60,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10,215,35,60,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,23,183,209,56,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,23,183,209,56,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,23,183,209,56,0,0,128,63,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,172,197,39,55,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,172,197,39,55,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,172,197,39,55,0,36,116,73,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,36,116,73,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,36,116,73,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,7,4,3,0,0,0,0,0,0,10,3,4,25,64,18,24,0,64,114,8,0,13,226,109
};
#endif /* INCLUDE_USECASE_NDOF */

#else /* Full BSX Library use cases */
#if defined(INCLUDE_USECASE_NDOF)
{62,6,1,1,1,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,51,51,179,62,205,204,12,63,0,0,0,0,51,51,51,63,205,204,76,63,1,0,7,2,23,183,209,56,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,23,183,209,56,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,23,183,209,56,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,189,55,134,53,189,55,134,53,189,55,134,53,0,0,0,0,0,0,16,66,232,3,5,0,45,0,132,3,176,4,150,0,8,150,0,12,1,3,1,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,51,51,179,62,205,204,12,63,0,0,0,0,51,51,51,63,205,204,76,62,1,3,2,0,5,0,65,1,64,1,36,0,120,0,4,1,20,20,2,2,0,4,0,0,128,63,205,204,204,61,154,153,153,63,205,204,204,62,205,204,204,61,1,0,20,0,16,4,120,0,8,0,0,5,154,153,25,63,154,153,25,63,80,0,9,0,30,0,232,3,80,0,65,0,4,0,4,0,0,128,62,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,62,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,62,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,64,181,254,22,55,181,254,22,55,181,254,22,55,139,222,169,56,0,0,224,64,12,1,1,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,205,204,204,61,1,7,3,19,50,163,4,205,12,100,40,4,12,0,3,154,153,153,62,154,153,153,62,205,204,204,62,154,153,25,63,154,153,153,62,0,0,128,62,154,153,153,62,236,81,184,62,205,204,76,63,205,204,76,63,205,204,76,63,205,204,76,63,205,204,76,62,205,204,76,62,205,204,76,62,205,204,76,62,1,194,184,178,62,53,250,142,60,10,0,10,0,0,2,0,10,0,80,119,86,61,12,0,0,128,62,143,194,245,60,10,215,163,60,100,128,52,45,70,10,0,80,0,0,0,192,63,0,0,0,64,9,2,0,0,200,65,0,0,128,66,0,0,128,65,0,0,192,63,205,204,76,61,194,184,178,61,50,37,59,24,71,0,0,160,64,154,153,25,63,80,119,86,61,0,1,205,204,76,63,0,0,96,64,0,0,32,64,205,204,204,61,4,143,194,245,60,2,2,4,5,1,10,176,4,88,2,10,215,35,60,10,0,10,0,0,0,250,67,0,0,122,68,0,0,160,63,0,0,72,66,0,0,128,63,0,0,128,62,205,204,204,61,0,0,32,66,0,0,128,62,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,62,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,62,0,36,116,73,0,0,0,0,0,0,0,0,0,0,0,0,0,36,116,73,0,0,0,0,0,0,0,0,0,0,0,0,0,36,116,73,0,0,192,64,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,192,64,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,192,64,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,64,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,64,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,64,0,0,128,64,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,64,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,64,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,64,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,64,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,64,10,215,35,60,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10,215,35,60,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10,215,35,60,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,23,183,209,56,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,23,183,209,56,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,23,183,209,56,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,128,63,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,172,197,39,55,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,172,197,39,55,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,172,197,39,55,0,36,116,73,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,36,116,73,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,36,116,73,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10,3,4,25,64,18,24,0,64,114,8,0,12,194,123};
#endif
#endif /* BSX_LITE */

/* Mag Scaling Factor is (1300uT/4095) * 16 = 5.08 approx 5 */
#define MAG_SCALING_FACTOR	5

static PhysSensorId_t highRateSensor;
static uint32_t work_mode_cur;
static uint32_t work_mode_new;
libraryinput_t LibInput;
/* corrected sensor data */
ts_dataxyzf32 cor_acc;
ts_dataxyzf32 raw_acc;
ts_dataxyzf32 filt_mag;
ts_dataxyzf32 raw_mag;
ts_dataxyzf32 cor_mag;
ts_dataxyzf32 cor_gyro;
ts_dataxyzf32 raw_gyro;
ts_dataeulerf32 f32_euler;
ts_dataquatf32 f32_quat;
uint64_t stepCount;
/* Bit field flags for sensor enable/disable */
uint64_t virtualSensors;
uint32_t phySensors;

uint8_t mag_accuracy;
ts_calibprofile mag_calibProfile;

typedef struct {
	enum LPCSH_SENSOR_ID virtSensor;
	uint8_t phySensor;
}SensorMap_T;

SensorMap_T SensorMap[] = {
	{LPCSH_SENSOR_ID_ACCELEROMETER, 1 << PHYS_ACCEL_ID},
	{LPCSH_SENSOR_ID_ACCELEROMETER_UNCALIBRATED, 1 << PHYS_ACCEL_ID},
	{LPCSH_SENSOR_ID_MAGNETIC_FIELD, 1 << PHYS_MAG_ID},
	{LPCSH_SENSOR_ID_MAGNETIC_FIELD_UNCALIBRATED, 1 << PHYS_MAG_ID},
	{LPCSH_SENSOR_ID_GYROSCOPE, 1 << PHYS_GYRO_ID},
	{LPCSH_SENSOR_ID_GYROSCOPE_UNCALIBRATED, 1 << PHYS_GYRO_ID},
	{LPCSH_SENSOR_ID_LIGHT, 1 << PHYS_AMBIENT_ID},
	{LPCSH_SENSOR_ID_PRESSURE, 1 << PHYS_PRESSURE_ID},
	{LPCSH_SENSOR_ID_PROXIMITY, 1 << PHYS_PROX_ID},
	{LPCSH_SENSOR_ID_ORIENTATION, ((1 << PHYS_ACCEL_ID) | (1 << PHYS_MAG_ID) | (1 << PHYS_GYRO_ID))},
	{LPCSH_SENSOR_ID_ROTATION_VECTOR, ((1 << PHYS_ACCEL_ID) | (1 << PHYS_MAG_ID) | (1 << PHYS_GYRO_ID))},
	{LPCSH_SENSOR_ID_STEP_COUNTER, ((1 << PHYS_ACCEL_ID) | (1 << PHYS_MAG_ID) | (1 << PHYS_GYRO_ID))},
};


/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/


/** Initialize algorithm module */
void Algorithm_Init(void)
{
	ts_workingModes s_workingmode;
	initParam_t InitParam;
	uint8_t err;

	/* Set Mag Data Rate to 25Hz */
	g_phySensors[2]->hw->setDelay(g_phySensors[2], 40);

	InitParam.accelspec = ACC_Spec;
	InitParam.magspec = MAG_Spec;
	InitParam.gyrospec = GYRO_Spec;
	InitParam.usecase = (uint8_t *)USE_Case;

	err = bsx_init(&InitParam);

	if(err || (InitParam.accelspec_status == 0) || (InitParam.magspec_status == 0) || (InitParam.gyrospec_status == 0) || (InitParam.usecase_status == 0)) {
		while(1);
	}

	s_workingmode.opMode = BSX_WORKINGMODE_SLEEP;

	bsx_set_workingmode(&s_workingmode);
	work_mode_cur = BSX_WORKINGMODE_SLEEP;

#if !defined(BSX_LITE)
	err = bsx_set_stepdetectionopmode(1);
	if(err)
		while(1);

	bsx_set_orientcoordinatesystem(BSX_ORIENTCOORDINATESYSTEM_ANDROID);
#endif
}

void Algorithm_EnableSensor(enum LPCSH_SENSOR_ID hif_SensorId, uint8_t enable)
{
	uint32_t i, j;
	ts_workingModes s_workingmode;
	uint32_t sensor_index;
	uint8_t found;

	/* Find Sensor Index from map table */
	for(i = 0; i < sizeof(SensorMap)/sizeof(SensorMap[0]); i++) {
		if(SensorMap[i].virtSensor == hif_SensorId) {
			sensor_index = i;
			break;
		}
	}
	/* Process only if sensor found in map table */
	if(i < sizeof(SensorMap)/sizeof(SensorMap[0])) {
		/* Sensor enable requested */
		if(enable) {
			virtualSensors |= (1 << hif_SensorId);
			for( i = 0; i < PHYS_MAX_ID; i++) {
				/* If needed physical sensors not enabled then enable them */
				if((SensorMap[sensor_index].phySensor & (1 << i)) && ((phySensors & (1 << i)) == 0)) {
					PhysSensors_Enable( (PhysSensorId_t) i, enable);
					phySensors |= (1 << i);
					/* Update work mode, data rate and range for Accel, Gyro and Mag */
					switch(i) {
						case PHYS_ACCEL_ID:
							work_mode_new = (work_mode_new & (~(3 << BSX_BITSHIFT_ACCOPMODE))) | (BSX_OPMODE_REGULAR << BSX_BITSHIFT_ACCOPMODE);
#if !defined(BSX_LITE)
							bsx_set_accdatarate(BSX_DATARATE_100HZ); /* Data rate @ 100Hz */
							bsx_set_accrange(BSX_ACCRANGE_4G); /* Accel Range set to 4G */
#endif
							break;
						case PHYS_GYRO_ID:
							work_mode_new = (work_mode_new & (~(3 << BSX_BITSHIFT_GYROOPMODE))) | (BSX_OPMODE_REGULAR << BSX_BITSHIFT_GYROOPMODE);
#if !defined(BSX_LITE)
							bsx_set_gyrodatarate(BSX_DATARATE_100HZ); /* Data rate @ 100Hz */
							bsx_set_gyrorange(BSX_GYRORANGE_2000DPS); /* 2000 dps */
#endif
							break;
						case PHYS_MAG_ID:
							work_mode_new = (work_mode_new & (~(3 << BSX_BITSHIFT_MAGOPMODE))) | (BSX_OPMODE_REGULAR << BSX_BITSHIFT_MAGOPMODE);
#if !defined(BSX_LITE)
							bsx_set_magdatarate(BSX_DATARATE_25HZ); /* Data rate @ 25Hz */
#endif
							break;
						default:
							break;
					}
				}
			}
		}
		/* Sensor disable requested */
		else {
			virtualSensors &= ~(1 << hif_SensorId);
			for( i = 0; i < PHYS_MAX_ID; i++) {
				/* If needed physical sensors not enabled then enable them */
				if(SensorMap[sensor_index].phySensor & (1 << i)) {
					/* Find if any enabled virtual sensor needs this physical sensor, if not disable it */
					found = 0;
					for(j = 0; j < sizeof(SensorMap)/sizeof(SensorMap[0]); j++) {
						if((virtualSensors & (1 << SensorMap[j].virtSensor)) && (SensorMap[j].phySensor & (1 << i))) {
							found = 1;
							break;
						}
					}
					if(found == 0) {
						PhysSensors_Enable((PhysSensorId_t) i, enable);
						phySensors &= ~(1 << i);
						/* Update work mode, data rate and range for Accel, Gyro and Mag */
						switch(i) {
							case PHYS_ACCEL_ID:
								work_mode_new = (work_mode_new & (~(3 << BSX_BITSHIFT_ACCOPMODE)));
								break;
							case PHYS_GYRO_ID:
								work_mode_new = (work_mode_new & (~(3 << BSX_BITSHIFT_GYROOPMODE)));
								break;
							case PHYS_MAG_ID:
								work_mode_new = (work_mode_new & (~(3 << BSX_BITSHIFT_MAGOPMODE)));
								break;
							default:
								break;
						}
					}
				}
			}
		}
	}

	if((work_mode_new & (BSX_OPMODE_REGULAR << BSX_BITSHIFT_ACCOPMODE)) &&
			(work_mode_new & (BSX_OPMODE_REGULAR << BSX_BITSHIFT_MAGOPMODE)) &&
			(work_mode_new & (BSX_OPMODE_REGULAR << BSX_BITSHIFT_GYROOPMODE))) {

		/* Work Mode is Acc + Mag + Gyro, so enable NDOF */
		work_mode_new = BSX_WORKINGMODE_NDOF;
		highRateSensor = PHYS_GYRO_ID;
	}
	else if((work_mode_new & (BSX_OPMODE_REGULAR << BSX_BITSHIFT_ACCOPMODE)) &&
			(work_mode_new & (BSX_OPMODE_REGULAR << BSX_BITSHIFT_MAGOPMODE))) {
		/* Work Mode is Mag + Accel */
		work_mode_new = BSX_WORKINGMODE_ACCMAG;
		highRateSensor = PHYS_ACCEL_ID;
	}
	else if((work_mode_new & (BSX_OPMODE_REGULAR << BSX_BITSHIFT_ACCOPMODE)) &&
			(work_mode_new & (BSX_OPMODE_REGULAR << BSX_BITSHIFT_GYROOPMODE))) {

		/* Work Mode is Acc + Gyro */
		work_mode_new = BSX_WORKINGMODE_ACCGYRO;
		highRateSensor = PHYS_GYRO_ID;
	}
	else if((work_mode_new & (BSX_OPMODE_REGULAR << BSX_BITSHIFT_MAGOPMODE)) &&
			(work_mode_new & (BSX_OPMODE_REGULAR << BSX_BITSHIFT_GYROOPMODE))) {
		/* Work Mode is Mag + Gyro */
		work_mode_new = BSX_WORKINGMODE_MAGGYRO;
		highRateSensor = PHYS_GYRO_ID;
	}
	else {
		/* Individual Sensor work mode */
		if(enable) {
			switch(hif_SensorId) {
				case LPCSH_SENSOR_ID_ACCELEROMETER:
				case LPCSH_SENSOR_ID_ACCELEROMETER_UNCALIBRATED:
					work_mode_new = BSX_WORKINGMODE_ACCONLY;
					highRateSensor = PHYS_ACCEL_ID;
					break;
				case LPCSH_SENSOR_ID_MAGNETIC_FIELD:
				case LPCSH_SENSOR_ID_MAGNETIC_FIELD_UNCALIBRATED:
					work_mode_new = BSX_WORKINGMODE_MAGONLY;
					highRateSensor = PHYS_MAG_ID;
					break;
				case LPCSH_SENSOR_ID_GYROSCOPE:
				case LPCSH_SENSOR_ID_GYROSCOPE_UNCALIBRATED:
					work_mode_new = BSX_WORKINGMODE_GYROONLY;
					highRateSensor = PHYS_GYRO_ID;
					break;
				default:
					break;
			}
		}
		/* Disable request for the only enabled sensor so Sleep Mode */
		else {
			work_mode_new = BSX_WORKINGMODE_SLEEP;
		}
	}

	/* If there is a change in the working mode expected then set the new working mode */
	if(work_mode_new != work_mode_cur) {
		s_workingmode.opMode = work_mode_new;
		/* Check for mag calibration accuracy */
		bsx_get_magcalibaccuracy(&mag_accuracy);
		/* If mag accuracy is high then read and update mag calib profile */
		if(mag_accuracy == BSX_SENSOR_STATUS_ACCURACY_HIGH) {
			bsx_get_magcalibprofile(&mag_calibProfile);
		}
		/* Change working mode */
		bsx_set_workingmode(&s_workingmode);
		/* If we have an accurate calib profile then set the mag calib profile */
		if(mag_calibProfile.accuracy == BSX_SENSOR_STATUS_ACCURACY_HIGH) {
			bsx_set_magcalibprofile(&mag_calibProfile);
		}
		work_mode_cur = work_mode_new;
	}
}

/** Process the sensor data */
uint32_t Algorithm_Process(PhysicalSensor_t *pSens)
{
	struct lpcsh_sensor_node hostBuffer;
	uint8_t err;

	/* Update Data structure to algorithm */
	switch(pSens->id) {
		case PHYS_ACCEL_ID:
			LibInput.acc.data.x = pSens->data16[0];
			LibInput.acc.data.y = pSens->data16[1];
			LibInput.acc.data.z = pSens->data16[2];
			LibInput.acc.time_stamp = g_Timer.getUsFromTicks(pSens->ts_lastSample);
			break;
		case PHYS_GYRO_ID:
			LibInput.gyro.data.x = pSens->data16[0];
			LibInput.gyro.data.y = pSens->data16[1];
			LibInput.gyro.data.z = pSens->data16[2];
			LibInput.gyro.time_stamp = g_Timer.getUsFromTicks(pSens->ts_lastSample);
			break;
		case PHYS_MAG_ID:
			LibInput.mag.data.x = pSens->data16[0];
			LibInput.mag.data.y = pSens->data16[1];
			LibInput.mag.data.z = pSens->data16[2];
			LibInput.mag.time_stamp = g_Timer.getUsFromTicks(pSens->ts_lastSample);
			break;
		case PHYS_PRESSURE_ID:
			hostBuffer.header.sensorId = LPCSH_SENSOR_ID_PRESSURE;
			hostBuffer.header.timeStamp = pSens->ts_lastSample;
			hostBuffer.data.sensorData.Data[0] = pSens->data16[0];
			hostBuffer.data.sensorData.Data[1] = pSens->data16[1];
			hostBuffer.data.sensorData.Data[2] = 0;
			Hostif_QueueBuffer((uint8_t *)&hostBuffer, sizeof(hostBuffer.data.sensorData));
			break;
		case PHYS_AMBIENT_ID:
			hostBuffer.header.sensorId = LPCSH_SENSOR_ID_LIGHT;
			hostBuffer.header.timeStamp = pSens->ts_lastSample;
			hostBuffer.data.sensorData.Data[0] = pSens->data16[0];
			hostBuffer.data.sensorData.Data[1] = 0;
			hostBuffer.data.sensorData.Data[2] = 0;
			Hostif_QueueBuffer((uint8_t *)&hostBuffer, sizeof(hostBuffer.data.sensorData));
			break;
		case PHYS_PROX_ID:
			hostBuffer.header.sensorId = LPCSH_SENSOR_ID_PROXIMITY;
			hostBuffer.header.timeStamp = pSens->ts_lastSample;
			hostBuffer.data.sensorData.Data[0] = pSens->data16[0];
			hostBuffer.data.sensorData.Data[1] = 0;
			hostBuffer.data.sensorData.Data[2] = 0;
			Hostif_QueueBuffer((uint8_t *)&hostBuffer, sizeof(hostBuffer.data.sensorData));
			break;
	}
	/* Run Algo when sensor with highest rate is sampled */
	if(pSens->id == highRateSensor) {
		/* enter PLL mode for high speed processing */
		ResMgr_EnterPLLMode();
		/* Call Algo */
		err = bsx_dostep(&LibInput);
		if(err) {
			/* exit PLL mode */
			ResMgr_EnterNormalMode();
			return 0;
		}

		switch(work_mode_cur)
		{
			/* Read Sensors based on Working mode and virtual sensors that are enabled */
			case BSX_WORKINGMODE_NDOF:
				if(virtualSensors & (1 << LPCSH_SENSOR_ID_ORIENTATION)) {
					err = bsx_get_orientdata_euler_rad(&f32_euler);
					if(!err) {
						hostBuffer.header.sensorId = LPCSH_SENSOR_ID_ORIENTATION;
						hostBuffer.header.timeStamp = g_Timer.getUsFromTicks(pSens->ts_lastSample);
						hostBuffer.data.orientationData.Data[0] = (int32_t)((double)f32_euler.p * 16777216.0);
						hostBuffer.data.orientationData.Data[1] = (int32_t)((double)f32_euler.r * 16777216.0);
						hostBuffer.data.orientationData.Data[2] = (int32_t)((double)f32_euler.y * 16777216.0);
						Hostif_QueueBuffer((uint8_t *)&hostBuffer, sizeof(hostBuffer.data.orientationData));
					}
				}
				if(virtualSensors & (1 << LPCSH_SENSOR_ID_ROTATION_VECTOR)) {
					err = bsx_get_orientdata_quat(&f32_quat);
					if(!err) {
						hostBuffer.header.sensorId = LPCSH_SENSOR_ID_ROTATION_VECTOR;
						hostBuffer.header.timeStamp = g_Timer.getUsFromTicks(pSens->ts_lastSample);
						hostBuffer.data.quaternionData.X = (int32_t)((double)f32_quat.x * 16777216.0);
						hostBuffer.data.quaternionData.Y = (int32_t)((double)f32_quat.y * 16777216.0);
						hostBuffer.data.quaternionData.Z = (int32_t)((double)f32_quat.z * 16777216.0);
						hostBuffer.data.quaternionData.W = (int32_t)((double)f32_quat.w * 16777216.0);
						hostBuffer.data.quaternionData.E_EST = 0;
						Hostif_QueueBuffer((uint8_t *)&hostBuffer, sizeof(hostBuffer.data.quaternionData));
					}
				}
#if !defined(BSX_LITE)
				if(virtualSensors & (1 << LPCSH_SENSOR_ID_STEP_COUNTER)) {
					err = bsx_get_stepcount(&stepCount);
					if(!err) {
						hostBuffer.header.sensorId = LPCSH_SENSOR_ID_STEP_COUNTER;
						hostBuffer.header.timeStamp = g_Timer.getUsFromTicks(pSens->ts_lastSample);
						/* Note - stepCount is downgraded from 64 bits to 32 bits */
						hostBuffer.data.stepData.numTotalsteps = (uint32_t)stepCount;
						Hostif_QueueBuffer((uint8_t *)&hostBuffer, sizeof(hostBuffer.data.stepData));
					}
				}
#endif
			default:
				if(virtualSensors & (1 << LPCSH_SENSOR_ID_ACCELEROMETER_UNCALIBRATED)) {
					err = bsx_get_accrawdata(&raw_acc);
					if(!err) {
						hostBuffer.header.sensorId = LPCSH_SENSOR_ID_ACCELEROMETER_UNCALIBRATED;
						hostBuffer.header.timeStamp = g_Timer.getUsFromTicks(pSens->ts_lastSample);
						hostBuffer.data.sensorData.Data[0] = (int16_t)(raw_acc.x * 16777216);
						hostBuffer.data.sensorData.Data[1] = (int16_t)(raw_acc.y * 16777216);
						hostBuffer.data.sensorData.Data[2] = (int16_t)(raw_acc.z * 16777216);
						Hostif_QueueBuffer((uint8_t *)&hostBuffer, sizeof(hostBuffer.data.sensorData));
					}
				}
				if(virtualSensors & (1 << LPCSH_SENSOR_ID_ACCELEROMETER)) {
#if defined(BSX_LITE)
					err = bsx_get_accrawdata(&raw_acc);
					if(!err) {
						hostBuffer.header.sensorId = LPCSH_SENSOR_ID_ACCELEROMETER;
						hostBuffer.header.timeStamp = g_Timer.getUsFromTicks(pSens->ts_lastSample);
						hostBuffer.data.sensorData.Data[0] = (int16_t)(raw_acc.x * 16777216);
						hostBuffer.data.sensorData.Data[1] = (int16_t)(raw_acc.y * 16777216);
						hostBuffer.data.sensorData.Data[2] = (int16_t)(raw_acc.z * 16777216);
						Hostif_QueueBuffer((uint8_t *)&hostBuffer, sizeof(hostBuffer.data.sensorData));
					}
#else
					err = bsx_get_acccordata(&cor_acc);
					if(!err) {
						hostBuffer.header.sensorId = LPCSH_SENSOR_ID_ACCELEROMETER;
						hostBuffer.header.timeStamp = g_Timer.getUsFromTicks(pSens->ts_lastSample);
						hostBuffer.data.sensorData.Data[0] = (int16_t)(cor_acc.x * 16777216);
						hostBuffer.data.sensorData.Data[1] = (int16_t)(cor_acc.y * 16777216);
						hostBuffer.data.sensorData.Data[2] = (int16_t)(cor_acc.z * 16777216);
						Hostif_QueueBuffer((uint8_t *)&hostBuffer, sizeof(hostBuffer.data.sensorData));
					}
#endif
				}
				if(virtualSensors & (1 << LPCSH_SENSOR_ID_MAGNETIC_FIELD_UNCALIBRATED)) {
					err = bsx_get_magrawdata(&raw_mag);
					if(!err) {
						hostBuffer.header.sensorId = LPCSH_SENSOR_ID_MAGNETIC_FIELD_UNCALIBRATED;
						hostBuffer.header.timeStamp = g_Timer.getUsFromTicks(pSens->ts_lastSample);
						hostBuffer.data.sensorData.Data[0] = (int16_t)(raw_mag.x * 16777216);
						hostBuffer.data.sensorData.Data[1] = (int16_t)(raw_mag.y * 16777216);
						hostBuffer.data.sensorData.Data[2] = (int16_t)(raw_mag.z * 16777216);
						Hostif_QueueBuffer((uint8_t *)&hostBuffer, sizeof(hostBuffer.data.sensorData));
					}
				}
				if(virtualSensors & (1 << LPCSH_SENSOR_ID_MAGNETIC_FIELD)) {
#if defined(BSX_LITE)
					err = bsx_get_magcordata(&cor_mag);
					if(!err) {
						hostBuffer.header.sensorId = LPCSH_SENSOR_ID_MAGNETIC_FIELD;
						hostBuffer.header.timeStamp = g_Timer.getUsFromTicks(pSens->ts_lastSample);
						hostBuffer.data.sensorData.Data[0] = (int16_t)(cor_mag.x * 16777216);
						hostBuffer.data.sensorData.Data[1] = (int16_t)(cor_mag.y * 16777216);
						hostBuffer.data.sensorData.Data[2] = (int16_t)(cor_mag.z * 16777216);
						Hostif_QueueBuffer((uint8_t *)&hostBuffer, sizeof(hostBuffer.data.sensorData));
					}
#else
					err = bsx_get_magfiltdata1(&filt_mag);
					if(!err) {
						hostBuffer.header.sensorId = LPCSH_SENSOR_ID_MAGNETIC_FIELD;
						hostBuffer.header.timeStamp = g_Timer.getUsFromTicks(pSens->ts_lastSample);
						hostBuffer.data.sensorData.Data[0] = (int16_t)(filt_mag.x * 16777216);
						hostBuffer.data.sensorData.Data[1] = (int16_t)(filt_mag.y * 16777216);
						hostBuffer.data.sensorData.Data[2] = (int16_t)(filt_mag.z * 16777216);
						Hostif_QueueBuffer((uint8_t *)&hostBuffer, sizeof(hostBuffer.data.sensorData));
					}
#endif
				}
				if(virtualSensors & (1 << LPCSH_SENSOR_ID_GYROSCOPE_UNCALIBRATED)) {
					err = bsx_get_gyrorawdata_rps(&raw_gyro);
					if(!err) {
						hostBuffer.header.sensorId = LPCSH_SENSOR_ID_GYROSCOPE_UNCALIBRATED;
						hostBuffer.header.timeStamp = g_Timer.getUsFromTicks(pSens->ts_lastSample);
						hostBuffer.data.sensorData.Data[0] = (int16_t)(raw_gyro.x * 16777216);
						hostBuffer.data.sensorData.Data[1] = (int16_t)(raw_gyro.y * 16777216);
						hostBuffer.data.sensorData.Data[2] = (int16_t)(raw_gyro.z * 16777216);
						Hostif_QueueBuffer((uint8_t *)&hostBuffer, sizeof(hostBuffer.data.sensorData));
					}
				}
				if(virtualSensors & (1 << LPCSH_SENSOR_ID_GYROSCOPE)) {
					err = bsx_get_gyrocordata_rps(&cor_gyro);
					if(!err) {
						hostBuffer.header.sensorId = LPCSH_SENSOR_ID_GYROSCOPE;
						hostBuffer.header.timeStamp = g_Timer.getUsFromTicks(pSens->ts_lastSample);
						hostBuffer.data.sensorData.Data[0] = (int16_t)(cor_gyro.x * 16777216);
						hostBuffer.data.sensorData.Data[1] = (int16_t)(cor_gyro.y * 16777216);
						hostBuffer.data.sensorData.Data[2] = (int16_t)(cor_gyro.z * 16777216);
						Hostif_QueueBuffer((uint8_t *)&hostBuffer, sizeof(hostBuffer.data.sensorData));
					}
				}
				break;
		}

		/* exit PLL mode */
		ResMgr_EnterNormalMode();
	}

	return 0;
}

/** Run background calibration routines - Not needed for BSX */
uint32_t Algorithm_bgProcess(int32_t *pEstSleepTime)
{
	return 1;
}

