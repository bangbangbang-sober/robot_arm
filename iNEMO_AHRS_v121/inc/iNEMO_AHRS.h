/**
*
* \file    iNEMO_AHRS.h
* \author  ART Team IMS-Systems Lab
* \version V1.2.1  [FW v2.0.0]
* \date    15/10/2010
* \brief   This file is the header the AHRS of iNEMO,   This the interface from main
*
********************************************************************************
*
* \details
*
* This is the interface from your application
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*

* <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
*/


#ifndef __INEMO_AHRS
#define __INEMO_AHRS

#include <stddef.h>

/** @defgroup iNEMO_Engine_Lite        iNEMO Engine Lite
  * @{
  */

/** @defgroup iNEMO_AHRS        iNEMO AHRS
  * @{
  */

/**
 * @brief This define sets the first digit of the AHRS version number.
 */ 
#define iNEMO_AHRS_VERSION_N		1

/**
 * @brief This define sets the second digit of the AHRS version number.
 */  
#define iNEMO_AHRS_VERSION_SUBN		2

/**
 * @brief This define sets the third digit of the AHRS version number.
 */  
#define iNEMO_AHRS_VERSION_SUBMIN	1

/**
 * @brief This macro is used to set the AHRS version number.
 */  
#define iNEMO_AHRS_LIBRARY_VER_HEX  (((uint16_t)iNEMO_AHRS_VERSION_N<<8)|((uint16_t)iNEMO_AHRS_VERSION_SUBN<<4)|((uint16_t)iNEMO_AHRS_VERSION_SUBMIN))

/** @defgroup iNEMO_AHRS_Structures     iNEMO AHRS Structures
  * @{
  */

/**
 * @struct iNEMO_SENSORDATA
 * @brief Sensor data struct
*/

typedef struct
{
  unsigned int m_nCount;  /*!< It is used to perform correction at different frequencies */
  float   m_fDeltaTime;   /*!< It should trace the time difference */

  float m_fAcc[3];		/*!< The Acceleration Measurement */
  float m_fGyro[3];		/*!< The Gyroscope Measurement */
  float m_fMag[3];		/*!< The Magnitude Measurement */

  float m_fScaleFactor[9];      /*!< The Scale Factor Vector for each Measurement */
  float m_fOffset[9];		/*!< The Offset Vector for each Measurement */
  
  float m_fAccRef[3];           /*!< The gravitational vector refer */
  float m_fMagRef[3];           /*!< The magnetic vector refer */

  float m_fVarAcc;              /*!< The accelerometer variance */
  float m_fVarMag;              /*!< The magnetometer variance */
  float m_fVarGyro;             /*!< The gyroscope variance */
  
} iNEMO_SENSORDATA;

/**
 * @struct iNEMO_EULER_ANGLES
 * @brief iNEMO Euler Angle Struct
 */

typedef struct
{
  float m_fRoll;          /*!< Roll Angle */ 
  float m_fPitch;         /*!< Pitch Angle */  
  float m_fYaw;           /*!< Yaw Angle */ 
} iNEMO_EULER_ANGLES;

/**
 * @typedef iNEMO_QUAT
 * @brief iNEMO Quaternion Type
 */

typedef float iNEMO_QUAT[4];

/**
*@}
 */ /* end of group iNEMO_AHRS_Structures */



/** @defgroup iNEMO_AHRS_Function       iNEMO AHRS Functions
* @{
*/


/**
* @brief  Initialize all the AHRS variable
* @param pExtSensorData : Sensor Data
* @param pExtAngle      : Roll, Pitch and Yaw Angles
* @param pExtQuat       : Quaternion
* @retval None
*/
void iNEMO_AHRS_Init(iNEMO_SENSORDATA*    pExtSensorData,
                    iNEMO_EULER_ANGLES*  pExtAngle,
                    iNEMO_QUAT*          pExtQuat);


/**
* @brief  Update AHRS variables
* @param pExtSensorData  : Sensor Data
* @param pExtAngle      : Roll, Pitch and Yaw Angles
* @param pExtQuat       : Quaternion
* @retval None
*/
void iNEMO_AHRS_Update(iNEMO_SENSORDATA*    pExtSensorData,
                      iNEMO_EULER_ANGLES*  pExtAngle,
                      iNEMO_QUAT*          pExtQuat);


/**
* @brief  Delete all the AHRS variables allocated dinamically
* @param pSensorData : Sensor Data
* @param pAngle      : Roll, Pitch and Yaw Angles
* @param pQuat       : Quaternion
* @retval None
*/
void iNEMO_AHRS_DeInit(iNEMO_SENSORDATA*    pSensorData,
                      iNEMO_EULER_ANGLES*  pAngle,
                      iNEMO_QUAT*          pQuat);
 



/**
 * @brief Dynamic Memory allocation function used in the iNEMO platform. See @ref iNEMO_Malloc.
 */
extern void iNEMO_Free(void *p);

/**
 * @brief Memory release function used in the iNEMO platform. See @ref iNEMO_Free.
 */
extern void *iNEMO_Malloc(size_t size);

/**
 * @}
 */ 

/**
 * @}
 */ 

/**
 * @}
 */ 

#endif /*__INEMO_AHRS */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/


