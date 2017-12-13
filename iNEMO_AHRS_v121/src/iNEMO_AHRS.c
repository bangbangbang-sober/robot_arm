/**
*
* \file    iNEMO_AHRS.c
* \author  ART Team IMS-Systems Lab
* \version V1.2.1 [FW v2.0.0]
* \date    13/09/2010
* \brief   Implementation file of the AHRS of iNEMO
*
********************************************************************************
*
* \details
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
*/

#include "iNEMO_AHRS.h"
#include "iNEMO_EKF.h"

#include <math.h>

/** @addtogroup iNEMO_Engine_Lite        iNEMO Engine Lite
  * @{
  */

/** @addtogroup iNEMO_AHRS        iNEMO AHRS
  * @{
  */


/** @defgroup iNEMO_AHRS_Private_Define        iNEMO AHRS Private Define
  * @{
  */


/**
 * @brief Macro to retrieve m_nCount field of @ref iNEMO_SENSORDATA.
 */
#define iNEMO_CNT     (pSensorData->m_nCount)

/**
 * @brief Macro to retrieve m_fDeltaTime field of @ref iNEMO_SENSORDATA.
 */
#define iNEMO_DT      (pSensorData->m_fDeltaTime)

/**
 * @brief Macro to retrieve m_fAcc X-Axis field of @ref iNEMO_SENSORDATA.
 */
#define iNEMO_AccX    (pSensorData->m_fAcc[0])

/**
 * @brief Macro to retrieve m_fAcc Y-Axis field of @ref iNEMO_SENSORDATA.
 */
#define iNEMO_AccY    (pSensorData->m_fAcc[1])

/**
 * @brief Macro to retrieve m_fAcc Z-Axis field of @ref iNEMO_SENSORDATA.
 */
#define iNEMO_AccZ    (pSensorData->m_fAcc[2])

/**
 * @brief Macro to retrieve m_fGyro X-Axis field of @ref iNEMO_SENSORDATA.
 */
#define iNEMO_GyroX   (pSensorData->m_fGyro[0])

/**
 * @brief Macro to retrieve m_fGyro Y-Axis field of @ref iNEMO_SENSORDATA.
 */
#define iNEMO_GyroY   (pSensorData->m_fGyro[1])

/**
 * @brief Macro to retrieve m_fGyro Z-Axis field of @ref iNEMO_SENSORDATA.
 */
#define iNEMO_GyroZ   (pSensorData->m_fGyro[2])

/**
 * @brief Macro to retrieve m_fMag X-Axis field of @ref iNEMO_SENSORDATA.
 */
#define iNEMO_MagX    (pSensorData->m_fMag[0])

/**
 * @brief Macro to retrieve m_fMag Y-Axis field of @ref iNEMO_SENSORDATA.
 */
#define iNEMO_MagY    (pSensorData->m_fMag[1])

/**
 * @brief Macro to retrieve m_fMag Z-Axis field of @ref iNEMO_SENSORDATA.
 */
#define iNEMO_MagZ    (pSensorData->m_fMag[2])

/**
 * @brief Macro to retrieve m_fVarAcc field of @ref iNEMO_SENSORDATA.
 */
#define iNEMO_VarAcc     (pSensorData->m_fVarAcc)

/**
 * @brief Macro to retrieve m_fVarMag field of @ref iNEMO_SENSORDATA.
 */
#define iNEMO_VarMag     (pSensorData->m_fVarMag)

/**
 * @brief Macro to retrieve m_fVarGyro field of @ref iNEMO_SENSORDATA.
 */
#define iNEMO_VarGyro    (pSensorData->m_fVarGyro)

/**
 * @brief Macro to retrieve m_fAccRef field of @ref iNEMO_SENSORDATA.
 */
#define iNEMO_AccRef     (pSensorData->m_fAccRef)

/**
 * @brief Macro to retrieve m_fMagRef field of @ref iNEMO_SENSORDATA.
 */
#define iNEMO_MagRef     (pSensorData->m_fMagRef)

/**
* @}
*/

/** @defgroup iNEMO_AHRS_Private_Variables        iNEMO AHRS Private Variables
  * @{
  */

/**
 * @brief The Error Covariance Matrix.
 */
iNEMO_fMATRIX_TYPE* pMat_P;    

/**
 * @brief The State Matrix (Jacobian).
 */
iNEMO_fMATRIX_TYPE* pMat_Fsys;

/**
 * @brief The Covariance Matrix of the Noise Process.
 */
iNEMO_fMATRIX_TYPE* pMat_Q;

/**
 * @brief The Error Covariance Matrix (updated).
 */
iNEMO_fMATRIX_TYPE* pMat_Pnew;

/**
 * @brief Kalman Gain Matrix of Accelerometer.
 */
iNEMO_fMATRIX_TYPE* pMat_Ka;

/**
 * @brief Kalman Gain Matrix of Magnetometer.
 */
iNEMO_fMATRIX_TYPE* pMat_Km;

/**
 * @brief Jacobian Matrix of Accelerometer.
 */
iNEMO_fMATRIX_TYPE* pMat_Hja;

/**
 * @brief Jacobian Matrix of Magnetometer.
 */
iNEMO_fMATRIX_TYPE* pMat_Hjm;

/**
 * @brief The Accelerometer Measurement Noise.
 */
iNEMO_fMATRIX_TYPE* pMat_Ra;

/**
 * @brief The Magnetometer Measurement Noise.
 */
iNEMO_fMATRIX_TYPE* pMat_Rm;

/**
 * @brief State Variable Vector.
 */
float fSV[7];

/**
 * @brief The half of the DT.
 */
float fHalfDt;

/**
 * @brief Acceleration reference. The norm of this vector will be used for the accereation correction.
 */
float fAccRef;

/**
 * @brief Magnetic field reference. The norm of this vector will be used for the magnetic field correction.
 */
float fMagRef;

/**
* @}
*/


/** @addtogroup iNEMO_AHRS_Function
  * @{
  */


/******************************************************************************/
void iNEMO_AHRS_Init(iNEMO_SENSORDATA*    pSensorData,
                    iNEMO_EULER_ANGLES*  pAngle,
                    iNEMO_QUAT*          pQuat)
{

  /* Initialize all the EKF Matrix to Zero */
  pMat_P = iNEMO_fMatCreateZero(7,7);
  pMat_Fsys = iNEMO_fMatCreateUnit(7,7);
  pMat_Q = iNEMO_fMatCreateZero(7,7);

  pMat_Pnew = iNEMO_fMatCreateZero(7,7);

  pMat_Ra = iNEMO_fMatCreateZero(3,3);
  pMat_Rm = iNEMO_fMatCreateZero(3,3);
  pMat_Ka = iNEMO_fMatCreateZero(7,3);
  pMat_Km = iNEMO_fMatCreateZero(7,3);

  /* Assign specific values within the matrixes */
  for (short int i=0; i < 7; ++i)
  {
    iNEMO_MatData(pMat_P)[i][i] = 1.0e-1f;
    iNEMO_MatData(pMat_Q)[i][i] = 5.0e-7f;
  }

  for(short int i=0;i<3;i++)
  {
    iNEMO_MatData(pMat_Ra)[i][i] = iNEMO_VarAcc;
    iNEMO_MatData(pMat_Rm)[i][i] = iNEMO_VarMag;
  }

  pMat_Hja = iNEMO_fMatCreateZero(3,7);
  pMat_Hjm = iNEMO_fMatCreateZero(3,7);


  /* State Vector initialization... */
  /* ...Option 1: {1, 0, 0, 0, 0, 0, 0} */
   fSV[0] = 1;
   for (short int i=1; i < 7; ++i)
   	fSV[i] = 0;

   /* Sensor Data Initialization */
   iNEMO_CNT = 0;

   iNEMO_AccX = 0.0f;
   iNEMO_AccY = 0.0f;
   iNEMO_AccZ = 0.0f;
   iNEMO_GyroX = 0.0f;
   iNEMO_GyroY = 0.0f;
   iNEMO_GyroZ = 0.0f;
   iNEMO_MagX = 0.0f;
   iNEMO_MagY = 0.0f;
   iNEMO_MagZ = 0.0f;

   /* Angle initialization... */
   /* ...Option 1 (equivalent to O1 of State Vector) */
   iNEMO_Roll(pAngle) = 0.0f;
   iNEMO_Pitch(pAngle) = 0.0f;
   iNEMO_Yaw(pAngle) = 0.0f;

   fHalfDt=iNEMO_DT/2;

   fAccRef=0.0;
   fMagRef=0.0;
   for(short int i=0;i<3;i++)
   {
      fAccRef+=(iNEMO_AccRef[i]*iNEMO_AccRef[i]);
      fMagRef+=(iNEMO_MagRef[i]*iNEMO_MagRef[i]);
   }

   fAccRef=sqrt(fAccRef);
   fMagRef=sqrt(fMagRef);

}

/******************************************************************************/
void iNEMO_AHRS_DeInit(iNEMO_SENSORDATA*    pSensorData,
                      iNEMO_EULER_ANGLES*  pAngle,
                      iNEMO_QUAT*          pQuat)
{
  /* Delete the basic EKF Matrix */
  iNEMO_fMatFree(pMat_P);
  iNEMO_fMatFree(pMat_Fsys);
  iNEMO_fMatFree(pMat_Q);

  iNEMO_fMatFree(pMat_Ra);
  iNEMO_fMatFree(pMat_Rm);
  iNEMO_fMatFree(pMat_Ka);
  iNEMO_fMatFree(pMat_Km);

  iNEMO_fMatFree(pMat_Hja);
  iNEMO_fMatFree(pMat_Hjm);
}

/******************************************************************************/
void iNEMO_AHRS_Update(iNEMO_SENSORDATA*    pSensorData,
                      iNEMO_EULER_ANGLES*  pAngle,
                      iNEMO_QUAT*          pQuat)
{
  int i;
  float fGyroX, fGyroY, fGyroZ;
  iNEMO_QUAT fSVnorm;
  float fTempNorm;
  float fH[4];



  /* Self increment of the Counter of the Update Sessions */
  iNEMO_CNT++;

  /* calculate Gyro Values */
  fGyroX = (iNEMO_GyroX - fSV[4]) * fHalfDt;
  fGyroY = (iNEMO_GyroY - fSV[5]) * fHalfDt;
  fGyroZ = (iNEMO_GyroZ - fSV[6]) * fHalfDt;

  /*state transition matrix */
  iNEMO_MatData(pMat_Fsys)[0][1] = -fGyroX;
  iNEMO_MatData(pMat_Fsys)[0][2] = -fGyroY;
  iNEMO_MatData(pMat_Fsys)[0][3] = -fGyroZ;
  iNEMO_MatData(pMat_Fsys)[1][0] =  fGyroX;
  iNEMO_MatData(pMat_Fsys)[1][2] =  fGyroZ;
  iNEMO_MatData(pMat_Fsys)[1][3] = -fGyroY;
  iNEMO_MatData(pMat_Fsys)[2][0] =  fGyroY;
  iNEMO_MatData(pMat_Fsys)[2][1] = -fGyroZ;
  iNEMO_MatData(pMat_Fsys)[2][3] =  fGyroX;
  iNEMO_MatData(pMat_Fsys)[3][0] =  fGyroZ;
  iNEMO_MatData(pMat_Fsys)[3][1] =  fGyroY;
  iNEMO_MatData(pMat_Fsys)[3][2] = -fGyroX;

  iNEMO_MatData(pMat_Fsys)[0][4] =  fSV[1] * fHalfDt;
  iNEMO_MatData(pMat_Fsys)[0][5] =  fSV[2] * fHalfDt;
  iNEMO_MatData(pMat_Fsys)[0][6] =  fSV[3] * fHalfDt;
  iNEMO_MatData(pMat_Fsys)[1][4] = -fSV[0] * fHalfDt;
  iNEMO_MatData(pMat_Fsys)[1][5] =  fSV[3] * fHalfDt;
  iNEMO_MatData(pMat_Fsys)[1][6] = -fSV[2] * fHalfDt;
  iNEMO_MatData(pMat_Fsys)[2][4] = -fSV[3] * fHalfDt;
  iNEMO_MatData(pMat_Fsys)[2][5] = -fSV[0] * fHalfDt;
  iNEMO_MatData(pMat_Fsys)[2][6] =  fSV[1] * fHalfDt;
  iNEMO_MatData(pMat_Fsys)[3][4] =  fSV[2] * fHalfDt;
  iNEMO_MatData(pMat_Fsys)[3][5] = -fSV[1] * fHalfDt;
  iNEMO_MatData(pMat_Fsys)[3][6] = -fSV[0] * fHalfDt;

  /*
  ***********************************************************************
  Extended Kalman Filter: Prediction Step
  ***********************************************************************
  */

  /* Update Quaternion with the new gyroscope measurements */
  fSVnorm[0] = fSV[0] - (fGyroX * fSV[1]) - (fGyroY * fSV[2]) - (fGyroZ * fSV[3]);
  fSVnorm[1] = fSV[1] + (fGyroX * fSV[0]) - (fGyroY * fSV[3]) + (fGyroZ * fSV[2]);
  fSVnorm[2] = fSV[2] + (fGyroX * fSV[3]) + (fGyroY * fSV[0]) - (fGyroZ * fSV[1]);
  fSVnorm[3] = fSV[3] - (fGyroX * fSV[2]) + (fGyroY * fSV[1]) + (fGyroZ * fSV[0]);

  for (i=0; i < 4; ++i)
    fSV[i] = fSVnorm[i];

  /*  P = F * P F' + Q  */
  pMat_Pnew = iNEMO_PropagateP(pMat_P, pMat_Fsys, pMat_Q, pMat_Pnew);

  /* Copy the new P in P */
  iNEMO_fMatCopy(pMat_Pnew, pMat_P);

  /*
  ***********************************************************************
  Extended Kalman Filter: Correction Step for tilting m_angle
  ***********************************************************************
  Perform the Correction any two updates
   */

  if(iNEMO_CNT % 2 == 0)
  {
    /* Assing the non-linear function h, it relates the State Variables with
    the measurements */
    fH[0] = -2.0*fAccRef * (fSV[1] * fSV[3] - fSV[0] * fSV[2]);
    fH[1] = -2.0*fAccRef * (fSV[0] * fSV[1] + fSV[2] * fSV[3]);
    fH[2] = -fAccRef  * (fSV[0] * fSV[0] - fSV[1] * fSV[1] - fSV[2] * fSV[2] + fSV[3] * fSV[3]);

    /* Its Jacobian matrix */
    iNEMO_MatData(pMat_Hja)[0][0] =  2.0*fAccRef * fSV[2];
    iNEMO_MatData(pMat_Hja)[0][1] = -2.0*fAccRef * fSV[3];
    iNEMO_MatData(pMat_Hja)[0][2] =  2.0*fAccRef * fSV[0];
    iNEMO_MatData(pMat_Hja)[0][3] = -2.0*fAccRef * fSV[1];

    iNEMO_MatData(pMat_Hja)[1][0] = -2.0*fAccRef * fSV[1];
    iNEMO_MatData(pMat_Hja)[1][1] = -2.0*fAccRef * fSV[0];
    iNEMO_MatData(pMat_Hja)[1][2] = -2.0*fAccRef * fSV[3];
    iNEMO_MatData(pMat_Hja)[1][3] = -2.0*fAccRef * fSV[2];

    iNEMO_MatData(pMat_Hja)[2][0] = -2.0*fAccRef * fSV[0];
    iNEMO_MatData(pMat_Hja)[2][1] =  2.0*fAccRef * fSV[1];
    iNEMO_MatData(pMat_Hja)[2][2] =  2.0*fAccRef * fSV[2];
    iNEMO_MatData(pMat_Hja)[2][3] = -2.0*fAccRef * fSV[3];

    /* Ka = P * Hja' * (Hja * P * Hja' + Ra)^-1  */
    pMat_Ka = iNEMO_CalculateK(pMat_P, pMat_Hja, pMat_Ra, pMat_Ka);

    /* Update State Vector */
    for (i=0; i < 7; ++i)
      fSV[i] += iNEMO_MatData(pMat_Ka)[i][0] * (iNEMO_AccX - fH[0]) +
        iNEMO_MatData(pMat_Ka)[i][1] * (iNEMO_AccY - fH[1]) +
          iNEMO_MatData(pMat_Ka)[i][2] * (iNEMO_AccZ - fH[2]);

    /* P = (I - Ka*Hja)*P */
    pMat_Pnew = iNEMO_UpdateP(pMat_P, pMat_Ka, pMat_Hja, pMat_Pnew);
    /* Copy the new P in P */
    iNEMO_fMatCopy(pMat_Pnew, pMat_P);
 } /* endif CNT /2 */

  /*
  ***********************************************************************
  Extended Kalman Filter: Correction Step for heading m_angle
  ***********************************************************************
  Perform the Correction any two updates
  */
  if(iNEMO_CNT % 2  == 0)
  {
    /* Yaw Correction */

    /* Assing the non-linear function h, it relates the State Variables with
    the measurements */
    fH[0] =         fMagRef * (fSV[0]*fSV[0] + fSV[1]*fSV[1] - fSV[2]*fSV[2] - fSV[3]*fSV[3]);
    fH[1] =  2.0f * fMagRef * (fSV[1]*fSV[2] - fSV[3]*fSV[0]);
    fH[2] =  2.0f * fMagRef * (fSV[1]*fSV[3] + fSV[2]*fSV[0]);

    /* Calculate the relative Jacobian Matrix */
    iNEMO_MatData(pMat_Hjm)[0][0] =   2.0f * fMagRef * fSV[0];
    iNEMO_MatData(pMat_Hjm)[0][1] =   2.0f * fMagRef * fSV[1];
    iNEMO_MatData(pMat_Hjm)[0][2] = - 2.0f * fMagRef * fSV[2];
    iNEMO_MatData(pMat_Hjm)[0][3] = - 2.0f * fMagRef * fSV[3];
    iNEMO_MatData(pMat_Hjm)[1][0] = - 2.0f * fMagRef * fSV[3];
    iNEMO_MatData(pMat_Hjm)[1][1] =   2.0f * fMagRef * fSV[2];
    iNEMO_MatData(pMat_Hjm)[1][2] =   2.0f * fMagRef * fSV[1];
    iNEMO_MatData(pMat_Hjm)[1][3] = - 2.0f * fMagRef * fSV[0];
    iNEMO_MatData(pMat_Hjm)[2][0] =   2.0f * fMagRef * fSV[2];
    iNEMO_MatData(pMat_Hjm)[2][1] =   2.0f * fMagRef * fSV[3];
    iNEMO_MatData(pMat_Hjm)[2][2] =   2.0f * fMagRef * fSV[0];
    iNEMO_MatData(pMat_Hjm)[2][3] =   2.0f * fMagRef * fSV[1];

    /* Km = P * Hjm' * (Hjm * P * Hjm' + Rm)^-1  */
    pMat_Km = iNEMO_CalculateK(pMat_P, pMat_Hjm, pMat_Rm, pMat_Km);

    /* State Vector Update */
    for (i=0; i < 7; ++i)
      fSV[i] += iNEMO_MatData(pMat_Km)[i][0] * (iNEMO_MagX - fH[0]) +
        iNEMO_MatData(pMat_Km)[i][1] * (iNEMO_MagY - fH[1]) +
          iNEMO_MatData(pMat_Km)[i][2] * (iNEMO_MagZ - fH[2]);

    /* Update of Error Covariance Matrix */
    /* P = (I - Km*Hjm*P */
    pMat_Pnew = iNEMO_UpdateP(pMat_P, pMat_Km, pMat_Hjm, pMat_Pnew);
    /* Copy the new P in P */
    iNEMO_fMatCopy(pMat_Pnew, pMat_P);

  } /* End of Yaw Correction */

  /* Rescale quaternion to have module 1 */
  fTempNorm = 1.0f / sqrt(fSV[0]*fSV[0] + fSV[1]*fSV[1] + fSV[2]*fSV[2] + fSV[3]*fSV[3]);
  for (i = 0; i < 4; ++i)
  {
     fSV[i] = fTempNorm * fSV[i];
     pQuat[0][i]=fSV[i];
  }

  /* Update the RPY angles */
  iNEMO_Roll(pAngle)  = atan2(2.0f*(fSV[0]*fSV[1] + fSV[2]*fSV[3]),
                              1.0f - 2.0f*(fSV[1]*fSV[1] + fSV[2]*fSV[2]));

  iNEMO_Pitch(pAngle) = asin( -2.0f*(fSV[1]*fSV[3] - fSV[0]*fSV[2]));

  iNEMO_Yaw(pAngle)   = atan2(2.0f*(fSV[1]*fSV[2] + fSV[0]*fSV[3]),
                               (1.0f - 2.0f*(fSV[2]*fSV[2] + fSV[3]*fSV[3])));


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

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/


