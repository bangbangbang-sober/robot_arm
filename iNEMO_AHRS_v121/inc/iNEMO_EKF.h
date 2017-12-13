/**
*
* \file    iNEMO_EKF.h
* \author  ART Team IMS-Systems Lab
* \version V1.2.1 [FW v2.0.0]
* \date    13/09/2010
* \brief   Header file of Extended Kalman Filter Library of iNEMO
*
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


#ifndef __INEMO_EKF
#define __INEMO_EKF

#include "iNEMO_math.h"

/** @addtogroup iNEMO_Engine_Lite        
  * @{
  */

/* Exported functions ------------------------------------------------------- */

/** @defgroup iNEMO_EKF        iNEMO EKF
* @{
*/

/** @defgroup iNEMO_EKF_Functions        iNEMO EKF Functions
* @{
*/
iNEMO_fMATRIX_TYPE* iNEMO_PropagateP(iNEMO_fMATRIX_TYPE* pPoldMat, 
                                    iNEMO_fMATRIX_TYPE* pStateMat, 
                                    iNEMO_fMATRIX_TYPE* pQMat, 
                                    iNEMO_fMATRIX_TYPE* pPnewMat);

iNEMO_fMATRIX_TYPE* iNEMO_CalculateK(iNEMO_fMATRIX_TYPE* pPMat,
                               iNEMO_fMATRIX_TYPE* pHJMat,
                               iNEMO_fMATRIX_TYPE* pRMat,
                               iNEMO_fMATRIX_TYPE* pKMat);

iNEMO_fMATRIX_TYPE* iNEMO_UpdateP(iNEMO_fMATRIX_TYPE* pPoldMat,
                            iNEMO_fMATRIX_TYPE* pKMat,
                            iNEMO_fMATRIX_TYPE* pHJMat, 
                            iNEMO_fMATRIX_TYPE* pPnewMat);

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

#endif

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
