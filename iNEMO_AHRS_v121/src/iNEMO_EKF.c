/**
*
* \file    iNEMO_EKF.c
* \author  ART Team IMS-Systems Lab
* \version V1.2.1  [FW v2.0.0]
* \date    07/07/2010
* \brief   Implementation file of Extended Kalman Filter Library of iNEMO
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

#include "iNEMO_EKF.h"

/** @addtogroup iNEMO_Engine_Lite        iNEMO Engine Lite
  * @{
  */

/**
 * @addtogroup iNEMO_EKF
 * @{
 */

/**
 * @addtogroup iNEMO_EKF_Functions
 * @{
 */


/**
* @brief Propagate the Error Covariance Matrix
* @param  pPoldMat  : old P Matrix  
* @param  pStateMat : the State Matrix 
* @param  pQMat     : the Q Covariance Matrix of noise on state 
* @param  pPnewMat  : the new P matrix 
* @retval Pointer to iNEMO_fMATRIX_TYPE
* @par Functions called:
* @ref iNEMO_fMatCreate
* @ref iNEMO_fMatMulMat
* @ref iNEMO_fMatMulMatMT
* @ref iNEMO_fMatAdd
* @ref iNEMO_fMatFree
* @details
* <b>Algorithm:</b> <br>
*    P = F * P F' + Q <br>
*     became <br>
*    pPnewMat = pStateMat * pPoldMat * pStateMat' + pQMat <br>
*
*  This operation is divided in steps: <br>
* 1)    pPnewMat = pStateMat * pPoldMat <br>
* 
* 2)    pPnewMat = pPnewMat * pStateMat' <br>
* 
* 3)    pPnewMat += Q <br>
*/

iNEMO_fMATRIX_TYPE* iNEMO_PropagateP(iNEMO_fMATRIX_TYPE* pPoldMat, 
                                    iNEMO_fMATRIX_TYPE* pStateMat, 
                                    iNEMO_fMATRIX_TYPE* pQMat, 
                                    iNEMO_fMATRIX_TYPE* pPnewMat)
{
  /* Internal Variables */
  iNEMO_fMATRIX_TYPE* pTmp;

  pTmp = iNEMO_fMatCreate(iNEMO_MatRow(pPoldMat), iNEMO_MatCol(pPoldMat));
  
  /* First Step: pTmp = pStateMat * pPoldMat */
  pTmp = iNEMO_fMatMulMat(pStateMat, pPoldMat, pTmp);

  /* Second Step: pPnewMat = pTmp * pStateMat' */
  pPnewMat = iNEMO_fMatMulMatMT(pTmp, pStateMat, pPnewMat);

  /* Third Step: pPnewMat += Q */
  pPnewMat = iNEMO_fMatAdd(pPnewMat, pQMat, pPnewMat);
  
  /* Delete the Temporary Matrix before to exit */
  iNEMO_fMatFree(pTmp);
  
  return pPnewMat;  
}


/**
* @brief Calculate the Kalman Gain
* @param pPMat  : the P matrix
* @param pHJMat : the HJ matrix
* @param pRMat  : the R matrix
* @param pKMat  : the K matrix
* @retval Pointer to iNEMO_fMATRIX_TYPE
* @par Functions called:
* @ref iNEMO_fMatCreate
* @ref iNEMO_fMatMulMatMT
* @ref iNEMO_fMatMulMat
* @ref iNEMO_fMatAdd
* @ref iNEMO_fMatInv
* @ref iNEMO_fMatFree
* @details
* <b>Algorithm:</b> <br>
*    K = P * Hj' * (Hj * P * Hj' + R)^-1 <br>
*     became <br>
*    pKMat = pPMat * pHJMat' * (pHJMat * pPMat * pHJMat' + pRMat)^-1 <br>
*
* 1) pTmp1 = pPMat * pHJMat' <br> 
*
* 2) pTmp2 = pHJMat * (pPMat * pHJMat') = pHJMat * pTmp1 <br>
*
* 3) pTmp2 = pTmp2 + pRMat <br>
*
* 4) pRInvMat = (pTmp2)^-1 <br>
*
* 5) pKMat = pTmp1 * pRInvMat <br>
*/

iNEMO_fMATRIX_TYPE* iNEMO_CalculateK(iNEMO_fMATRIX_TYPE* pPMat,
                               iNEMO_fMATRIX_TYPE* pHJMat,
                               iNEMO_fMATRIX_TYPE* pRMat,
                               iNEMO_fMATRIX_TYPE* pKMat)
{
  /* Internal Variables */
  iNEMO_fMATRIX_TYPE* pTmp1;
  iNEMO_fMATRIX_TYPE* pTmp2;
  iNEMO_fMATRIX_TYPE* pRInvMat;
  
  pTmp1 = iNEMO_fMatCreate(iNEMO_MatRow(pPMat), iNEMO_MatRow(pHJMat));
  pTmp2 = iNEMO_fMatCreate(iNEMO_MatRow(pHJMat), iNEMO_MatCol(pTmp1));  
  
  /* First Step: pTmp1 = pPMat * pHJMat' */
  pTmp1 = iNEMO_fMatMulMatMT(pPMat, pHJMat, pTmp1);
  
  /* Second Step: pTmp2 = pHJMat * (pPMat * pHJMat') = pHJMat * pTmp1 */
  pTmp2 = iNEMO_fMatMulMat(pHJMat, pTmp1, pTmp2);
  
  /* Third Step: pTmp2 = pTmp2 + pRMat */
  pTmp2 = iNEMO_fMatAdd(pTmp2, pRMat, pTmp2);
  
  /* Fourth Step: pRInvMat = (pTmp2)^-1 */
  pRInvMat = iNEMO_fMatCreate(iNEMO_MatRow(pTmp2), iNEMO_MatCol(pTmp2));
  
  pRInvMat = iNEMO_fMatInv(pTmp2, pRInvMat);
  
  /* Fifth Step: pKMat = pTmp1 * pRInvMat */
  pKMat = iNEMO_fMatMulMat(pTmp1, pRInvMat, pKMat);
  
  /* Delete the Temporary Matrix before to exit */
  iNEMO_fMatFree(pTmp1);
  iNEMO_fMatFree(pTmp2);  
  iNEMO_fMatFree(pRInvMat);  
  
  return pKMat;
  
}



/**
* @brief Update the Error Covariance Matrix
* @param pPoldMat : old P matrix 
* @param pKMat    : the Gain Matrix 
* @param pHJMat   : the Jacobian Matrix  
* @param pPnewMat : the new P matrix 
* @retval Pointer to iNEMO_fMATRIX_TYPE
* @par Functions called:
* @ref iNEMO_fMatCreateZero
* @ref iNEMO_fMatCreateUnit
* @ref iNEMO_fMatMulMat
* @ref iNEMO_fMatSub
* @ref iNEMO_fMatFree
* @details
* <b>Algorithm:</b> <br>
*    P = (I - K * Hj) * P <br>
*     became <br>
*    pKMat = (I - pKMat * pHJMat) * pPoldMat <br>
*
*
*    where pHJMat has this structure <br>
*     | * * * *  | 0 0 0 |  <br>
*     | * * * *  | 0 0 0 |  <br>
*     | * * * *  | 0 0 0 |  <br>
*
*/

iNEMO_fMATRIX_TYPE* iNEMO_UpdateP(iNEMO_fMATRIX_TYPE* pPoldMat,
                            iNEMO_fMATRIX_TYPE* pKMat,
                            iNEMO_fMATRIX_TYPE* pHJMat, 
                            iNEMO_fMATRIX_TYPE* pPnewMat)
{
  iNEMO_fMATRIX_TYPE* pTmp;
  iNEMO_fMATRIX_TYPE* pI;
   
  pTmp = iNEMO_fMatCreateZero(iNEMO_MatRow(pPoldMat), iNEMO_MatCol(pPoldMat));
  pI = iNEMO_fMatCreateUnit(iNEMO_MatRow(pPoldMat), iNEMO_MatCol(pPoldMat));

  // First Step: pTmp = (pKMat * pHJMat)
  pTmp = iNEMO_fMatMulMat(pKMat, pHJMat, pTmp); 

  // Second Step: pTmp = I - pTmp
  pTmp = iNEMO_fMatSub(pI, pTmp, pTmp);
  
  // Third Step: pPnewMat = pTmp * pPoldMat
  pPnewMat = iNEMO_fMatMulMat(pTmp, pPoldMat, pPnewMat);
  
  iNEMO_fMatFree(pTmp); 
  iNEMO_fMatFree(pI);
  
  return (pPnewMat);  
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
