/**
  ******************************************************************************
  * @file    sha256_low_level.h
  * @author  MCD Application Team
  * @version V2.0.6
  * @date    25-June-2013
  * @brief   SHA-256 core functions
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SHA256_LOW_LEVEL_H__
#define __SHA256_LOW_LEVEL_H__

#ifdef __cplusplus
 extern "C" {
#endif


/* Exported functions ------------------------------------------------------- */ 
/* SHA-256 Initialize new context */
void SHA256Init(SHA256ctx_stt* P_pSHA256ctx);
/* SHA-256 finalization function */
void SHA256Final(SHA256ctx_stt* P_pSHA256ctx, uint8_t *P_pDigest);


#ifdef __cplusplus
}
#endif

#endif  /*__SHA256_LOW_LEVEL_H__*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
