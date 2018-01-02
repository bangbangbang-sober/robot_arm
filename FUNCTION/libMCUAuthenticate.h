#ifndef LIBMCUAUTHENTICATE_H
#define LIBMCUAUTHENTICATE_H

#ifdef __cplusplus
extern "C" {
#endif


int MCUAuthenticateGetToken(unsigned char* tokenData);

int MCUAuthenticateEncode(const unsigned char* inputData,
								unsigned char* outputData, unsigned int *outputDataSize ); 

int MCUAuthenticateVerify(const unsigned char* inputData,const unsigned char* tokenData,
								unsigned char* outputData, int *outputDataSize );

#ifdef __cplusplus
} // extern "C"
#endif

#endif /*LIBMCUAUTHENTICATE*/


