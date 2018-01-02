#include <stdio.h>
#include <stdio.h>
#include <string.h>


#include "libMCUAuthenticate.h"
#include "us_mcu_transfer.h"
#define US_MCU
//#define US_HOST
#ifdef US_MCU
#include "us_aes_func.h"	//AES 
#else
#include "openssl/include/openssl/aes.h"
#include "openssl/include/openssl/rand.h"
#include "openssl/include/openssl/rsa.h"
#endif

#define RSA_KEY_BIT_SIZE	128
#define MCU_ID_SIZE			12
#define TOKEN_SIZE			4
#define ENCODE_SIZE			(MCU_ID_SIZE + RSA_KEY_BIT_SIZE/8 + TOKEN_SIZE)


//128bits key.
/* Key to be used for AES encryption/decryption */
unsigned char Host_Key[16] =
  {
    0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
    0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c
  };

/* Initialization Vector, used only in non-ECB modes */
unsigned char Host_IV[16] =
  {
    0xf0 , 0xf1 , 0xf2 , 0xf3 , 0xf4 , 0xf5 , 0xf6 , 0xf7,
    0xf8 , 0xf9 , 0xfa , 0xfb , 0xfc , 0xfd , 0xfe , 0xff
  };

#if 0
static void hexdump(const unsigned char *data, int length)
{
	int n = 0;

	for(n = 0; n < length; n++){
		//printf("0x%02x ", data[n]);
	}
	//printf("\n");
}
#endif

#ifdef US_HOST

int MCUAuthenticateGetToken(unsigned char* tokenData)
{
	int ret = 0;

	if(tokenData == NULL){
		ret = -1;
	}
	
	if(!ret){
		RAND_pseudo_bytes(tokenData, TOKEN_SIZE);
		ret = TOKEN_SIZE;
	}

	return ret;
}



int MCUAuthenticateDecode(const unsigned char* inputData, const int intputDataSize,
								unsigned char* outputData, int *outputDataSize
								)
{

	int ret = 0;
	int nr_of_bits = 8 * sizeof(Host_Key);
	int nr_of_bytes = intputDataSize;
	
	AES_KEY key;
	
	unsigned char   checktext[64] = {0};
	unsigned char	saved_iv[AES_BLOCK_SIZE] = {0};

	if(inputData == NULL || intputDataSize == 0 ||
		outputData == NULL || outputDataSize == NULL){
		ret = -1;
	}
	
	/*Make Decrypt Key*/
	if(!ret){
		AES_set_decrypt_key(Host_Key, nr_of_bits, &key);
	}

	/*Get Host IV*/
	if(!ret){
		memcpy(saved_iv, Host_IV, AES_BLOCK_SIZE);
	}

	/*Decrypt*/
	if(!ret){
		AES_cbc_encrypt(inputData, checktext, nr_of_bytes, &key, saved_iv, AES_DECRYPT);
	}
	//hexdump(checktext, sizeof(checktext));
	
	/*Separate Data and Token Data*/
	if(!ret){
		memcpy(outputData, checktext, intputDataSize);

		*outputDataSize = intputDataSize;
	}

	if(ret < 0){
		us_dev_error(MCU_DEV_ID, (unsigned char *)__func__, strlen(__func__) + 1, ret);
	}
	return ret;
}

unsigned char n_b[] = {0x9e, 0x98, 0xc3, 0x70, 0xa7, 0x08, 0x4b, 0x95, 0xd9, 0xf6, 0x91, 0x36, 0x5c, 0x8b, 0x0e, 0xaf};
unsigned char e_b[] = {0x01, 0x00, 0x01};
unsigned char d_b[] = {0x62, 0x2c, 0xdc, 0xa7, 0x0a, 0xaf, 0x06, 0xe2, 0x35, 0xf6, 0xc3, 0x02, 0xae, 0xd9, 0x08, 0x21};

int MCUAuthenticateVerify(const unsigned char* inputData,const unsigned char* tokenData,
								unsigned char* outputData, int *outputDataSize )
{
	int ret = 0, flen = 0;
	unsigned char  rsa_decoder[24] = {0};
	unsigned char de_token[10] = {0};
	unsigned char data[64] = {0}, mcu_id[MCU_ID_SIZE] = {0}, mcu_id_en[24] = {0};
	int de_length = 0, data_length = 0;

	RSA *pubrsa = RSA_new();
	
	pubrsa->n = BN_bin2bn(n_b, sizeof(n_b), NULL);
	pubrsa->e = BN_bin2bn(e_b, sizeof(e_b), NULL);

	MCUAuthenticateDecode(inputData, ENCODE_SIZE, data, &data_length);
	

	//printf("Begin Verify...\n");

	if(inputData == NULL ||tokenData == NULL || outputData == NULL || outputDataSize == NULL){
		ret = -1;
	}

	if(!ret){
		memcpy(mcu_id, data, MCU_ID_SIZE);
		
		memcpy(mcu_id_en, data + MCU_ID_SIZE, RSA_KEY_BIT_SIZE/8);
		memcpy(de_token, data + MCU_ID_SIZE + RSA_KEY_BIT_SIZE/8, TOKEN_SIZE);
	}
	

	/*Get RSA Decoder Size*/
	if(!ret){
		flen = RSA_size(pubrsa);
	}
	/*RSA Decrypt*/
	if(!ret){
		de_length = RSA_public_decrypt(flen, mcu_id_en, rsa_decoder,  pubrsa, RSA_NO_PADDING);
		if(de_length < 0){
			ret = -2;
		}
	}
#if 0
	if(0){
		printf("======\n");
		hexdump(inputData, ENCODE_SIZE);
		printf("======\n");
		hexdump(data, data_length);
		printf("======\n");
		hexdump(mcu_id, MCU_ID_SIZE);
		printf("======\n");
		hexdump(mcu_id_en, RSA_KEY_BIT_SIZE/8);
		printf("======\n");
		hexdump(de_token, TOKEN_SIZE);
		printf("======\n");
		hexdump(rsa_decoder, MCU_ID_SIZE);
	}
#endif

	if(!ret){
		if((strncmp(mcu_id, rsa_decoder, MCU_ID_SIZE) == 0) && 
			(strncmp(de_token, tokenData, TOKEN_SIZE) == 0)){
			//printf("|---Verify Successful!---|\n");
		}else{
			//printf("|---Verify Error!---|\n");
			ret = -3;
		}
	}

	if(!ret){
		memcpy(outputData, mcu_id, MCU_ID_SIZE);
		memcpy(outputData + MCU_ID_SIZE, rsa_decoder, MCU_ID_SIZE);
		memcpy(outputData + MCU_ID_SIZE + MCU_ID_SIZE, de_token, TOKEN_SIZE);


		*outputDataSize = MCU_ID_SIZE + MCU_ID_SIZE + TOKEN_SIZE;
	}
	
	if(ret < 0){
		us_dev_error(MCU_RSA_LOCK, (unsigned char *)__func__, strlen(__func__) + 1, ret);
	}

	return ret;
}

int MCU_OpenSSL_API(const unsigned char* inputData,
								unsigned char* outputData, unsigned int *outputDataSize )
{
	int ret = 0;
	int nr_of_bits = 8 * sizeof(Host_Key);
	int nr_of_bytes = ENCODE_SIZE;

	unsigned char ciphertext[64] = {0}, out_data[64] = {0};
	unsigned char saved_iv[AES_BLOCK_SIZE] = {0};
	
	AES_KEY key;

	if(inputData == NULL || outputData == NULL || outputDataSize == NULL){
		ret = -1;
	}

	/*Connect InputData and tokenData*/
	if(!ret){
		memcpy(ciphertext, inputData, ENCODE_SIZE);
	}
	//hexdump(inputData, intputDataSize);
	
	/*Make Encrypt Key*/
	if(!ret){
		AES_set_encrypt_key(Host_Key, nr_of_bits, &key);
	}
	
	/*Get Host IV Data*/
	if(!ret){
		memcpy(saved_iv, Host_IV, AES_BLOCK_SIZE);
	}
	
	/*Enctypt*/
	if(!ret){
		AES_cbc_encrypt(ciphertext, out_data, nr_of_bytes, &key, saved_iv, AES_ENCRYPT);
		memcpy(outputData, out_data, ENCODE_SIZE);
		//hexdump(ciphertext, ENCODE_SIZE);
		*outputDataSize = ENCODE_SIZE;
	}

	if(ret < 0){
		us_dev_error(MCU_DEV_ID, (unsigned char *)__func__, strlen(__func__) + 1, ret);
	}
	
	return ret;
}
#endif

int MCUAuthenticateEncode(const unsigned char* inputData,
								unsigned char* outputData, unsigned int *outputDataSize )
{
	int ret = 0;

#ifdef US_MCU
	ret = STM32_AES_CBC_Encrypt( (unsigned char *) inputData, ENCODE_SIZE, Host_Key, Host_IV, sizeof(Host_IV), outputData,
						   outputDataSize);
#else
	MCU_OpenSSL_API(inputData, outputData, outputDataSize);
#endif

	if(ret < 0){
		us_dev_error(MCU_DEV_ID, (unsigned char *)__func__, strlen(__func__) + 1, ret);
	}	

	return ret;
}

#if 0

int test_encrypt(const unsigned char* inputData,const int intputDataSize,
								const unsigned char* tokenData,const int tokenDataSize, unsigned char *outputData, int * outputDataSize)
{
	int ret = 0, flen = 0;
	unsigned char rsa_data[512] = {0}, rsa_encoder[512] = {0}, rsa_decoder[512] = {0};
	int en_length = 0, de_length = 0;
	
	RSA *prirsa = RSA_new();
	
	prirsa->n = BN_bin2bn(n_b, n_size, NULL);
	prirsa->e = BN_bin2bn(e_b, b_size, NULL);
	prirsa->d = BN_bin2bn(d_b, d_size, NULL);

	printf("make RSA Key\n");

	memcpy(rsa_data, inputData, intputDataSize);
	strncat(rsa_data, tokenData, tokenDataSize);

	hexdump(rsa_data, intputDataSize + tokenDataSize);

	
	printf(" Begin encrypt...\n");

	flen = RSA_size(prirsa);
	printf("flen = %d\n", flen);
	
	en_length = RSA_private_encrypt(flen, rsa_data, rsa_encoder, prirsa, RSA_NO_PADDING);
	printf("en length:%d\n", en_length);
	
	hexdump(rsa_encoder, en_length);

	memcpy(outputData, rsa_encoder, en_length);

	*outputDataSize = en_length;
	
#if 0
	printf("	Begin decrypt...\n");
	
	flen = RSA_size(pub);
	printf("flen = %d\n", flen);

	de_length = RSA_public_decrypt(flen, rsa_encoder, rsa_decoder,  pub, RSA_NO_PADDING);
	 
	printf("de length:%d\n", de_length);
	hexdump(rsa_decoder, de_length);

#endif	

	RSA_free(prirsa);
	return ret;
}

#endif
