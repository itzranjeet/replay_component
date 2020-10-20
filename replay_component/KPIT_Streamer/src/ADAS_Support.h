/*******************************************************************************/
/* Copyright(c) KPIT Technologies, Pune, India.                                */
/*                                                                             */
/*******************************************************************************/

/*******************************************************************************/
/*                                                                             */
/* Project      : ADAS														   */
/*                                                                             */
/* Module       : Applicable for All Modules                                   */
/*                                                                             */
/* File name    : PlayerAndTagger.cpp.c										   */
/*                                                                             */
/* Author       : Rakesh Shroff & Rohan Adivarekar                             */
/*                                                                             */
/* Description  : This file contains source code for Video Player & tagger.    */
/*                Various user controls such as Start/Stop Forword/Backword    */
/*                Playback, restart playback, playback of specific frame 	   */
/*				  number ,tagging of videos etc. Supported input file types	   */
/*                are: iz,bmp,png,jpg,avi									   */
/*******************************************************************************/
#ifndef ADAS_PTC_SUPPORT_H
#define ADAS_PTC_SUPPORT_H

#ifdef __cplusplus
extern "C" {
#endif


typedef unsigned int        boolean_t;
typedef unsigned char       uint8_t;
typedef unsigned short      uint16_t;
typedef unsigned int        uint32_t;
//typedef unsigned long long  uint64_t;

typedef signed char         sint8_t;
typedef signed short        sint16_t;
typedef signed int          sint32_t;
typedef signed long long    sint64_t;

typedef float               float32_t;
typedef long double         float64_t;


//#include "./ADAS_Typedef.h"
#if 1
/* FPS MACROS */
#define FRAMECOUNTLIMIT 3000
#define SLOWCHANGERATE 1
#define FASTCHANGERATE 10

//#define SHOW_DEBUG_POINTS

#ifdef SHOW_DEBUG_POINTS
#define DEBUG_POINT() printf("Debug: %s:%d\n",__FILE__,__LINE__)
#else
#define DEBUG_POINT() 
#endif

#if 0
#define PRINT_STRING(x) printf("%s : %s\n",#x,x)
#define PRINT_DECIMAL(x) printf("%s : %d\n",#x,x)
#define PRINT_LONG(x) printf("%s : %ld\n",#x,x)
#endif

#define PRINTS(x) printf("%s : %s\n",#x,x)
#define PRINTD(x) printf("%s : %d\n",#x,x)
#define PRINTF(x) printf("%s : %f\n",#x,x)
#define PRINTL(x) printf("%s : %ld\n",#x,x)
#define PRINTLL(x) printf("%s : %lld\n",#x,x)

#define PROCESS_TOTAL_FPS 0
#define FPS_WINDOW_SIZE (20)

#if (1 == PROCESS_TOTAL_FPS)

#if 0
#define CREATE_LOCAL_FPS \
    int FPS_AveragingCount = FPS_WINDOW_SIZE;\
    int FPS_AveragingIndex = 1;\
    long FPS_TimeStampNew = 0;\
    long FPS_TimeStampOld = 0;\
    double FPS_Value = 0;
#endif



#define CREATE_LOCAL_FPS int FPS_AveragingCount = FPS_WINDOW_SIZE;\ 
		int FPS_AveragingIndex = 1;\
		long FPS_TimeStampNew = 0;\
		long FPS_TimeStampOld = 0;\
		double FPS_Value = 0;


#define PRINT_FPS(x,y) \
	if(1 == y)\
	{\
		if(0==FPS_TimeStampNew) FPS_TimeStampNew = GetTickCount(); \
		\
		if(FPS_AveragingCount <= FPS_AveragingIndex)\
		{\
			FPS_TimeStampOld = FPS_TimeStampNew;\
			FPS_TimeStampNew = GetTickCount();\
			FPS_Value = FPS_TimeStampNew - FPS_TimeStampOld;\
			FPS_Value = FPS_Value / 1000;\
			FPS_Value = FPS_Value / FPS_AveragingCount;\
			FPS_Value = 1 / FPS_Value;\
			printf("%s = %f\n",x,FPS_Value);\
			FPS_AveragingIndex = 0;\
		}\
		FPS_AveragingIndex++; \
	}
#else

#define CREATE_LOCAL_FPS int temp;
#define PRINT_FPS(x,y)

#endif

//#include <direct.h> 
#define CREATE_FOLDER(x) \
    if(0 != _mkdir(x))\
    {\
		switch(errno)\
		{\
		case EEXIST:\
			printf("Folder Existing: ""%s""\n",x);\
			break;\
		case ENOENT:\
			printf("Cannot create folder: ""%s""\n",x);\
			break;\
		}\
	 }\
	else\
		{\
			printf("Folder Created: ""%s""\n",x);\
		}
#if 0
/*******************************************************************************/
/* Function Name   : GetFileNameParameters									   */
/* Input Arguments : const char * const FilePath - Complete File Path		   */
/* Return Value    : Structure Pointer of type st_file_name_parameters_t		   */
/* Description     : Using the command line argument(Complete File Path),this  */
/*					 function extracts necessary information such as Video 	   */
/*					 Name, Tag folder Path.									   */
/*******************************************************************************/

extern st_file_name_parameters_t * GetFileNameParameters(const char * const FilePath,st_file_name_parameters_t * stptr_FileParameters_arg);

extern int CheckFileNameIsStandard (const char* const FileName);

extern en_image_save_type_t FindTypeFromExtension(const char * const p_chInputExtension);
extern void ADAS_InitFileParameters(st_file_name_parameters_t * p_stFileParameters);
extern void ADAS_ReleaseFileParameters(st_file_name_parameters_t * p_stFileParameters);
#endif

typedef enum en_image_save_type_t_
{
	EN_SAVE_BMP = 0,
	EN_SAVE_JPG = 1,
	EN_SAVE_JPEG = 2,
	EN_SAVE_RAW = 3,
	EN_SAVE_RAW_RCCC16 = 4,
	EN_SAVE_RAW_YUV = 5,
	EN_SAVE_RAW_YUV420_YUYV = 6,
	EN_SAVE_RAW_YUV420_UYVY = 7,
	EN_SAVE_RAW_GREY = 8,
	EN_SAVE_PNG = 9,
	EN_SAVE_AVI = 10,
	EN_SAVE_IZ = 11,
	EN_SAVE_IYUV = 12,
	EN_SAVE_IGREY = 13,
	EN_SAVE_IRCCC = 14,
	EN_SAVE_7Z = 15,
	EN_SAVE_7Z_IYUV = 16,
	EN_SAVE_7Z_IGREY = 17,
	EN_SAVE_7Z_IZ = 18,
	EN_SAVE_7Z_IR = 19,
	EN_SAVE_7Z_BMP = 20,
	EN_SAVE_7Z_PNG = 21,
	EN_SAVE_7Z_JPEG = 22,
	EN_SAVE_7Z_JPG = 23,
	EN_SAVE_CONFIG = 24,
	EN_SAVE_ADAS_BATCH_FILE = 25,
	EN_SAVE_CSV = 26,
	EN_SAVE_CAN = 27,
	EN_SAVE_TXT = 28,
	EN_SAVE_LST = 29,
	EN_UNKNOWN_IMAGE_SAVE_TYPE = 30
}en_image_save_type_t;

extern en_image_save_type_t FindTypeFromFileName(const char * const Filename,char * const p_chExtension);

/***********************
* Module		: YUV422_UYVYtoRGB888_BGR()
* Description	: convert yuv422 interleaved to RGB interleaved	
***********************/
extern void YUV422_UYVYtoRGB888_BGR(const unsigned char *ptr_u8YUV, unsigned char *ptr_u8BGR,unsigned int u32ImageSize);

extern void YUV422_UYVYtoRGB888_GBR(const unsigned char *ptr_u8YUV, unsigned char *ptr_u8BGR,unsigned int u32ImageSize);

/***********************
* Module		: RGB888_BGRtoYUV422_UYVY()
* Description	: convert RGB interleaved to YUV 422 interleaved	
***********************/
extern void RGB888_BGRtoYUV422_UYVY(const unsigned char *ptr_u8BGR, unsigned char *ptr_u8YUV,unsigned int u32ImageSize);

/***********************
* Module		: RGB888_BGRtoYUV422_UYVY_Fixed()
* Description	: convert RGB interleaved to YUV 422 interleaved	
***********************/
extern void RGB888_BGRtoYUV422_UYVY_Fixed(const unsigned char *ptr_u8BGR, unsigned char *ptr_u8YUV,unsigned int u32ImageSize);

/***********************
* Module		: RGB888_BGRtoYUV422_YUYV()
* Description	: convert RGB interleaved to YUV 422 interleaved	
***********************/
extern void RGB888_BGRtoYUV422_YUYV(const unsigned char *ptr_u8BGR, unsigned char *ptr_u8YUV,unsigned int u32ImageSize);


extern void YUV422toRGB888(const unsigned char *ptr_u8YUVIntImg, 
				unsigned int u32ImageSize, unsigned char *ptr_Image);

extern unsigned char * YUV444toYUV422_UYVY(unsigned char * p_YUVData,unsigned int intImageSize);

extern void GetNameFromCompletePath(const char * const CompletePath, char * DestinationFilename);

unsigned char * YUV422_UYVYtoYUV444(unsigned char * p_UYVYData,unsigned int intImageSize);
unsigned char * YUV422_YUYVtoYUV444(unsigned char * p_UYVYData,unsigned int intImageSize);

void YUV422_YUYVtoRGB888_BGR(const unsigned char *ptr_u8YUV, unsigned char *ptr_u8BGR,
			unsigned int u32ImageSize);

void ADAS_YUV444_YUVtoYUV422_YUYV(unsigned char * InputYUV,unsigned char * OutputYUYV,unsigned int ImageSize);

void YUV422_YUYVtoY(const unsigned char *ptr_u8YUV, unsigned char *ptr_u8Y,
			unsigned int u32ImageSize);
void YUV422_UYVYtoY(const unsigned char *ptr_u8YUV, unsigned char *ptr_u8Y,
			unsigned int u32ImageSize);

void YUV444_YUVtoRGB888_BGR(const unsigned char *ptr_u8YUV, unsigned char *ptr_u8BGR,
			unsigned int u32ImageSize);
			
void YUYVtoUYVY(const unsigned char *ptr_u8YUYV, unsigned char *ptr_u8UYVY,unsigned int u32ImageSize);	

void RCCC16ToGrey8(unsigned char* pch_RCCC16,unsigned char* pch_Grey8,uint32_t u32_ImageSize);
#if 0
char * ADAS_StringClone(const char * const p_chInputString);

unsigned int ADAS_ParseYesNo(char *chString);

unsigned int ADAS_ParseTrueFalse(char *chString);

void ADAS_CreateDir(char const * const Path);

extern void ADAS_CreateDirOfFile(char const * const FilePath);

extern void SleepInSeconds(uint32_t u32Seconds);
extern void ExitInSeconds(uint32_t u32Seconds);

extern st_init_thread_arguments_t * ADAS_CreateThreadArgsPipe(st_init_thread_arguments_t * pst_init_thread_arguments);
extern void ADAS_WritedataToThreadArgsStreamPipe(void * pv_Indata,st_init_thread_arguments_t * pst_init_thread_arguments,int DataSize);
extern void ADAS_ReleaseThreadArgsPipe(st_init_thread_arguments_t * pst_init_thread_arguments);
#endif

#endif
#endif

#ifdef __cplusplus
}
#endif
