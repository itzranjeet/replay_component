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
/* Author       : Rakesh Shroff					                               */
/*                                                                             */
/* Description  : This file contains source code for Video Player & tagger.    */
/*                Various user controls such as Start/Stop Forword/Backword    */
/*                Playback, restart playback, playback of specific frame 	   */
/*				  number ,tagging of videos etc. Supported input file types	   */
/*                are: iz,bmp,png,jpg,avi									   */
/*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <conio.h>
//#include <windows.h>


//#include "ADAS_Typedef.h"
#include "replay_component/ADAS_Support.h"
#include <string.h>
#if 0
void GetNameFromCompletePath(const char * const CompletePath, char * DestinationFilename);

void GetNameFromCompletePath(const char * const CompletePath, char * DestinationFilename)
{
	char * p_TempPointer = NULL;
	unsigned short u16_PathLength;
	
	
	u16_PathLength = strlen(CompletePath);
	p_TempPointer  = CompletePath + u16_PathLength;
    
	while( p_TempPointer > CompletePath)
    {
        if('/' == *(p_TempPointer - 1))
        {
            break;
        }else if('\\' == *(p_TempPointer-1))
        {
            break;
        }

        p_TempPointer --;
    }
	
	strcpy(DestinationFilename,p_TempPointer);

}

/*******************************************************************************/
/* Function Name   : GetFileNameParameters									   */
/* Input Arguments : const char * const FilePath - Complete File Path		   */
/* Return Value    : Structure Pointer of type st_file_name_parameters_t		   */
/* Description     : Using the command line argument(Complete File Path),this  */
/*					 function extracts necessary information such as Video 	   */
/*					 Name, Tag folder Path.									   */
/*******************************************************************************/

st_file_name_parameters_t * GetFileNameParameters(const char * const FilePath,st_file_name_parameters_t * stptr_FileParameters_arg)
{
    int u32_PathLength;
    char * p_TempPointer;
	int u32_ReturnValue = 0;
    st_file_name_parameters_t * stptr_FileParameters;
	
    /* Copy path */
	
	PRINTS(FilePath);

	if(NULL == stptr_FileParameters_arg)
	{
		stptr_FileParameters = (st_file_name_parameters_t *)malloc(sizeof(st_file_name_parameters_t));
	}
	else
	{
		stptr_FileParameters = stptr_FileParameters_arg;
	}

	ADAS_InitFileParameters(stptr_FileParameters);
		
    u32_PathLength = strlen(FilePath);
    stptr_FileParameters->pch_CompletePath = (char *)malloc(u32_PathLength + 1);
    strcpy_s(stptr_FileParameters->pch_CompletePath,u32_PathLength + 1,FilePath);

	/* Find extension and Type */
	stptr_FileParameters->pch_Extension = (char *)malloc(8);
	stptr_FileParameters->en_ImageType = FindTypeFromFileName(stptr_FileParameters->pch_CompletePath,stptr_FileParameters->pch_Extension);
    //PRINTS(stptr_FileParameters->pch_Extension);

	/* Find trailing digits */
	switch(stptr_FileParameters->en_ImageType)
	{
	case EN_SAVE_AVI:
	case EN_SAVE_RAW:
		stptr_FileParameters->u8_NumberOfTrailingDigits = 0;
		stptr_FileParameters->u32_StartNumber = 0;
		break;

	case EN_SAVE_7Z:
		stptr_FileParameters->u8_NumberOfTrailingDigits = 6;
		stptr_FileParameters->u32_StartNumber = 0;
		break;
	case EN_SAVE_IZ:
	case EN_SAVE_IYUV:
	case EN_SAVE_IGREY:
	case EN_SAVE_IRCCC:
	case EN_SAVE_JPEG:
	case EN_SAVE_JPG:
	case EN_SAVE_BMP:
	case EN_SAVE_PNG:
	case EN_SAVE_RAW_RCCC16:
		stptr_FileParameters->u8_NumberOfTrailingDigits = 0;
		p_TempPointer = stptr_FileParameters->pch_CompletePath + strlen(stptr_FileParameters->pch_CompletePath) - strlen(stptr_FileParameters->pch_Extension) - 2;
		while( p_TempPointer > stptr_FileParameters->pch_CompletePath)
		{
			if('0' <= *(p_TempPointer))
			{
				if('9' >= *(p_TempPointer))
				{
					stptr_FileParameters->u8_NumberOfTrailingDigits++;
				}
				else 
				break;
			}
			else
				break;

			p_TempPointer --;
		}
		//PRINTD(stptr_FileParameters->u8_NumberOfTrailingDigits);
		sscanf_s(p_TempPointer+1,"%d",&(stptr_FileParameters->u32_StartNumber));
		//PRINTD(stptr_FileParameters->u32_StartNumber);
		break;	
	default:
		break;
	}

    /* Find File Name */
    p_TempPointer = stptr_FileParameters->pch_CompletePath + u32_PathLength;
    while( p_TempPointer > stptr_FileParameters->pch_CompletePath)
    {
        if('/' == *(p_TempPointer - 1))
        {
            break;
        }else if('\\' == *(p_TempPointer-1))
        {
            break;
        }

        p_TempPointer --;
    }
	
	PRINTS(p_TempPointer);

	//Store file name
    stptr_FileParameters->pch_FileName = (char *)malloc(strlen(p_TempPointer) + 1);
    strcpy_s(stptr_FileParameters->pch_FileName,strlen(p_TempPointer) + 1,p_TempPointer);
	stptr_FileParameters->pch_FileName[strlen(stptr_FileParameters->pch_FileName) - strlen(stptr_FileParameters->pch_Extension) - 1] = '\0';
    
	/*Check whether file name is in standard format or not. Standard form here means: YearMonthDate_HoursMinutes_SecondsMilliSeconds*/
	u32_ReturnValue = CheckFileNameIsStandard(stptr_FileParameters->pch_FileName);

	if ( !u32_ReturnValue)
	{
		stptr_FileParameters->b_IsStandard = 0;
	}
	else
	{
		stptr_FileParameters->b_IsStandard = 1;
	}
	
    /* Find whether name is standard */
    if(1 == stptr_FileParameters->b_IsStandard)
    {
		stptr_FileParameters->pch_BaseStandardName = (char *)malloc(32);
		strncpy_s(stptr_FileParameters->pch_BaseStandardName,32,stptr_FileParameters->pch_FileName,19);
		
		stptr_FileParameters->pch_BaseStandardName[19] = '\0';
      
		sscanf_s(stptr_FileParameters->pch_BaseStandardName,"%ld_%ld_%ld",
			&(stptr_FileParameters->u64_Day),
			&(stptr_FileParameters->u64_Time),
			&(stptr_FileParameters->u64_Seconds));

		stptr_FileParameters->u8_Day  =		stptr_FileParameters->u64_Day%100;
		stptr_FileParameters->u8_Month =	((stptr_FileParameters->u64_Day - stptr_FileParameters->u8_Day)/100)%100;
		stptr_FileParameters->u16_Year =	(uint16_t)((stptr_FileParameters->u64_Day - stptr_FileParameters->u8_Month*100 - stptr_FileParameters->u8_Day)/10000);
		
		stptr_FileParameters->u8_Minutes = 	stptr_FileParameters->u64_Time%100;
		stptr_FileParameters->u8_Hour =		((stptr_FileParameters->u64_Time - stptr_FileParameters->u8_Minutes)/100)%100;

		stptr_FileParameters->u16_MilliSeconds = 	stptr_FileParameters->u64_Seconds%1000;
		stptr_FileParameters->u8_Seconds =	(uint8_t)((stptr_FileParameters->u64_Seconds - stptr_FileParameters->u16_MilliSeconds)/1000);
		


       
		//PRINTL(stptr_FileParameters->u64_Day);
        //PRINTL(stptr_FileParameters->u64_Time);
        //PRINTL(stptr_FileParameters->u64_Seconds);

        stptr_FileParameters->pch_VideoName = (char *)malloc(strlen(stptr_FileParameters->pch_FileName) + 1);
        sprintf_s(stptr_FileParameters->pch_VideoName,strlen(stptr_FileParameters->pch_FileName) + 1
        ,"%08ld_%04ld_%05ld",(stptr_FileParameters->u64_Day),(stptr_FileParameters->u64_Time),(stptr_FileParameters->u64_Seconds));
        
    }else
    {
		stptr_FileParameters->pch_BaseStandardName = NULL;
        stptr_FileParameters->pch_VideoName = (char *)malloc(strlen(stptr_FileParameters->pch_FileName) + 1);
        strcpy_s(stptr_FileParameters->pch_VideoName,strlen(stptr_FileParameters->pch_FileName) + 1,stptr_FileParameters->pch_FileName);
        
    }

    //PRINTS(stptr_FileParameters->pch_VideoName);
    //PRINTD(stptr_FileParameters->b_IsStandard);
    
    /* Find Base Name */
    stptr_FileParameters->pch_BaseFileName = (char *)malloc(strlen(stptr_FileParameters->pch_FileName) + 1);
    strcpy_s(stptr_FileParameters->pch_BaseFileName,strlen(stptr_FileParameters->pch_FileName) + 1 , stptr_FileParameters->pch_FileName);
    stptr_FileParameters->pch_BaseFileName[strlen(stptr_FileParameters->pch_FileName) - stptr_FileParameters->u8_NumberOfTrailingDigits] = '\0';
    //PRINTS(stptr_FileParameters->pch_BaseFileName);


    /* Find File Path */
    do
    {
        if('/' == *(p_TempPointer -1))
        {
            p_TempPointer --;
            
        }else if('\\' == *(p_TempPointer - 1))
        {
            p_TempPointer --;
        }
        else
        {
            break;
        }
    }while( p_TempPointer > stptr_FileParameters->pch_CompletePath);
    *p_TempPointer = '\\';
    *(p_TempPointer+1) = '\0';
    stptr_FileParameters->pch_FolderPath = (char *)malloc(p_TempPointer - stptr_FileParameters->pch_CompletePath + 2 );
    strcpy_s(stptr_FileParameters->pch_FolderPath,p_TempPointer - stptr_FileParameters->pch_CompletePath + 2,stptr_FileParameters->pch_CompletePath);
    //PRINTS(stptr_FileParameters->pch_FolderPath);

	if(1 == stptr_FileParameters->b_IsStandard)
	{
		strncpy_s(stptr_FileParameters->ch_CSVFileName,sizeof(stptr_FileParameters->ch_CSVFileName),stptr_FileParameters->pch_VideoName,19);
		stptr_FileParameters->ch_CSVFileName[24] = '\0';
		strcat_s(stptr_FileParameters->ch_CSVFileName,sizeof(stptr_FileParameters->ch_CSVFileName),".csv");
	}
	else
	{	
		stptr_FileParameters->ch_CSVFileName[0] = '\0';
	}
	
	/*Get CSV File Path*/
    p_TempPointer = stptr_FileParameters->pch_FolderPath + strlen(stptr_FileParameters->pch_FolderPath) - 1;

	while( p_TempPointer > stptr_FileParameters->pch_CompletePath)
    {
        if('/' == *(p_TempPointer - 1))
        {
            break;
        }else if('\\' == *(p_TempPointer-1))
        {
            break;
        }
        p_TempPointer --;
    }

    stptr_FileParameters->pch_CSVFilePath =(char *) malloc(1 + (p_TempPointer - stptr_FileParameters->pch_FolderPath) + sizeof(stptr_FileParameters->ch_CSVFileName) +7);
    stptr_FileParameters->pch_CSVFolder =(char *) malloc(1 + (p_TempPointer - stptr_FileParameters->pch_FolderPath) + sizeof(stptr_FileParameters->ch_CSVFileName) +7);
    
	//strncpy_s(stptr_FileParameters->pch_CSVFilePath,(p_TempPointer - stptr_FileParameters->pch_FolderPath + 1),stptr_FileParameters->pch_FolderPath,(p_TempPointer - stptr_FileParameters->pch_FolderPath));

	switch(stptr_FileParameters->en_ImageType)
	{
	case EN_SAVE_AVI:
	case EN_SAVE_7Z:
	case EN_SAVE_RAW:
	case EN_SAVE_RAW_YUV:
	case EN_SAVE_RAW_YUV420_UYVY:
	case EN_SAVE_RAW_YUV420_YUYV:
	case EN_SAVE_RAW_RCCC16:
			sprintf_s(stptr_FileParameters->pch_CSVFilePath,sizeof(stptr_FileParameters->ch_CSVFileName),"%s\\..\\CSV\\%s",stptr_FileParameters->pch_FolderPath,stptr_FileParameters->ch_CSVFileName);
			sprintf_s(stptr_FileParameters->pch_CSVFolder,sizeof(stptr_FileParameters->ch_CSVFileName),"%s\\..\\CSV\\",stptr_FileParameters->pch_FolderPath);
		break;

	case EN_SAVE_IZ:
	case EN_SAVE_IYUV:
	case EN_SAVE_IGREY:
	case EN_SAVE_JPEG:
	case EN_SAVE_JPG:
	case EN_SAVE_BMP:
	case EN_SAVE_PNG:
	case EN_SAVE_IRCCC:
		sprintf_s(stptr_FileParameters->pch_CSVFilePath,sizeof(stptr_FileParameters->ch_CSVFileName),"%s\\..\\..\\CSV\\%s",stptr_FileParameters->pch_FolderPath,stptr_FileParameters->ch_CSVFileName);
		sprintf_s(stptr_FileParameters->pch_CSVFolder,sizeof(stptr_FileParameters->ch_CSVFileName),"%s\\..\\..\\CSV\\",stptr_FileParameters->pch_FolderPath);
		break;	
	default:
		break;
	}
	

	

	if( stptr_FileParameters->en_ImageType == EN_SAVE_AVI)
	{
		//stptr_FileParameters->pch_CSVFilePath = (char *)malloc( 1 + sizeof(stptr_FileParameters->pch_FolderPath) + sizeof(stptr_FileParameters->ch_CSVFileName)+7);
		//sprintf_s(stptr_FileParameters->pch_CSVFilePath,strlen(stptr_FileParameters->pch_CSVFilePath),"%s\\CSV\\%s",stptr_FileParameters->pch_FolderPath,stptr_FileParameters->ch_CSVFileName );
		stptr_FileParameters->pch_CSVFilePath[strlen(stptr_FileParameters->pch_CSVFilePath)]= '\0';
		//PRINTS(stptr_FileParameters->pch_CSVFilePath);
	}
 
    /*Complete File Path is overwritten here. So copying complete file Path again*/
    u32_PathLength = strlen(FilePath);
    stptr_FileParameters->pch_CompletePath = (char *)malloc(u32_PathLength + 1);
    strcpy_s(stptr_FileParameters->pch_CompletePath,u32_PathLength + 1,FilePath);
    
    return stptr_FileParameters;
}

#define RELEASE_POINTER_IFNOT_NULL(x) if(NULL != x) free(x)

void ADAS_InitFileParameters(st_file_name_parameters_t * p_stFileParameters)
{

	p_stFileParameters->pch_BaseFileName = NULL;
	p_stFileParameters->pch_BaseStandardName= NULL;
	p_stFileParameters->pch_CompletePath = NULL;
	p_stFileParameters->pch_Extension = NULL;
	p_stFileParameters->pch_FileName = NULL;
	p_stFileParameters->pch_FolderPath = NULL;
	p_stFileParameters->pch_VideoName = NULL;
	
	memset(p_stFileParameters->ch_CSVFileName,'\0', sizeof(p_stFileParameters->ch_CSVFileName));

	/*RELEASE_POINTER_IFNOT_NULL(p_stFileParameters->pch_CSVFilePath);
	RELEASE_POINTER_IFNOT_NULL(p_stFileParameters->pch_CSVFolder);*/

	p_stFileParameters->pch_CSVFilePath = NULL;
	p_stFileParameters->pch_CSVFolder = NULL;

	p_stFileParameters->en_ImageType = EN_UNKNOWN_IMAGE_SAVE_TYPE;
	p_stFileParameters->en_ContainerType = EN_UNKNOWN_IMAGE_CONTAINER_TYPE;
    p_stFileParameters->u8_NumberOfTrailingDigits = 0;
    p_stFileParameters->u32_StartNumber = 0;
    p_stFileParameters->b_IsStandard = 0;
	p_stFileParameters->u16_Year = 0;
	p_stFileParameters->u8_Month = 0;
	p_stFileParameters->u8_Day = 0;
	p_stFileParameters->u8_Hour = 0;
	p_stFileParameters->u8_Minutes = 0;
	p_stFileParameters->u8_Seconds = 0;
	p_stFileParameters->u16_MilliSeconds = 0;
    p_stFileParameters->u64_Day = 0;
    p_stFileParameters->u64_Time = 0;
    p_stFileParameters->u64_Seconds = 0;
}


void ADAS_ReleaseFileParameters(st_file_name_parameters_t * p_stFileParameters)
{
	if(NULL != p_stFileParameters)
	{
		RELEASE_POINTER_IFNOT_NULL(p_stFileParameters->pch_BaseFileName ) ;
		RELEASE_POINTER_IFNOT_NULL(p_stFileParameters->pch_BaseStandardName );
		RELEASE_POINTER_IFNOT_NULL(p_stFileParameters->pch_CompletePath );
		RELEASE_POINTER_IFNOT_NULL(p_stFileParameters->pch_CSVFilePath );
		RELEASE_POINTER_IFNOT_NULL(p_stFileParameters->pch_CSVFolder);
		RELEASE_POINTER_IFNOT_NULL(p_stFileParameters->pch_Extension );
		RELEASE_POINTER_IFNOT_NULL(p_stFileParameters->pch_FileName );
		RELEASE_POINTER_IFNOT_NULL(p_stFileParameters->pch_FolderPath );
		RELEASE_POINTER_IFNOT_NULL(p_stFileParameters->pch_VideoName );
		RELEASE_POINTER_IFNOT_NULL(p_stFileParameters);
	}
}

/*******************************************************************************/
/* Function Name   : CheckFileNameIsStandard								   */
/* Input Arguments : const char* const FileName 							   */
/* Return Value    : int													   */
/* Description     : This function is used to check whether file name is in    */
/*					 standard format or not. Standard format is as below:	   */
/*					 YearMonthDate_HourMinute_Secondsmilliseconds              */
/*******************************************************************************/

int CheckFileNameIsStandard (const char* const FileName)
{
	char* temp_ptr ;
	unsigned short DateLength = 0;
	unsigned short HrMinLength = 0;
	unsigned short SecMilisecLength = 0;
	unsigned int FileNameLength = 0;

	st_file_name_parameters_t * stptr_FileParameters1;

	FileNameLength = strlen(FileName);

	stptr_FileParameters1 = (st_file_name_parameters_t *)malloc(sizeof(st_file_name_parameters_t));
	stptr_FileParameters1->pch_FileName = (char *)malloc(FileNameLength + 1);
	strcpy_s(stptr_FileParameters1->pch_FileName, FileNameLength+1,FileName);
	stptr_FileParameters1->b_IsStandard = 0;
	
	/*Start with the check of year first*/
	temp_ptr = stptr_FileParameters1->pch_FileName;

	while ( *(temp_ptr) != '_')
	{
		if ((*(temp_ptr) >= '0') && (*(temp_ptr) <= '9'))
		{
			DateLength++;
		}
		else
		{
			break;
		}
		temp_ptr++;
	}
	if ( DateLength != 8) 
	{
		return (stptr_FileParameters1->b_IsStandard);
	}
	else
	{
		sscanf_s(FileName, "%4d%2d%2d", &(stptr_FileParameters1->u16_Year),&(stptr_FileParameters1->u8_Month),&(stptr_FileParameters1->u8_Day));
		if (stptr_FileParameters1->u16_Year < 2015)
		{
			return (stptr_FileParameters1->b_IsStandard);
		}
		if ( (stptr_FileParameters1->u8_Month < 1) || (stptr_FileParameters1->u8_Month > 12))
		{
			return (stptr_FileParameters1->b_IsStandard);
		}
		if ( (stptr_FileParameters1->u8_Day < 1) || (stptr_FileParameters1->u8_Day > 31))
		{
			return (stptr_FileParameters1->b_IsStandard);
		}
	}/*Check for Correct date Fromat Ends here.*/

	/*Next character expected in file name is '_' */
	if (  *(temp_ptr) != '_' )
	{
		return (stptr_FileParameters1->b_IsStandard);
	}
	temp_ptr ++;
	/*Next in the expected format is Hour & Minutes value.*/
	while ( *(temp_ptr) != '_')
	{
		if( (*(temp_ptr) >= '0') && (*(temp_ptr) <= '9'))
		{
			HrMinLength ++ ;
		}
		else
		{
			return (stptr_FileParameters1->b_IsStandard);
			break;
		}
		temp_ptr++ ;
	}
	if ( HrMinLength !=4 )
	{
		return (stptr_FileParameters1->b_IsStandard);
	}
	else
	{
		sscanf_s(&FileName[FileNameLength - ((strlen(temp_ptr)) + HrMinLength)], "%2d%2d", &(stptr_FileParameters1->u8_Hour),&(stptr_FileParameters1->u8_Minutes));
		if (( stptr_FileParameters1->u8_Hour < 0 ) || ( stptr_FileParameters1->u8_Hour >= 24 ))
		{
			return (stptr_FileParameters1->b_IsStandard);
		}
		if (( stptr_FileParameters1->u8_Minutes < 0 ) || ( stptr_FileParameters1->u8_Minutes >= 60 ))
		{
			return (stptr_FileParameters1->b_IsStandard);
		}
	}
	
	/*Next character expected in file name is '_' */
	if (  *(temp_ptr) != '_' )
	{
		return (stptr_FileParameters1->b_IsStandard);
	}
	temp_ptr ++;

	/*Next in the expected format is Seconds & Milliseconds value.*/
	while (( *(temp_ptr) != '_') && ( *(temp_ptr) != '\0'))
	{
		if( (*(temp_ptr) >= '0') && (*(temp_ptr) <= '9'))
		{
			SecMilisecLength ++ ;
		}
		else
		{
			return (stptr_FileParameters1->b_IsStandard);
			break;
		}
		temp_ptr++ ;
	}
	if ( SecMilisecLength != 5 )
	{
		return (stptr_FileParameters1->b_IsStandard);
	}
	else
	{
		sscanf_s(&FileName[FileNameLength - ((strlen(temp_ptr)) + SecMilisecLength)], "%02d%03d", &(stptr_FileParameters1->u8_Seconds),&(stptr_FileParameters1->u16_MilliSeconds));
		if (( stptr_FileParameters1->u8_Seconds < 0 ) || ( stptr_FileParameters1->u8_Seconds >= 60 ))
		{
			return (stptr_FileParameters1->b_IsStandard);
		}
		if (( stptr_FileParameters1->u16_MilliSeconds < 0 ) || ( stptr_FileParameters1->u16_MilliSeconds >= 1000 ))
		{
			return (stptr_FileParameters1->b_IsStandard);
		}
	}

	stptr_FileParameters1->b_IsStandard = 1;
	return stptr_FileParameters1->b_IsStandard;
}
#endif
/***********************
* Module		: YUV422_UYVYtoRGB888_BGR()
* Description	: convert yuv422 interleaved to RGB interleaved	
***********************/
void YUV422_UYVYtoRGB888_BGR(const unsigned char *ptr_u8YUV, unsigned char *ptr_u8BGR,
			unsigned int u32ImageSize)
{

	#define SATURATE(x)					(((x) > 255) ? (255) : (((x) < 0) ? (0) : (x)))

	unsigned char *ptr_u8Out_RGB = (unsigned char *)ptr_u8BGR;
    unsigned char *ptr_u8In_YUV = (unsigned char *)ptr_u8YUV;
	
	double RData0, RData1;
	double GData0, GData1;
	double BData0, BData1;
	
	double YData0, YData1;
	double UData, VData;

	unsigned long u32loopCounter;
	
	u32ImageSize = (u32ImageSize * 3);	//RGB image size.
	
	/*Loop mapping RGB to YUV image.*/
    for(u32loopCounter = 0; u32loopCounter < u32ImageSize; u32loopCounter += 6)
	{
        	UData = (*ptr_u8In_YUV);
		ptr_u8In_YUV++;
		YData0 = (*ptr_u8In_YUV);
		ptr_u8In_YUV++;
		VData = (*ptr_u8In_YUV);
		ptr_u8In_YUV++;
		YData1 = (*ptr_u8In_YUV);
		ptr_u8In_YUV++;
		
		RData0 = YData0 + 0.0000000 * (UData - 128) + 1.1398320 * (VData - 128);
		GData0 = YData0 - 0.3946500 * (UData - 128) - 0.5806000 * (VData - 128);
		BData0 = YData0 + 2.0321120 * (UData - 128) + 0.0000000 * (VData - 128);
		
		RData1 = YData1 + 0.0000000 * (UData - 128) + 1.1398320 * (VData - 128);
		GData1 = YData1 - 0.3946500 * (UData - 128) - 0.5806000 * (VData - 128);
		BData1 = YData1 + 2.0321120 * (UData - 128) + 0.0000000 * (VData - 128);
	
		*(unsigned char *)ptr_u8Out_RGB = (unsigned char)SATURATE(BData0);	
		(ptr_u8Out_RGB)++;
		*(unsigned char *)ptr_u8Out_RGB = (unsigned char)SATURATE(GData0);	
		(ptr_u8Out_RGB)++;
		*(unsigned char *)ptr_u8Out_RGB = (unsigned char)SATURATE(RData0);	
		(ptr_u8Out_RGB)++;

		*(unsigned char *)ptr_u8Out_RGB = (unsigned char)SATURATE(BData1);	
		(ptr_u8Out_RGB)++;
		*(unsigned char *)ptr_u8Out_RGB = (unsigned char)SATURATE(GData1);	
		(ptr_u8Out_RGB)++;
		*(unsigned char *)ptr_u8Out_RGB = (unsigned char)SATURATE(RData1);	
		(ptr_u8Out_RGB)++;
	}
}


/***********************
* Module		: YUV422_UYVYtoRGB888_GBR()
* Description	: convert yuv422 interleaved to RGB interleaved	
***********************/
void YUV422_UYVYtoRGB888_GBR(const unsigned char *ptr_u8YUV, unsigned char *ptr_u8BGR,
			unsigned int u32ImageSize)
{

	#define SATURATE(x)					(((x) > 255) ? (255) : (((x) < 0) ? (0) : (x)))

	unsigned char *ptr_u8Out_RGB = (unsigned char *)ptr_u8BGR;
    unsigned char *ptr_u8In_YUV = (unsigned char *)ptr_u8YUV;
	
	double RData0, RData1;
	double GData0, GData1;
	double BData0, BData1;
	
	double YData0, YData1;
	double UData, VData;

	unsigned long u32loopCounter;
	
	u32ImageSize = (u32ImageSize * 3);	//RGB image size.
	
	/*Loop mapping RGB to YUV image.*/
    for(u32loopCounter = 0; u32loopCounter < u32ImageSize; u32loopCounter += 6)
	{
        	UData = (*ptr_u8In_YUV);
		ptr_u8In_YUV++;
		YData0 = (*ptr_u8In_YUV);
		ptr_u8In_YUV++;
		VData = (*ptr_u8In_YUV);
		ptr_u8In_YUV++;
		YData1 = (*ptr_u8In_YUV);
		ptr_u8In_YUV++;
		
		RData0 = YData0 + 0.0000000 * (UData - 128) + 1.1398320 * (VData - 128);
		GData0 = YData0 - 0.3946500 * (UData - 128) - 0.5806000 * (VData - 128);
		BData0 = YData0 + 2.0321120 * (UData - 128) + 0.0000000 * (VData - 128);
		
		RData1 = YData1 + 0.0000000 * (UData - 128) + 1.1398320 * (VData - 128);
		GData1 = YData1 - 0.3946500 * (UData - 128) - 0.5806000 * (VData - 128);
		BData1 = YData1 + 2.0321120 * (UData - 128) + 0.0000000 * (VData - 128);
	
		*(unsigned char *)ptr_u8Out_RGB = (unsigned char)SATURATE(GData0);	
		(ptr_u8Out_RGB)++;
		*(unsigned char *)ptr_u8Out_RGB = (unsigned char)SATURATE(BData0);	
		(ptr_u8Out_RGB)++;
		*(unsigned char *)ptr_u8Out_RGB = (unsigned char)SATURATE(RData0);	
		(ptr_u8Out_RGB)++;

		*(unsigned char *)ptr_u8Out_RGB = (unsigned char)SATURATE(GData1);	
		(ptr_u8Out_RGB)++;
		*(unsigned char *)ptr_u8Out_RGB = (unsigned char)SATURATE(BData1);	
		(ptr_u8Out_RGB)++;
		*(unsigned char *)ptr_u8Out_RGB = (unsigned char)SATURATE(RData1);	
		(ptr_u8Out_RGB)++;
	}
}


/***********************
* Module		: RGB888_BGRtoYUV422_UYVY()
* Description	: convert RGB interleaved to YUV 422 interleaved	
***********************/
void RGB888_BGRtoYUV422_UYVY(const unsigned char *ptr_u8BGR, unsigned char *ptr_u8YUV,
			unsigned int u32ImageSize)
{
#define SATURATE(x)					(((x) > 255) ? (255) : (((x) < 0) ? (0) : (x)))

	unsigned char *ptr_u8In_BGR = (unsigned char *)ptr_u8BGR;
    unsigned char *ptr_u8Out_YUV = (unsigned char *)ptr_u8YUV;
	
	double RData0, RData1;
	double GData0, GData1;
	double BData0, BData1;
	
	double YData0, YData1;
	double UData, VData;
	unsigned long u32loopCounter;
	
	u32ImageSize = (u32ImageSize * 2);	//RGB image size.

	/*Loop mapping RGB to YUV image.*/
    for(u32loopCounter = 0; u32loopCounter < u32ImageSize; u32loopCounter += 4)
	{
        BData0 = (*ptr_u8In_BGR);
		ptr_u8In_BGR++;
		GData0 = (*ptr_u8In_BGR);
		ptr_u8In_BGR++;
		RData0 = (*ptr_u8In_BGR);
		ptr_u8In_BGR++;
		
		BData1 = (*ptr_u8In_BGR);
		ptr_u8In_BGR++;
		GData1 = (*ptr_u8In_BGR);
		ptr_u8In_BGR++;
		RData1 = (*ptr_u8In_BGR);
		ptr_u8In_BGR++;
		
		YData0 = RData0 * 0.2990000		+ GData0 * 0.5870000	+ BData0 * 0.1140000;
		YData1 = RData1 * 0.2990000		+ GData1 * 0.5870000	+ BData1 * 0.1140000;
		
		UData = ((RData0 + RData1)/2) * -0.1471300 	+ ((GData0 + GData1)/2) * -0.2888600	+ ((BData0 + BData1)/2) *  0.4360000;
		VData = ((RData0 + RData1)/2) *  0.6150000	+ ((GData0 + GData1)/2) * -0.5149900	+ ((BData0 + BData1)/2) * -0.1000100;
		
		// U Data 
		*(unsigned char *)ptr_u8Out_YUV = (unsigned char)SATURATE(UData + 128);	
		(ptr_u8Out_YUV)++;
		// Y 0 Data 
		*(unsigned char *)ptr_u8Out_YUV = (unsigned char)SATURATE(YData0);	
		(ptr_u8Out_YUV)++;
		// V Data 
		*(unsigned char *)ptr_u8Out_YUV = (unsigned char)SATURATE(VData  + 128);	
		(ptr_u8Out_YUV)++;
		// Y 1 Data 
		*(unsigned char *)ptr_u8Out_YUV = (unsigned char)SATURATE(YData1);	
		(ptr_u8Out_YUV)++;
	}
}

/***********************
* Module		: RGB888_BGRtoYUV422_UYVY_Fixed()
* Description	: convert RGB interleaved to YUV 422 interleaved	
***********************/
void RGB888_BGRtoYUV422_UYVY_Fixed(const unsigned char *ptr_u8BGR, unsigned char *ptr_u8YUV,
			unsigned int u32ImageSize)
{
	#define SATURATE(x)					(((x) > 255) ? (255) : (((x) < 0) ? (0) : (x)))
	#define Q_VALUE 20
	#define MULTIPLY(x1, x2)			(((x1) * (x2)) >> Q_VALUE)

#if (20 == Q_VALUE)
	long FixYRConst = 313524;		// 0.299 
	long FixYGConst = 615514;		// 0.587
	long FixYBConst = 119538;		// 0.114
	long FixURConst = -154277;		// -0.14713
	long FixUGConst = -302892;		// -0.28886
	long FixUBConst = 457179;		// 0.436
	long FixVRConst = 644874;		// 0.615
	long FixVGConst = -540006;		// -0.51499
	long FixVBConst = -104868;		// -0.10001
#endif


	unsigned char *ptr_u8In_BGR = (unsigned char *)ptr_u8BGR;
    unsigned char *ptr_u8Out_YUV = (unsigned char *)ptr_u8YUV;
	
	long RData0, RData1;
	long GData0, GData1;
	long BData0, BData1;
	
	long YData0, YData1;
	long UData, VData;

	unsigned int u32loopCounter;
	
	u32ImageSize = (u32ImageSize * 2);	//RGB image size.

	/*Loop mapping RGB to YUV image.*/
    for(u32loopCounter = 0; u32loopCounter < u32ImageSize; u32loopCounter += 4)
	{
		BData0 = ((long)(*ptr_u8In_BGR))<<Q_VALUE;
		ptr_u8In_BGR++;
		GData0 = ((long)(*ptr_u8In_BGR))<<Q_VALUE;
		ptr_u8In_BGR++;
		RData0 = ((long)(*ptr_u8In_BGR))<<Q_VALUE;
		ptr_u8In_BGR++;
		
		BData1 = ((long)(*ptr_u8In_BGR))<<Q_VALUE;
		ptr_u8In_BGR++;
		GData1 = ((long)(*ptr_u8In_BGR))<<Q_VALUE;
		ptr_u8In_BGR++;
		RData1 = ((long)(*ptr_u8In_BGR))<<Q_VALUE;
		ptr_u8In_BGR++;
		
		YData0 = 0;
		YData0 += MULTIPLY(RData0,(long long)FixYRConst);
		YData0 += MULTIPLY(GData0,(long long)FixYGConst);
		YData0 += MULTIPLY(BData0,(long long)FixYBConst);

		YData1 = 0;
		YData1 += MULTIPLY(RData1,(long long)FixYRConst);
		YData1 += MULTIPLY(GData1,(long long)FixYGConst);
		YData1 += MULTIPLY(BData1,(long long)FixYBConst);
		
		
		UData = 0;
		UData += MULTIPLY((RData0 + RData1)/2,(long long)FixURConst);
		UData += MULTIPLY((GData0 + GData1)/2,(long long)FixUGConst);
		UData += MULTIPLY((BData0 + BData1)/2,(long long)FixUBConst);
		
		VData = 0;
		VData += MULTIPLY((RData0 + RData1)/2,(long long)FixVRConst);
		VData += MULTIPLY((GData0 + GData1)/2,(long long)FixVGConst);
		VData += MULTIPLY((BData0 + BData1)/2,(long long)FixVBConst);
		
		// U Data 
		*(unsigned char *)ptr_u8Out_YUV = (unsigned char)SATURATE((UData>>Q_VALUE) + 128);	
		(ptr_u8Out_YUV)++;
		// Y 0 Data 
		*(unsigned char *)ptr_u8Out_YUV = (unsigned char)SATURATE((YData0>>Q_VALUE));	
		(ptr_u8Out_YUV)++;
		// V Data 
		*(unsigned char *)ptr_u8Out_YUV = (unsigned char)SATURATE((VData>>Q_VALUE)  + 128);	
		(ptr_u8Out_YUV)++;
		// Y 1 Data 
		*(unsigned char *)ptr_u8Out_YUV = (unsigned char)SATURATE((YData1>>Q_VALUE));	
		(ptr_u8Out_YUV)++;
	}
}



/***********************
* Module		: RGB888_BGRtoYUV422_YUYV()
* Description	: convert RGB interleaved to YUV 422 interleaved	
***********************/
void RGB888_BGRtoYUV422_YUYV(const unsigned char *ptr_u8BGR, unsigned char *ptr_u8YUV,
			unsigned int u32ImageSize)
{
#define SATURATE(x)					(((x) > 255) ? (255) : (((x) < 0) ? (0) : (x)))

	unsigned char *ptr_u8In_BGR = (unsigned char *)ptr_u8BGR;
    unsigned char *ptr_u8Out_YUV = (unsigned char *)ptr_u8YUV;
	
	double RData0, RData1;
	double GData0, GData1;
	double BData0, BData1;
	
	double YData0, YData1;
	double UData, VData;
	unsigned long u32loopCounter;
	
	u32ImageSize = (u32ImageSize * 2);	//RGB image size.
	
	/*Loop mapping RGB to YUV image.*/
    for(u32loopCounter = 0; u32loopCounter < u32ImageSize; u32loopCounter += 4)
	{
        BData0 = (*ptr_u8In_BGR);
		ptr_u8In_BGR++;
		GData0 = (*ptr_u8In_BGR);
		ptr_u8In_BGR++;
		RData0 = (*ptr_u8In_BGR);
		ptr_u8In_BGR++;
		
		BData1 = (*ptr_u8In_BGR);
		ptr_u8In_BGR++;
		GData1 = (*ptr_u8In_BGR);
		ptr_u8In_BGR++;
		RData1 = (*ptr_u8In_BGR);
		ptr_u8In_BGR++;
		
		YData0 = RData0 * 0.2990000		+ GData0 * 0.5870000	+ BData0 * 0.1140000;
		YData1 = RData1 * 0.2990000		+ GData1 * 0.5870000	+ BData1 * 0.1140000;
		
		UData = ((RData0)) * -0.1471300 	+ ((GData0)) * -0.2888600	+ ((BData0)) *  0.4360000;
		VData = ((RData0)) *  0.6150000	+ ((GData0)) * -0.5149900	+ ((BData0)) * -0.1000100;
		
		// Y 0 Data 
		*(unsigned char *)ptr_u8Out_YUV = (unsigned char)SATURATE(YData0);	
		(ptr_u8Out_YUV)++;
		// U Data 
		*(unsigned char *)ptr_u8Out_YUV = (unsigned char)SATURATE(UData + 128);	
		(ptr_u8Out_YUV)++;
		// Y 1 Data 
		*(unsigned char *)ptr_u8Out_YUV = (unsigned char)SATURATE(YData1);	
		(ptr_u8Out_YUV)++;
		// V Data 
		*(unsigned char *)ptr_u8Out_YUV = (unsigned char)SATURATE(VData  + 128);	
		(ptr_u8Out_YUV)++;
		
		
	}
}


#if 0 
void RGB888_BGRtoYUV422_YUYV(const unsigned char *ptr_u8BGR, unsigned char *ptr_u8YUV,
			unsigned int u32ImageSize)
{
#define SATURATE(x)					(((x) > 255) ? (255) : (((x) < 0) ? (0) : (x)))

	unsigned char *ptr_u8In_BGR = (unsigned char *)ptr_u8BGR;
    unsigned char *ptr_u8Out_YUV = (unsigned char *)ptr_u8YUV;
	
	double RData0, RData1;
	double GData0, GData1;
	double BData0, BData1;
	
	double YData0, YData1;
	double UData, VData;
	unsigned long u32loopCounter;
	
	u32ImageSize = (u32ImageSize * 2);	//RGB image size.
	
	/*Loop mapping RGB to YUV image.*/
    for(u32loopCounter = 0; u32loopCounter < u32ImageSize; u32loopCounter += 4)
	{
        BData0 = (*ptr_u8In_BGR);
		ptr_u8In_BGR++;
		GData0 = (*ptr_u8In_BGR);
		ptr_u8In_BGR++;
		RData0 = (*ptr_u8In_BGR);
		ptr_u8In_BGR++;
		
		BData1 = (*ptr_u8In_BGR);
		ptr_u8In_BGR++;
		GData1 = (*ptr_u8In_BGR);
		ptr_u8In_BGR++;
		RData1 = (*ptr_u8In_BGR);
		ptr_u8In_BGR++;
		
		YData0 = RData0 * 0.2990000		+ GData0 * 0.5870000	+ BData0 * 0.1140000;
		YData1 = RData1 * 0.2990000		+ GData1 * 0.5870000	+ BData1 * 0.1140000;
		
		UData = ((RData0)) * -0.1471300 	+ ((GData0)) * -0.2888600	+ ((BData0)) *  0.4360000;
		VData = ((RData0)) *  0.6150000	+ ((GData0)) * -0.5149900	+ ((BData0)) * -0.1000100;
		
		// Y 0 Data 
		*(unsigned char *)ptr_u8Out_YUV = (unsigned char)SATURATE(YData0);	
		(ptr_u8Out_YUV)++;
		// U Data 
		*(unsigned char *)ptr_u8Out_YUV = (unsigned char)SATURATE(UData + 128);	
		(ptr_u8Out_YUV)++;
		// Y 1 Data 
		*(unsigned char *)ptr_u8Out_YUV = (unsigned char)SATURATE(YData1);	
		(ptr_u8Out_YUV)++;
		// V Data 
		*(unsigned char *)ptr_u8Out_YUV = (unsigned char)SATURATE(VData  + 128);	
		(ptr_u8Out_YUV)++;
		
		
	}
}
#endif

void YUV422toRGB888(const unsigned char *ptr_u8YUVIntImg, 
				unsigned int u32ImageSize, unsigned char *ptr_Image)
{
	#define SATURATE(x)					(((x) > 255) ? (255) : (((x) < 0) ? (0) : (x)))
	#define Q_VALUE 20
	#define MULTIPLY(x1, x2)			(((x1) * (x2)) >> Q_VALUE)

	unsigned char *ptr_u8Int = (unsigned char *)ptr_u8YUVIntImg;
	unsigned char *ptr_RGBBuff = (unsigned char *)ptr_Image;
	long u8yData0, u8yData1;
	long R_UProduct, G1_UProduct, G2_VProduct, B_VProduct;
	long long u8uData, u8vData;
	unsigned int u32loopCounter;
	long long FixRConst, FixG1Const, FixG2Const, FixBConst;

	FixRConst = 1857028;
	FixG1Const = 362388;
	FixG2Const = 749208;
	FixBConst = 1470313;
	
	u32ImageSize = (u32ImageSize * 3);	//RGB image size.
	
	/*Loop mapping YUV to RGB image.*/
	for(u32loopCounter = 0; u32loopCounter < u32ImageSize; u32loopCounter += 6)
	{
		u8uData    = ((((*ptr_u8Int++) << Q_VALUE) - (127 << Q_VALUE)) );		//U in fixed point format in YUYV image.
		u8yData0     = ((*ptr_u8Int++) << Q_VALUE);								//Y0 in fixed point format in YUYV image.
		u8vData    = ((((*ptr_u8Int++) << Q_VALUE) - (127 << Q_VALUE)) );		//V in fixed point format in YUYV image.
		u8yData1     = ((*ptr_u8Int++) << Q_VALUE);								//Y1 in fixed point format in YUYV image.
		
		

		R_UProduct = (unsigned long)MULTIPLY(FixRConst, u8uData);				//(1.7710 * U)
		G1_UProduct = (unsigned long)MULTIPLY(FixG1Const, u8uData);				//(0.3456 * U) 
		G2_VProduct = (unsigned long)MULTIPLY(FixG2Const, u8vData);				//(0.7145 * V)
		B_VProduct = (unsigned long)MULTIPLY(FixBConst, u8vData);				//(1.4022 * V)
		
		// B Data of RGB image
		*ptr_RGBBuff = (unsigned char)SATURATE(((u8yData0 + B_VProduct)) >> Q_VALUE);
		ptr_RGBBuff++;
		// G Data of RGB image
		*ptr_RGBBuff = (unsigned char)SATURATE(((u8yData0 - G1_UProduct - G2_VProduct)) >> Q_VALUE);
		ptr_RGBBuff++;
		// R Data of RGB image
		*ptr_RGBBuff = (unsigned char)SATURATE(((u8yData0 + R_UProduct)) >> Q_VALUE);	
		ptr_RGBBuff++;
		
		// B Data of RGB image
		*ptr_RGBBuff = (unsigned char)SATURATE(((u8yData1 + B_VProduct)) >> Q_VALUE);			
		ptr_RGBBuff++;
		// G Data of RGB image
		*ptr_RGBBuff = (unsigned char)SATURATE(((u8yData1 - G1_UProduct - G2_VProduct)) >> Q_VALUE);
		ptr_RGBBuff++;
		// R Data of RGB image
		*ptr_RGBBuff = (unsigned char)SATURATE(((u8yData1 + R_UProduct)) >> Q_VALUE);
		ptr_RGBBuff++;
		
	}
}

unsigned char * YUV444toYUV422_UYVY(unsigned char * p_YUVData,unsigned int intImageSize)
{
	unsigned char * p_RearrangedData;
	unsigned int PixelCounter;

	unsigned char *p_In_UData = p_YUVData + 1;
	unsigned char *p_In_VData = p_YUVData + 2;
	unsigned char *p_In_YData = p_YUVData + 0;

	unsigned char *p_Out_UData;
	unsigned char *p_Out_VData;
	unsigned char *p_Out_YData;

	p_RearrangedData = (unsigned char *)malloc(intImageSize*2);

	p_Out_YData = p_RearrangedData + 1;
	p_Out_UData = p_RearrangedData + 0;
	p_Out_VData = p_RearrangedData + 2;

	for(PixelCounter = 0;PixelCounter < intImageSize;PixelCounter+=2)
	{
		*p_Out_YData = *p_In_YData;
		p_Out_YData += 2 ;
		p_In_YData += 3;
		*p_Out_YData = *p_In_YData;
		p_Out_YData += 2 ;
		p_In_YData += 3 ;

		*p_Out_UData = *p_In_UData;
		p_Out_UData += 4 ;
		p_In_UData += 6 ;

		*p_Out_VData = *p_In_VData;
		p_Out_VData += 4 ;
		p_In_VData += 6 ;
	}

	free(p_YUVData);
	return p_RearrangedData;
}

unsigned char * YUV444toYUV422_YUYV(unsigned char * p_YUVData,unsigned int intImageSize)
{
	unsigned char * p_RearrangedData;
	unsigned int PixelCounter;

	unsigned char *p_In_UData = p_YUVData + 1;
	unsigned char *p_In_VData = p_YUVData + 2;
	unsigned char *p_In_YData = p_YUVData + 0;

	unsigned char *p_Out_UData;
	unsigned char *p_Out_VData;
	unsigned char *p_Out_YData;

	p_RearrangedData = (unsigned char *)malloc(intImageSize*2);

	p_Out_YData = p_RearrangedData + 0;
	p_Out_UData = p_RearrangedData + 1;
	p_Out_VData = p_RearrangedData + 3;

	for(PixelCounter = 0;PixelCounter < intImageSize;PixelCounter+=2)
	{
		*p_Out_YData = *p_In_YData;
		p_Out_YData += 2 ;
		p_In_YData += 3;
		*p_Out_YData = *p_In_YData;
		p_Out_YData += 2 ;
		p_In_YData += 3 ;

		*p_Out_UData = *p_In_UData;
		p_Out_UData += 4 ;
		p_In_UData += 6 ;

		*p_Out_VData = *p_In_VData;
		p_Out_VData += 4 ;
		p_In_VData += 6 ;
	}

	free(p_YUVData);
	return p_RearrangedData;
}



unsigned char * YUV422_UYVYtoYUV444(unsigned char * p_UYVYData,unsigned int intImageSize)
{
	unsigned char * p_RearrangedData;
	unsigned int PixelCounter;

	unsigned char *p_In_UData = p_UYVYData + 0;
	unsigned char *p_In_VData = p_UYVYData + 2;
	unsigned char *p_In_YData = p_UYVYData + 1;

	unsigned char *p_Out_UData;
	unsigned char *p_Out_VData;
	unsigned char *p_Out_YData;

	p_RearrangedData = (unsigned char *)malloc(intImageSize*3);

	p_Out_YData = p_RearrangedData;
	p_Out_UData = p_RearrangedData + 1;
	p_Out_VData = p_RearrangedData + 2;

	for(PixelCounter = 0;PixelCounter < intImageSize;PixelCounter+=2)
	{
		*p_Out_YData = *p_In_YData;
		p_Out_YData += 3 ;
		p_In_YData += 2;
		*p_Out_YData = *p_In_YData;
		p_Out_YData += 3 ;
		p_In_YData += 2 ;

		*p_Out_UData = *p_In_UData;
		p_Out_UData += 3 ;
		*p_Out_UData = *p_In_UData;
		p_Out_UData += 3 ;
		p_In_UData += 4 ;

		*p_Out_VData = *p_In_VData;
		p_Out_VData += 3 ;
		*p_Out_VData = *p_In_VData;
		p_Out_VData += 3 ;
		p_In_VData += 4 ;
	}

	free(p_UYVYData);
	return p_RearrangedData;
}

unsigned char * YUV422_YUYVtoYUV444(unsigned char * p_YUYVData,unsigned int intImageSize)
{
	unsigned char * p_RearrangedData;
	unsigned int PixelCounter;

	unsigned char *p_In_YData = p_YUYVData + 0;
	unsigned char *p_In_UData = p_YUYVData + 1;
	unsigned char *p_In_VData = p_YUYVData + 3;
	
	unsigned char *p_Out_UData;
	unsigned char *p_Out_VData;
	unsigned char *p_Out_YData;

	p_RearrangedData = (unsigned char *)malloc(intImageSize*3);

	p_Out_YData = p_RearrangedData;
	p_Out_UData = p_RearrangedData + 1;
	p_Out_VData = p_RearrangedData + 2;

	for(PixelCounter = 0;PixelCounter < intImageSize;PixelCounter+=2)
	{
		*p_Out_YData = *p_In_YData;
		p_Out_YData += 3 ;
		p_In_YData += 2;
		*p_Out_YData = *p_In_YData;
		p_Out_YData += 3 ;
		p_In_YData += 2 ;

		*p_Out_UData = *p_In_UData;
		p_Out_UData += 3 ;
		*p_Out_UData = *p_In_UData;
		p_Out_UData += 3 ;
		p_In_UData += 4 ;

		*p_Out_VData = *p_In_VData;
		p_Out_VData += 3 ;
		*p_Out_VData = *p_In_VData;
		p_Out_VData += 3 ;
		p_In_VData += 4 ;
	}

	free(p_YUYVData);
	return p_RearrangedData;
}

void YUYVtoUYVY(const unsigned char *ptr_u8YUYV, unsigned char *ptr_u8UYVY,unsigned int u32ImageSize)
{
	unsigned char * p_RearrangedData;
	unsigned int PixelCounter;

	unsigned char *p_In_Y1Data = ptr_u8YUYV + 0;
	unsigned char *p_In_UData = ptr_u8YUYV + 1;
	unsigned char *p_In_Y2Data = ptr_u8YUYV + 2;
	unsigned char *p_In_VData = ptr_u8YUYV + 3;
	
	unsigned char *p_Out_UData= ptr_u8UYVY + 0;
	unsigned char *p_Out_Y1Data= ptr_u8UYVY + 1;
	unsigned char *p_Out_VData= ptr_u8UYVY + 2;
	unsigned char *p_Out_Y2Data= ptr_u8UYVY + 3;


	for(PixelCounter = 0;PixelCounter < u32ImageSize;PixelCounter++)
	{
		*p_Out_UData = *p_In_UData;
		*p_Out_Y1Data = *p_In_Y1Data;
		*p_Out_VData = *p_In_VData;
		*p_Out_Y2Data = *p_In_Y2Data;
		
		p_Out_UData += 4; 
		p_In_UData += 4;
		p_Out_Y1Data += 4;
		p_In_Y1Data += 4;
		p_Out_VData += 4;
		p_In_VData += 4;
		p_Out_Y2Data += 4;
		p_In_Y2Data += 4;		
	}
	return p_RearrangedData;
}
/***********************
* Module		: YUV422_YUYVtoRGB888_BGR()
* Description	: convert yuv422 interleaved to RGB interleaved	
***********************/
void YUV422_YUYVtoRGB888_BGR(const unsigned char *ptr_u8YUV, unsigned char *ptr_u8BGR,
			unsigned int u32ImageSize)
{

	#define SATURATE(x)					(((x) > 255) ? (255) : (((x) < 0) ? (0) : (x)))

	unsigned char *ptr_u8Out_RGB = (unsigned char *)ptr_u8BGR;
    unsigned char *ptr_u8In_YUV = (unsigned char *)ptr_u8YUV;
	
	double RData0, RData1;
	double GData0, GData1;
	double BData0, BData1;
	
	double YData0, YData1;
	double UData, VData;

	unsigned long u32loopCounter;
	
	u32ImageSize = (u32ImageSize * 3);	//RGB image size.
	
	/*Loop mapping RGB to YUV image.*/
    for(u32loopCounter = 0; u32loopCounter < u32ImageSize; u32loopCounter += 6)
	{
        YData1 = (*ptr_u8In_YUV);
		ptr_u8In_YUV++;
		UData = (*ptr_u8In_YUV);
		ptr_u8In_YUV++;
		YData0 = (*ptr_u8In_YUV);
		ptr_u8In_YUV++;
		VData = (*ptr_u8In_YUV);
		ptr_u8In_YUV++;
		

		RData0 = YData0 + 0.0000000 * (UData - 128) + 1.1398320 * (VData - 128);
		GData0 = YData0 - 0.3946500 * (UData - 128) - 0.5806000 * (VData - 128);
		BData0 = YData0 + 2.0321120 * (UData - 128) + 0.0000000 * (VData - 128);
		
		RData1 = YData1 + 0.0000000 * (UData - 128) + 1.1398320 * (VData - 128);
		GData1 = YData1 - 0.3946500 * (UData - 128) - 0.5806000 * (VData - 128);
		BData1 = YData1 + 2.0321120 * (UData - 128) + 0.0000000 * (VData - 128);
	
		*(unsigned char *)ptr_u8Out_RGB = (unsigned char)SATURATE(BData0);	
		(ptr_u8Out_RGB)++;
		*(unsigned char *)ptr_u8Out_RGB = (unsigned char)SATURATE(GData0);	
		(ptr_u8Out_RGB)++;
		*(unsigned char *)ptr_u8Out_RGB = (unsigned char)SATURATE(RData0);	
		(ptr_u8Out_RGB)++;
		
		*(unsigned char *)ptr_u8Out_RGB = (unsigned char)SATURATE(BData1);	
		(ptr_u8Out_RGB)++;
		*(unsigned char *)ptr_u8Out_RGB = (unsigned char)SATURATE(GData1);	
		(ptr_u8Out_RGB)++;
		*(unsigned char *)ptr_u8Out_RGB = (unsigned char)SATURATE(RData1);	
		(ptr_u8Out_RGB)++;
		
	}
}

/***********************
* Module		: YUV444_YUVtoRGB888_BGR()
* Description	: convert yuv444 interleaved to RGB interleaved	
***********************/
void YUV444_YUVtoRGB888_BGR(const unsigned char *ptr_u8YUV, unsigned char *ptr_u8BGR,
			unsigned int u32ImageSize)
{

	#define SATURATE(x)					(((x) > 255) ? (255) : (((x) < 0) ? (0) : (x)))

	unsigned char *ptr_u8Out_RGB = (unsigned char *)ptr_u8BGR;
    unsigned char *ptr_u8In_YUV = (unsigned char *)ptr_u8YUV;
	
	double RData;
	double GData;
	double BData;
	
	double YData,UData, VData;

	unsigned long u32loopCounter;
	
	u32ImageSize = (u32ImageSize * 3);	//RGB image size.
	
	/*Loop mapping RGB to YUV image.*/
    for(u32loopCounter = 0; u32loopCounter < u32ImageSize; u32loopCounter += 3)
	{
        YData = (*ptr_u8In_YUV);
		ptr_u8In_YUV++;
		UData = (*ptr_u8In_YUV);
		ptr_u8In_YUV++;
		VData = (*ptr_u8In_YUV);
		ptr_u8In_YUV++;
		
		RData = YData + 0.0000000 * (UData - 128) + 1.1398320 * (VData - 128);
		GData = YData - 0.3946500 * (UData - 128) - 0.5806000 * (VData - 128);
		BData = YData + 2.0321120 * (UData - 128) + 0.0000000 * (VData - 128);
		
		*(unsigned char *)ptr_u8Out_RGB = (unsigned char)SATURATE(BData);	
		(ptr_u8Out_RGB)++;
		*(unsigned char *)ptr_u8Out_RGB = (unsigned char)SATURATE(GData);	
		(ptr_u8Out_RGB)++;
		*(unsigned char *)ptr_u8Out_RGB = (unsigned char)SATURATE(RData);	
		(ptr_u8Out_RGB)++;
		
	}
}
/***********************
* Module		: ADAS_YUV444_YUVtoYUV422_YUYV()
* Description	: convert yuv444 interleaved to YUV422_YUYV interleaved	
***********************/
void ADAS_YUV444_YUVtoYUV422_YUYV(unsigned char * InputYUV,unsigned char * OutputYUYV,unsigned int ImageSize)
{
	

	for( ; ImageSize >= 3 ; ImageSize-=2)
	{
		//Y
		*OutputYUYV = *InputYUV;
		OutputYUYV++;
		InputYUV++;

		//U
		*OutputYUYV = *InputYUV;
		OutputYUYV++;
		InputYUV++;

		//V
		InputYUV++;

		//Y
		*OutputYUYV = *InputYUV;
		OutputYUYV++;
		InputYUV++;

		//U
		InputYUV++;

		//V
		*OutputYUYV = *InputYUV;
		OutputYUYV++;
		InputYUV++;

		//printf("%d\n",ImageSize);

	}

}

/***********************
* Module		: YUV422_YUYVtoY()
* Description	: convert yuv422 interleaved to RGB interleaved	
***********************/
void YUV422_YUYVtoY(const unsigned char *ptr_u8YUV, unsigned char *ptr_u8Y,
			unsigned int u32ImageSize)
{

	
	unsigned char *ptr_u8Out_Y = (unsigned char *)ptr_u8Y;
    unsigned char *ptr_u8In_YUV = (unsigned char *)ptr_u8YUV;
	
	unsigned long u32loopCounter;
	
	/*Loop mapping y to YUV image.*/
    for(u32loopCounter = 0; u32loopCounter < u32ImageSize; u32loopCounter += 2)
	{
        *(unsigned char *)ptr_u8Out_Y = (*ptr_u8In_YUV);
		ptr_u8In_YUV++;
		ptr_u8In_YUV++;
		(ptr_u8Out_Y)++;
		
		*(unsigned char *)ptr_u8Out_Y = (*ptr_u8In_YUV);
		ptr_u8In_YUV++;
		ptr_u8In_YUV++;		
		(ptr_u8Out_Y)++;
		
		
	}
}

/***********************
* Module		: YUV422_UYVYtoY()
* Description	: convert yuv422 interleaved to RGB interleaved	
***********************/
void YUV422_UYVYtoY(const unsigned char *ptr_u8YUV, unsigned char *ptr_u8Y,
			unsigned int u32ImageSize)
{

	
	unsigned char *ptr_u8Out_Y = (unsigned char *)ptr_u8Y;
    unsigned char *ptr_u8In_YUV = (unsigned char *)ptr_u8YUV;
	
	unsigned long u32loopCounter;
	
	/*Loop mapping y to YUV image.*/
    for(u32loopCounter = 0; u32loopCounter < u32ImageSize; u32loopCounter += 2)
	{
        ptr_u8In_YUV++;
		*(unsigned char *)ptr_u8Out_Y = (*ptr_u8In_YUV);
		(ptr_u8Out_Y)++;
		ptr_u8In_YUV++;
		ptr_u8In_YUV++;
		*(unsigned char *)ptr_u8Out_Y = (*ptr_u8In_YUV);
		(ptr_u8Out_Y)++;
		ptr_u8In_YUV++;		
		
	}
}
/***********************
* Module		: YUV422_UYVYtoYUV422_YUYV()
* Description	: convert YUV422 UYVY to YUV422 YUYV
***********************/
void YUV422_UYVYtoYUV422_YUYV(const unsigned char *ptr_UYVY, unsigned char *ptr_u8YUYV,
			unsigned int u32ImageSize)
{
	unsigned long u32loopCounter;
	unsigned char * ptr_inU;
	unsigned char * ptr_inV;
	for(u32loopCounter = 0; u32loopCounter < u32ImageSize ; u32loopCounter+=4)
	{
		ptr_inU = ptr_UYVY++;
		*(unsigned char *)ptr_u8YUYV =  *ptr_UYVY;
		ptr_u8YUYV++;
		ptr_UYVY++;
		*(unsigned char *)ptr_u8YUYV = *ptr_inU;
		ptr_u8YUYV++;
		ptr_inV = ptr_UYVY;
		ptr_UYVY++;
		*(unsigned char *)ptr_u8YUYV = *ptr_UYVY;
		ptr_u8YUYV++ ;
		ptr_UYVY++;
		*(unsigned char *)ptr_u8YUYV = *ptr_inV;
		ptr_u8YUYV++;

	}
}

/***********************
* Module		: RCCC16ToGrey8()
* Description	: convert RCCC16 to 8bit Grey	
***********************/

void RCCC16ToGrey8(unsigned char* pch_RCCC16,unsigned char* pch_Grey8,uint32_t u32_ImageSize)
{
	uint32_t u32_Pixel = 0;
	unsigned char data1= 0x00,data2= 0x00;
	unsigned short u16_Data = 0;

	for(u32_Pixel = 0; u32_Pixel < u32_ImageSize; u32_Pixel++)
	{
		data1 = (pch_RCCC16[2*u32_Pixel] & 0x00FF);
		data2 = (pch_RCCC16[2*u32_Pixel+1] & 0x00ff) ;
		
		u16_Data = data2 | (data1 << 8); 
		u16_Data = u16_Data >> 2; 

		pch_Grey8[u32_Pixel] = (unsigned char)(u16_Data);
	}
}

en_image_save_type_t FindTypeFromFileName(const char * const Filename,char * const p_chExtension)
{
	int u32PathLength;
	char * p_chLocalExtension;
	en_image_save_type_t enLocalSaveType;
	

	u32PathLength = strlen(Filename);

	if(p_chExtension != NULL)
	{
		p_chLocalExtension = p_chExtension;
	}
	else
	{
		p_chLocalExtension = (char *)malloc(8);
	}


 /* Find extension and type */
	if(('.' == Filename[u32PathLength - 2]) && (u32PathLength >= 3)) 
	{
		strncpy(p_chLocalExtension,&(Filename[u32PathLength - 1]),3);
		if(!strcmp(p_chLocalExtension,"h"))
        {
            enLocalSaveType = EN_SAVE_CONFIG;
        }else 
        {
            printf("Unsupported extension: %s\n",p_chLocalExtension);
			enLocalSaveType = EN_UNKNOWN_IMAGE_SAVE_TYPE;
        }
	}
    else if(('.' == Filename[u32PathLength - 3])  && (u32PathLength >= 4)) 
    {
		strncpy(p_chLocalExtension,&(Filename[u32PathLength - 2]),3);
        
        if(!strcmp(p_chLocalExtension,"iz"))
        {
            enLocalSaveType = EN_SAVE_IZ;
        }else if(!strcmp(p_chLocalExtension,"iy"))
        {
            enLocalSaveType = EN_SAVE_IYUV;
        }else if(!strcmp(p_chLocalExtension,"ig"))
        {
            enLocalSaveType = EN_SAVE_IGREY;
        }else if(!strcmp(p_chLocalExtension,"ir"))
        {
            enLocalSaveType = EN_SAVE_IRCCC;
        }else if(!strcmp(p_chLocalExtension,"7z"))
        {
            enLocalSaveType = EN_SAVE_7Z;
        }else 
        {
            printf("Unsupported extension: %s\n",p_chLocalExtension);
			enLocalSaveType = EN_UNKNOWN_IMAGE_SAVE_TYPE;
        }
    }
	else if(('.' == Filename[u32PathLength - 4]) && (u32PathLength >= 5)) 
    {
        strncpy(p_chLocalExtension,&(Filename[u32PathLength - 3]),4);
       
        if(!strcmp(p_chLocalExtension,"abf"))
        {
            enLocalSaveType = EN_SAVE_ADAS_BATCH_FILE;
        }else if(!strcmp(p_chLocalExtension,"bmp"))
        {
            enLocalSaveType = EN_SAVE_BMP;
        }else if(!strcmp(p_chLocalExtension,"raw"))
        {
            enLocalSaveType = EN_SAVE_RAW;
        }else if(!strcmp(p_chLocalExtension,"png"))
        {
            enLocalSaveType = EN_SAVE_PNG;
        }else if(!strcmp(p_chLocalExtension,"jpg"))
        {
            enLocalSaveType = EN_SAVE_JPG;
        }else if(!strcmp(p_chLocalExtension,"avi"))
        {
            enLocalSaveType = EN_SAVE_AVI;
        }else if(!strcmp(p_chLocalExtension,"ray"))
        {
            enLocalSaveType = EN_SAVE_RAW_YUV;
        }else if(!strcmp(p_chLocalExtension,"r16"))
        {
            enLocalSaveType = EN_SAVE_RAW_RCCC16;
        }else if(!strcmp(p_chLocalExtension,"yuv"))  
        {
			enLocalSaveType = EN_SAVE_RAW_YUV420_YUYV;
        }else if(!strcmp(p_chLocalExtension,"uyv"))
        {
			enLocalSaveType = EN_SAVE_RAW_YUV420_UYVY;
        }else if(!strcmp(p_chLocalExtension,"can"))
        {
			enLocalSaveType = EN_SAVE_CAN;
        }else if(!strcmp(p_chLocalExtension,"csv"))
        {
			enLocalSaveType = EN_SAVE_CSV;
        }else if(!strcmp(p_chLocalExtension,"txt"))
        {
			enLocalSaveType = EN_SAVE_TXT;
        }else if(!strcmp(p_chLocalExtension,"lst"))
        {
			enLocalSaveType = EN_SAVE_LST;
        }else 
        {
            printf("Unsupported extension: %s\n",p_chLocalExtension);
			enLocalSaveType = EN_UNKNOWN_IMAGE_SAVE_TYPE;
        }
    }else if(('.' == Filename[u32PathLength - 5]) && (u32PathLength >= 6)) 
    {
       strncpy(p_chLocalExtension,&(Filename[u32PathLength - 4]),5);
       
        if(!strcmp(p_chLocalExtension,"jpeg"))
        {
            enLocalSaveType = EN_SAVE_JPEG;
        }else
        {
            printf("Unsupported extension: %s\n",p_chLocalExtension);
			enLocalSaveType = EN_UNKNOWN_IMAGE_SAVE_TYPE;
        }
    }else
    {
        printf("Extension not found.\n");

		enLocalSaveType = EN_UNKNOWN_IMAGE_SAVE_TYPE;
    }

	return enLocalSaveType;
}
#if 0
en_image_save_type_t FindTypeFromExtension(const char * const p_chInputExtension)
{
	en_image_save_type_t enLocalSaveType = EN_UNKNOWN_IMAGE_SAVE_TYPE;
	
	if(NULL != p_chInputExtension)
	{	
		/* Find extension and type */
		if(!strcmp(p_chInputExtension,"h"))
		{
			enLocalSaveType = EN_SAVE_CONFIG;
		}else if(!strcmp(p_chInputExtension,"iz"))
		{
			enLocalSaveType = EN_SAVE_IZ;
		}else if(!strcmp(p_chInputExtension,"iy"))
		{
			enLocalSaveType = EN_SAVE_IYUV;
		}else if(!strcmp(p_chInputExtension,"7z"))
		{
			enLocalSaveType = EN_SAVE_7Z;
		}else if(!strcmp(p_chInputExtension,"ig"))
		{
			enLocalSaveType = EN_SAVE_IGREY;
		}else if(!strcmp(p_chInputExtension,"ir"))
		{
			enLocalSaveType = EN_SAVE_IRCCC;
		}else if(!strcmp(p_chInputExtension,"r16"))
		{
			enLocalSaveType = EN_SAVE_RAW_RCCC16;
		}else if(!strcmp(p_chInputExtension,"abf"))
		{
			enLocalSaveType = EN_SAVE_ADAS_BATCH_FILE;
		}else if(!strcmp(p_chInputExtension,"bmp"))
		{
			enLocalSaveType = EN_SAVE_BMP;
		}else if(!strcmp(p_chInputExtension,"raw"))
		{
			enLocalSaveType = EN_SAVE_RAW;
		}else if(!strcmp(p_chInputExtension,"png"))
		{
			enLocalSaveType = EN_SAVE_PNG;
		}else if(!strcmp(p_chInputExtension,"jpg"))
		{
			enLocalSaveType = EN_SAVE_JPG;
		}else if(!strcmp(p_chInputExtension,"avi"))
		{
			enLocalSaveType = EN_SAVE_AVI;
		}else if(!strcmp(p_chInputExtension,"jpeg"))
		{
			enLocalSaveType = EN_SAVE_JPEG;
		}
	}

	if(EN_UNKNOWN_IMAGE_SAVE_TYPE == enLocalSaveType)
	{
		printf("Unsupported extension: %s\n",p_chInputExtension);
	}

	return enLocalSaveType;
}


char * ADAS_StringClone(const char * const p_chInputString)
{
	char * p_chClonedString = NULL;
	uint16_t u16_StringLength;
	if(NULL != p_chInputString)
	{
		u16_StringLength = (uint16_t)strlen(p_chInputString);
		p_chClonedString = (char * )malloc(u16_StringLength + 1);
		strcpy_s(p_chClonedString, u16_StringLength+1, p_chInputString);
	}
	return p_chClonedString;
}

unsigned int ADAS_ParseYesNo(char *chString)
{
	if ((strcmp((char *)chString,"Yes")) == 0)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

unsigned int ADAS_ParseTrueFalse(char *chString)
{
	if ((strcmp((char *)chString,"True")) == 0)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}


void ADAS_CreateDir(char const * const Path)
{
 char DirName[256];
 const char * p = Path;
 char* q = DirName; 
 while(*p)
 {
   if (('\\' == *p) || ('/' == *p))
   {
     if (':' != *(p-1))
     {
        CreateDirectory(DirName, NULL);
     }
   }
   *q++ = *p++;
   *q = '\0';
 }
 CreateDirectory(DirName, NULL);
}

void ADAS_CreateDirOfFile(char const * const FilePath)
{
 char DirName[256];
 const char * p = FilePath;
 char* q = DirName; 
 while(*p)
 {
   if (('\\' == *p) || ('/' == *p))
   {
     if (':' != *(p-1))
     {
        CreateDirectory(DirName, NULL);
     }
   }
   *q++ = *p++;
   *q = '\0';
 }
 //CreateDirectory(DirName, NULL);
}

void SleepInSeconds(uint32_t u32Seconds)
{
	uint32_t u32SecondsCounter;

	for (u32SecondsCounter = 0; u32SecondsCounter < u32Seconds*100 ; u32SecondsCounter++)
	{
		if(kbhit())
		{
			_getch();
			break;
		}
		Sleep(10);
		if(u32SecondsCounter%100 == 0)
			_putch('.');
	}
	_putch('\n');
}

void ExitInSeconds(uint32_t u32Seconds)
{
	printf("Application will exit in %d seconds...\n",u32Seconds);
	SleepInSeconds(u32Seconds);
	exit(-1);
}

st_init_thread_arguments_t * ADAS_CreateThreadArgsPipe(st_init_thread_arguments_t * pst_init_thread_arguments)
{
	HANDLE CommandPipe_ReadHandler;
	HANDLE CommandPipe_WriteHandler;

	HANDLE ConfigPipe_ReadHandler;
	HANDLE ConfigPipe_WriteHandler;

	HANDLE StreamPipe_ReadHandler;
	HANDLE StreamPipe_WriteHandler;

	HANDLE FeedbackPipe_ReadHandler;
	HANDLE FeedbackPipe_WriteHandler;

	//st_init_thread_arguments_t * pst_init_thread_arguments;

	pst_init_thread_arguments = (st_init_thread_arguments_t*)malloc(sizeof(st_init_thread_arguments_t));
	pst_init_thread_arguments->pst_Initiator = (st_init_thread_arguments_initiator_t*)malloc(sizeof(st_init_thread_arguments_initiator_t));
	pst_init_thread_arguments->pst_Responder = (st_init_thread_arguments_responder_t*)malloc(sizeof(st_init_thread_arguments_responder_t));

	if(pst_init_thread_arguments != NULL)
	{
		if(0 == CreatePipe(&CommandPipe_ReadHandler,&CommandPipe_WriteHandler,0,0))
		{
			printf("Create CapturePipeME pipe failed.\n");
			pst_init_thread_arguments = NULL;
		}else
		{
			pst_init_thread_arguments->pst_Responder->CommandPipe_ReadHandler = CommandPipe_ReadHandler;
			pst_init_thread_arguments->pst_Initiator->CommandPipe_WriteHandler = CommandPipe_WriteHandler;
		}
	}
	
	if(pst_init_thread_arguments != NULL)
	{
		if(0 == CreatePipe(&ConfigPipe_ReadHandler,&ConfigPipe_WriteHandler,0,0))
		{
			printf("Create CapturePipe pipe failed.\n");
			pst_init_thread_arguments = NULL;
		}else
		{
			pst_init_thread_arguments->pst_Responder->ConfigPipe_ReadHandler = ConfigPipe_ReadHandler;
			pst_init_thread_arguments->pst_Initiator->ConfigPipe_WriteHandler = ConfigPipe_WriteHandler;
		}
	}
	
	if(pst_init_thread_arguments != NULL)
	{
		if(0 == CreatePipe(&StreamPipe_ReadHandler,&StreamPipe_WriteHandler,0,64*1024))
		{
			printf("Create CapturePipeME pipe failed.\n");
			pst_init_thread_arguments = NULL;
		}else
		{
			pst_init_thread_arguments->pst_Responder->StreamPipe_ReadHandler = StreamPipe_ReadHandler;
			pst_init_thread_arguments->pst_Responder->StreamPipe_WriteHandler = StreamPipe_WriteHandler;
			pst_init_thread_arguments->pst_Initiator->StreamPipe_WriteHandler = StreamPipe_WriteHandler;
			pst_init_thread_arguments->pst_Initiator->StreamPipe_ReadHandler = StreamPipe_ReadHandler;
		}
	}
	
	if(pst_init_thread_arguments != NULL)
	{
		if(0 == CreatePipe(&FeedbackPipe_ReadHandler,&FeedbackPipe_WriteHandler,0,0))
		{
			printf("Create FeedbackPipe failed.\n");
			pst_init_thread_arguments = NULL;
		}else
		{
			pst_init_thread_arguments->pst_Initiator->FeedbackPipe_ReadHandler = FeedbackPipe_ReadHandler;
			pst_init_thread_arguments->pst_Responder->FeedbackPipe_WriteHandler = FeedbackPipe_WriteHandler;
		}
	}
	if(pst_init_thread_arguments == NULL)
	{
		RELEASE_POINTER_IFNOT_NULL(pst_init_thread_arguments->pst_Initiator);
		RELEASE_POINTER_IFNOT_NULL(pst_init_thread_arguments->pst_Responder);
		RELEASE_POINTER_IFNOT_NULL(pst_init_thread_arguments);
		pst_init_thread_arguments = NULL;
	}
	return pst_init_thread_arguments;
}


void ADAS_WritedataToThreadArgsStreamPipe(void * pv_Indata,st_init_thread_arguments_t * pst_init_thread_arguments,int DataSize)
{
	int BytesRead = 0;
	int TotalBytesAvail = 0;

	PeekNamedPipe(pst_init_thread_arguments->pst_Responder->StreamPipe_ReadHandler,NULL,0,NULL,(LPDWORD)&TotalBytesAvail,NULL);

	if( TotalBytesAvail <= 500)
	{
	}else
	{
		Sleep(5);
	}

	TotalBytesAvail = 0;
	
	/*Write Data to Stream Pipe*/
	WriteFile(pst_init_thread_arguments->pst_Initiator->StreamPipe_WriteHandler,
		&pv_Indata,
		DataSize,
		(LPDWORD)&BytesRead,NULL);
}

void ADAS_ReleaseThreadArgsPipe(st_init_thread_arguments_t * pst_init_thread_arguments)
{
	if(NULL != pst_init_thread_arguments)
	{
		RELEASE_POINTER_IFNOT_NULL(pst_init_thread_arguments->pst_Initiator);
		RELEASE_POINTER_IFNOT_NULL(pst_init_thread_arguments->pst_Responder);
		RELEASE_POINTER_IFNOT_NULL(pst_init_thread_arguments);
	}
}

#endif
