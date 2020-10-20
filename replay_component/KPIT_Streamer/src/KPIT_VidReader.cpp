#include <stdio.h> 
#include <string.h> 
#include <fcntl.h> 
#include <sys/stat.h> 
#include <sys/types.h> 
#include <unistd.h> 
#include <stdlib.h>
#include <signal.h>
#include "ADAS_Support.h"
#include <pthread.h>

static int stop_transfers = 0;
static unsigned int 	u32_ImageWidth = 1280;
static unsigned int 	u32_ImageHeight = 960;

void my_handler(int s)
{
    /*printf("Caught signal %d\n",s);*/
    stop_transfers = 1;
}

void* threadFunc(void *arg)
{

    int fd; 
    unsigned int FrameSize = 1280*960*2;
    unsigned int BytesRead = 0;
    unsigned char * UYVYData = NULL;
    unsigned char * YUYVData = NULL;
    unsigned char ach_FileName[1024];
    
    FILE * fp_InYUV = NULL;
  
    // FIFO file path 
    char * myfifo = "/tmp/myfifo"; 
    
    // Creating the named file(FIFO) 
    // mkfifo(<pathname>, <permission>) 
    mkfifo(myfifo, 0666); 
    
    // Open FIFO for write only 
    fd = open(myfifo, O_WRONLY);
    
    strcpy((char*)ach_FileName,(const char*)arg);
    fp_InYUV = fopen((const char*)ach_FileName,"rb");
	if(fp_InYUV == 0)
    {
	    printf("Error opening Fle '%s'\n",ach_FileName);
	    exit(0);
    }
    
    UYVYData = (unsigned char *)malloc(u32_ImageWidth*u32_ImageHeight*2);
	YUYVData = (unsigned char *)malloc(u32_ImageWidth*u32_ImageHeight*2); 
	
    while (stop_transfers != 1) 
    {    
        BytesRead = fread(YUYVData,FrameSize,1,fp_InYUV);
        
        if(BytesRead == 0)
        {
	        fseek(fp_InYUV, 0, SEEK_SET);
	        BytesRead = fread(YUYVData,FrameSize,1,fp_InYUV);
	        YUYVtoUYVY(YUYVData,UYVYData,1280*960/2);	 			
        }   
        /*printf("Frame Read Done\n");*/
        YUYVtoUYVY(YUYVData,UYVYData,1280*960/2);
        
  
        // Write the input arr2ing on FIFO 
        // and close it 
        write(fd, UYVYData, 1280*960*2); 
        /*printf("Data Written to Pipe\n");*/
        usleep(5000);

    } 
    fclose(fp_InYUV);
	free(UYVYData);
	free(YUYVData);
}
  
int main(int argc, char **argv) 
{ 
    pthread_t tid;
    int err;
    
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;    
    
    if(argc < 2)
    {
        printf("ERROR : Input File Name with Path is required \n");
        exit(1);
    }
    //strcpy((char*)ach_FileName,(const char*)argv[1]);
    
    err = pthread_create(&tid, NULL, &threadFunc, (void*)argv[1]);
    
    while(stop_transfers != 1)
    {
        sleep(1);
    }
	
    return 0; 
} 
