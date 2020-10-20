/*
 * Filename             : streamer.cpp
 * Description          : Provides functions to test USB data transfer performance.
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/time.h>
#include "ADAS_Support.h"
#include <libusb-1.0/libusb.h>
#include <signal.h>
#include <stdlib.h>
#include "../include/cyusb.h"

using namespace std;

//extern ControlCenter *mainwin;
cyusb_handle *h;

// Variables storing the user provided application configuration.
static unsigned int endpoint = 0;	 // Endpoint to be tested
static unsigned int reqsize = 16;	 // Request size in number of packets
static unsigned int queuedepth = 16; // Number of requests to queue
static unsigned char eptype;		 // Type of endpoint (transfer type)
static unsigned int pktsize;		 // Maximum packet size for the endpoint

static unsigned int success_count = 0;		 // Number of successful transfers
static unsigned int failure_count = 0;		 // Number of failed transfers
static unsigned int transfer_size = 0;		 // Size of data transfers performed so far
static unsigned int transfer_index = 0;		 // Write index into the transfer_size array
static unsigned int transfer_perf = 0;		 // Performance in KBps
static volatile bool stop_transfers = false; // Request to stop data transfers
static volatile int rqts_in_flight = 0;		 // Number of transfers that are in progress
static volatile bool app_running = false;	 // Whether the streamer application is running
static pthread_t strm_thread;				 // Thread used for the streamer operation

static unsigned int u32_ImageWidth = 1280;
static unsigned int u32_ImageHeight = 960;

static struct timeval start_ts; // Data transfer start time stamp.
static struct timeval end_ts;	// Data transfer stop time stamp.

unsigned char *UYVYData = NULL;

// Function: streamer_set_params
// Sets the streamer test parameters
void streamer_set_params(
	unsigned int ep,
	unsigned int type,
	unsigned int maxpkt,
	unsigned int numpkts,
	unsigned int numrqts)
{
	endpoint = ep;
	eptype = type;
	pktsize = maxpkt;
	reqsize = numpkts;
	queuedepth = numrqts;
}

// Function: streamer_stop_xfer
// Requests the streamer operation to be stopped.
void streamer_stop_xfer(
	void)
{
	stop_transfers = true;
}

// Function: streamer_is_running
// Checks whether the streamer operation is running.
bool streamer_is_running(
	void)
{
	return app_running;
}

// Function: streamer_update_results
// Gets the streamer test results on an ongoing basis
static void
streamer_update_results(
	void)
{
	char buffer[64];

	// Print the transfer statistics into the character strings and update UI.
	sprintf(buffer, "%d", success_count);
	//mainwin->streamer_out_passcnt->setText (buffer);

	sprintf(buffer, "%d", failure_count);
	//mainwin->streamer_out_failcnt->setText (buffer);

	sprintf(buffer, "%d", transfer_perf);
	//mainwin->streamer_out_perf->setText (buffer);
}

// Function: xfer_callback
// This is the call back function called by libusb upon completion of a queued data transfer.
static void
xfer_callback(
	struct libusb_transfer *transfer)
{
	unsigned int elapsed_time;
	double performance;
	int size = 0;

	// Check if the transfer has succeeded.
	if (transfer->status != LIBUSB_TRANSFER_COMPLETED)
	{

		failure_count++;
	}
	else
	{

		if (eptype == LIBUSB_TRANSFER_TYPE_ISOCHRONOUS)
		{

			// Loop through all the packets and check the status of each packet transfer
			for (unsigned int i = 0; i < reqsize; ++i)
			{

				// Calculate the actual size of data transferred in each micro-frame.
				if (transfer->iso_packet_desc[i].status == LIBUSB_TRANSFER_COMPLETED)
				{
					size += transfer->iso_packet_desc[i].actual_length;
				}
			}
		}
		else
		{
			size = reqsize * pktsize;
		}

		success_count++;
	}

	// Update the actual transfer size for this request.
	transfer_size += size;

	// Print the transfer statistics when queuedepth transfers are completed.
	transfer_index++;
	if (transfer_index == queuedepth)
	{

		gettimeofday(&end_ts, NULL);
		elapsed_time = ((end_ts.tv_sec - start_ts.tv_sec) * 1000000 +
						(end_ts.tv_usec - start_ts.tv_usec));

		// Calculate the performance in KBps.
		performance = (((double)transfer_size / 1024) / ((double)elapsed_time / 1000000));
		transfer_perf = (unsigned int)performance;

		transfer_index = 0;
		transfer_size = 0;
		start_ts = end_ts;
	}

	// Reduce the number of requests in flight.
	rqts_in_flight--;

	// Prepare and re-submit the read request.
	if (!stop_transfers)
	{

		// We do not expect a transfer queue attempt to fail in the general case.
		// However, if it does fail; we just ignore the error.
		switch (eptype)
		{
		case LIBUSB_TRANSFER_TYPE_BULK:
			if (libusb_submit_transfer(transfer) == 0)
				rqts_in_flight++;
			break;

		case LIBUSB_TRANSFER_TYPE_INTERRUPT:
			if (libusb_submit_transfer(transfer) == 0)
				rqts_in_flight++;
			break;

		case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
			libusb_set_iso_packet_lengths(transfer, pktsize);
			if (libusb_submit_transfer(transfer) == 0)
				rqts_in_flight++;
			break;

		default:
			break;
		}
	}
}

// Function to free data buffers and transfer structures
static void
free_transfer_buffers(
	unsigned char **databuffers,
	struct libusb_transfer **transfers)
{
	// Free up any allocated transfer structures
	if (transfers != NULL)
	{
		for (unsigned int i = 0; i < queuedepth; i++)
		{
			if (transfers[i] != NULL)
			{
				libusb_free_transfer(transfers[i]);
			}
			transfers[i] = NULL;
		}
		free(transfers);
	}

	// Free up any allocated data buffers
	if (databuffers != NULL)
	{
		for (unsigned int i = 0; i < queuedepth; i++)
		{
			if (databuffers[i] != NULL)
			{
				free(databuffers[i]);
			}
			databuffers[i] = NULL;
		}
		free(databuffers);
	}
}

// Function: streamer_thread_func
// Function that implements the main streamer functionality. This will run on a dedicated thread
// created for the streamer operation.
static void *
streamer_thread_func(
	void *arg)
{
	cyusb_handle *dev_handle = (cyusb_handle *)arg;
	struct libusb_transfer **transfers = NULL; // List of transfer structures.
	unsigned char **databuffers = NULL;		   // List of data buffers.
	static unsigned int FrameCount = 0;
	unsigned int nBytesRead = 0, ImageSize = 0;
	int rStatus;
	struct timeval t1, t2, tout;
	double TimeDiff = 0;
	unsigned int FPSAveragingWindow = 200;
	double FPS_Value = 0.0;

	// Check for validity of the device handle
	if (dev_handle == NULL)
	{
		printf("Failed to get CyUSB device handle\n");
		pthread_exit(NULL);
	}

	// The endpoint is already found and its properties are known.
	printf("Starting test with the following parameters\n");
	printf("\tEndpoint to test : 0x%x\n", endpoint);
	printf("\tEndpoint type    : 0x%x\n", eptype);
	printf("\tMax packet size  : 0x%x\n", pktsize);
	printf("\tRequest size     : 0x%x\n", reqsize);
	printf("\tQueue depth      : 0x%x\n", queuedepth);
	printf("\n");
	int fd;
	char *myfifo = "/tmp/myfifo";
	// Creating the named file(FIFO)
	// mkfifo(<pathname>, <permission>)
	mkfifo(myfifo, 0666);
	// Open FIFO for read only
	fd = open(myfifo, O_RDONLY);
	if (fd == -1)
	{
		printf("Streamer : Named Pipe %s Not Opened\n");
		printf("Streamer : Open Named File and then restart this application\n");
		exit(1);
	}
	// Allocate buffers and transfer structures
	bool allocfail = false;

	databuffers = (unsigned char **)calloc(queuedepth, sizeof(unsigned char *));

	transfers = (struct libusb_transfer **)calloc(queuedepth, sizeof(struct libusb_transfer *));

	if ((databuffers != NULL) && (transfers != NULL))
	{

		for (unsigned int i = 0; i < queuedepth; i++)
		{

			databuffers[i] = (unsigned char *)malloc(reqsize * pktsize);
			transfers[i] = libusb_alloc_transfer(
				(eptype == LIBUSB_TRANSFER_TYPE_ISOCHRONOUS) ? reqsize : 0);

			if ((databuffers[i] == NULL) || (transfers[i] == NULL))
			{
				allocfail = true;
				break;
			}
		}
	}
	else
	{

		allocfail = true;
	}

	// Check if all memory allocations have succeeded
	if (allocfail)
	{
		printf("Failed to allocate buffers and transfer structures\n");
		free_transfer_buffers(databuffers, transfers);
		pthread_exit(NULL);
	}

	// Take the transfer start timestamp
	gettimeofday(&start_ts, NULL);

	ImageSize = u32_ImageWidth * u32_ImageHeight * 2;
	gettimeofday(&t1, NULL);

	while (!stop_transfers)
	{
		// Launch all the transfers till queue depth is complete
		read(fd, UYVYData, 1280 * 960 * 2);
		for (unsigned int i = 0; i < queuedepth; i++)
		{

			switch (eptype)
			{
			case LIBUSB_TRANSFER_TYPE_BULK:
				FrameCount++;
				memcpy(databuffers[i], UYVYData, reqsize * pktsize);
				libusb_fill_bulk_transfer(transfers[i], dev_handle, endpoint,
										  databuffers[i], reqsize * pktsize, xfer_callback, NULL, 1500);
				rStatus = libusb_submit_transfer(transfers[i]);
				if (rStatus == 0)
					rqts_in_flight++;
				break;

			case LIBUSB_TRANSFER_TYPE_INTERRUPT:
				libusb_fill_interrupt_transfer(transfers[i], dev_handle, endpoint,
											   databuffers[i], reqsize * pktsize, xfer_callback, NULL, 5000);
				rStatus = libusb_submit_transfer(transfers[i]);
				if (rStatus == 0)
					rqts_in_flight++;
				break;

			case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
				FrameCount++;
				memcpy(databuffers[i], UYVYData, reqsize * pktsize);
				libusb_fill_iso_transfer(transfers[i], dev_handle, endpoint, databuffers[i],
										 reqsize * pktsize, reqsize, xfer_callback, NULL, 5000);
				libusb_set_iso_packet_lengths(transfers[i], pktsize);
				rStatus = libusb_submit_transfer(transfers[i]);
				if (rStatus == 0)
					rqts_in_flight++;
				break;

			default:
				break;
			}
		}

		//printf ("Queued %d requests\n", rqts_in_flight);

		if (FrameCount == FPSAveragingWindow)
		{
			gettimeofday(&t2, NULL);
			TimeDiff = ((t2.tv_sec - t1.tv_sec) * 1000000L + (t2.tv_usec - t1.tv_usec));
			//printf("Time in MicroSeconds: %.0lf MicroSeconds\n",TimeDiff);
			FrameCount = 0;
			FPS_Value = ((FPSAveragingWindow * 1000000) / TimeDiff);
			//printf("FPS %.2lf\n",FPS_Value);
			gettimeofday(&t1, NULL);
		}
		// Use a 1 second timeout for the libusb_handle_events_timeout call
		tout.tv_sec = 1;
		tout.tv_usec = 0;
		usleep(5000);
#if 1
		// Keep handling events until transfer stop is requested.
		/*do*/ {
			libusb_handle_events_timeout(NULL, &tout);

			// Refresh the performance statistics about once in 0.5 seconds.
			//gettimeofday (&t2, NULL);
			//if (t2.tv_sec > t1.tv_sec) {
			//	streamer_update_results ();
			//	t1 = t2;
			//}

		} //while (!stop_transfers);
		//cvWaitKey(1);
#endif
	}
	free(UYVYData);
	close(fd);
	printf("Stopping streamer app\n");
	while (rqts_in_flight != 0)
	{
		printf("%d requests are pending\n", rqts_in_flight);
		libusb_handle_events_timeout(NULL, &tout);
		sleep(1);
	}

	free_transfer_buffers(databuffers, transfers);
	app_running = false;

	printf("Streamer test completed\n\n");
	pthread_exit(NULL);
}

// Function: streamer_start_xfer
// Function to start the streamer operation. This creates a new thread which runs the
// data transfers.
int streamer_start_xfer(
	void)
{
	if (app_running)
		return -EBUSY;

	// Default initialization for variables
	success_count = 0;
	failure_count = 0;
	transfer_index = 0;
	transfer_size = 0;
	transfer_perf = 0;
	rqts_in_flight = 0;
	stop_transfers = false;

	// Mark application running
	app_running = true;
	if (pthread_create(&strm_thread, NULL, streamer_thread_func, (void *)h) != 0)
	{
		app_running = false;
		return -ENOMEM;
	}

	return 0;
}

void my_handler(int s)
{
	//printf("Caught signal %d\n",s);
	stop_transfers = 1;
}
/*
void* threadFunc(void *arg)
{
    int fd;
    char * myfifo = "/tmp/myfifo"; 
	
	// Creating the named file(FIFO) 
    // mkfifo(<pathname>, <permission>) 
    mkfifo(myfifo, 0666); 
    
    // Open FIFO for read only 
    fd = open(myfifo, O_RDONLY);
    if(fd == -1)
    {
        printf("Streamer : Named Pipe %s Not Opened\n");
        printf("Streamer : Open Named File and then restart this application\n");
        exit(1);
    }
    while(stop_transfers == 0)
	{
        read(fd, UYVYData, 1280*960*2); 
        /*printf("Pipe Read Done\n");*/
// usleep(30000);
//}
//free(UYVYData);
//close(fd);
//}
int main(int argc, char **argv)
{
#if 1

	const char *temp;
	unsigned int ep;
	unsigned int eptype;
	unsigned int pktsize;
	unsigned int reqsize;
	unsigned int queuedepth;
	int err;
	pthread_t tid;
	bool ok;
	int iface = 0, aiface = 0;

	unsigned int FrameSize = 1280 * 960 * 2;
	unsigned int BytesRead = 0;
	int r;
	char user_input = 'n';
	pthread_t tid1, tid2;

	struct sigaction sigIntHandler;

	sigIntHandler.sa_handler = my_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;

	sigaction(SIGINT, &sigIntHandler, NULL);

	printf("Starting Node streamer from Package pkg_streamer  \n");
	r = cyusb_open();
	if (r < 0)
	{
		printf("Error opening library\n");
		return -1;
	}
	else if (r == 0)
	{
		printf("No device found\n");
		return 0;
	}
	if (r > 1)
	{
		printf("More than 1 devices of interest found. Disconnect unwanted devices\n");
		return 0;
	}
	h = cyusb_gethandle(0);
	if (cyusb_getvendor(h) != 0x04b4)
	{
		printf("Cypress chipset not detected\n");
		cyusb_close();
		return 0;
	}
	r = cyusb_kernel_driver_active(h, 0);
	if (r != 0)
	{
		printf("kernel driver active. Exitting\n");
		cyusb_close();
		return 0;
	}
	r = cyusb_claim_interface(h, 0);
	if (r != 0)
	{
		printf("Error in claiming interface\n");
		cyusb_close();
		return 0;
	}
	else
		printf("Successfully claimed interface\n");

	UYVYData = (unsigned char *)malloc(u32_ImageWidth * u32_ImageHeight * 2);

	ep = 0x01;
	eptype = LIBUSB_TRANSFER_TYPE_BULK;
	reqsize = 1;
	queuedepth = 1;
	pktsize = (u32_ImageWidth * u32_ImageHeight * 2) / reqsize;
	streamer_set_params(ep, eptype, pktsize, reqsize, queuedepth);
	streamer_start_xfer();

	//err = pthread_create(&tid, NULL, &threadFunc, NULL);

	while (stop_transfers != 1)
	{
		sleep(1);
	}

	printf("Closing Streamer Application\n");
	streamer_stop_xfer();

	stop_transfers = 1;
	cyusb_close();
#endif
	return 0;
}

/*[]*/
