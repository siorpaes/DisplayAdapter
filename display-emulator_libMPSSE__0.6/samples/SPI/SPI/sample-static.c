/*!
 * \file sample-static.c
 *
 * \author FTDI
 * \date 20110512
 *
 * Copyright � 2000-2014 Future Technology Devices International Limited
 *
 *
 * THIS SOFTWARE IS PROVIDED BY FUTURE TECHNOLOGY DEVICES INTERNATIONAL LIMITED ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL FUTURE TECHNOLOGY DEVICES INTERNATIONAL LIMITED
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Project: libMPSSE
 * Module: SPI Sample Application - Interfacing 94LC56B SPI EEPROM
 *
 * Rivision History:
 * 0.1  - 20110512 - Initial version
 * 0.2  - 20110801 - Changed LatencyTimer to 255
 * 					 Attempt to open channel only if available
 *					 Added & modified macros
 *					 Included stdlib.h
 * 0.3  - 20111212 - Added comments
 * 0.41 - 20140903 - Fixed compilation warnings
 *					 Added testing of SPI_ReadWrite()
 */

/******************************************************************************/
/* 							 Include files										   */
/******************************************************************************/
/* Standard C libraries */
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
/* OS specific libraries */
#ifdef _WIN32
#include<windows.h>
#endif

/* Include D2XX header*/
#include "ftd2xx.h"

/* Include libMPSSE header */
#include "libMPSSE_spi.h"

/******************************************************************************/
/*								Macro and type defines							   */
/******************************************************************************/
/* Helper macros */

#define APP_CHECK_STATUS(exp) {if(exp!=FT_OK){printf("%s:%d:%s(): status(0x%x) \
!= FT_OK\n",__FILE__, __LINE__, __FUNCTION__,exp);exit(1);}else{;}};
#define CHECK_NULL(exp){if(exp==NULL){printf("%s:%d:%s():  NULL expression \
encountered \n",__FILE__, __LINE__, __FUNCTION__);exit(1);}else{;}};

/* Application specific macro definations */
#define SPI_DEVICE_BUFFER_SIZE		256
#define SPI_WRITE_COMPLETION_RETRY		10
#define START_ADDRESS_EEPROM 	0x00 /*read/write start address inside the EEPROM*/
#define END_ADDRESS_EEPROM		0x10
#define RETRY_COUNT_EEPROM		10	/* number of retries if read/write fails */
#define CHANNEL_TO_OPEN			0	/*0 for first available channel, 1 for next... */
#define SPI_SLAVE_0				0
#define SPI_SLAVE_1				1
#define SPI_SLAVE_2				2
#define DATA_OFFSET				4
#define USE_WRITEREAD			0

/******************************************************************************/
/*								Global variables							  	    */
/******************************************************************************/
static FT_HANDLE ftHandle;
static uint8 buffer[SPI_DEVICE_BUFFER_SIZE] = {0};

/*!
 * \brief Main function / Entry point to the sample application
 *
 * This function is the entry point to the sample application. It opens the channel, writes to the
 * EEPROM and reads back.
 *
 * \param[in] none
 * \return Returns 0 for success
 * \sa
 * \note
 * \warning
 */
int main(int argc, char** argv)
{
	FT_STATUS status = FT_OK;
	FT_DEVICE_LIST_INFO_NODE devList = {0};
	ChannelConfig channelConf = {0};
	uint8 address = 0;
	uint32 channels = 0;
	uint16 data = 0;
	uint8 i = 0;
	uint8 latency = 255;

	if (argc != 2) {
		printf("Usage: %s <somestring>\n", argv[0]);
		return -1;
	}

	channelConf.ClockRate = 5000;
	channelConf.LatencyTimer = latency;
	channelConf.configOptions = SPI_CONFIG_OPTION_MODE0 | SPI_CONFIG_OPTION_CS_ACTIVELOW;
	channelConf.Pin = 0x00000000;/*FinalVal-FinalDir-InitVal-InitDir (for dir 0=in, 1=out)*/

	/* init library */
#ifdef _MSC_VER
	Init_libMPSSE();
#endif
	status = SPI_GetNumChannels(&channels);
	APP_CHECK_STATUS(status);
	printf("Number of available SPI channels = %d\n",(int)channels);

	if(channels>0)
	{
		for(i=0;i<channels;i++)
		{
			status = SPI_GetChannelInfo(i,&devList);
			APP_CHECK_STATUS(status);
			printf("Information on channel number %d:\n",i);
			/* print the dev info */
			printf("		Flags=0x%x\n",devList.Flags);
			printf("		Type=0x%x\n",devList.Type);
			printf("		ID=0x%x\n",devList.ID);
			printf("		LocId=0x%x\n",devList.LocId);
			printf("		SerialNumber=%s\n",devList.SerialNumber);
			printf("		Description=%s\n",devList.Description);
			printf("		ftHandle=0x%x\n",(unsigned int)devList.ftHandle);/*is 0 unless open*/
		}

		/* Open the first available channel */
		status = SPI_OpenChannel(CHANNEL_TO_OPEN,&ftHandle);
		APP_CHECK_STATUS(status);
		printf("\nhandle=0x%x status=0x%x\n",(unsigned int)ftHandle,status);
		status = SPI_InitChannel(ftHandle,&channelConf);
		APP_CHECK_STATUS(status);

		uint32 sizeTransfered;
		for (int i = 0; i < 256; i++)
			buffer[i] = i;
		
		strcpy(buffer, argv[1]);
		printf("Sending %s\n", buffer);

		status = SPI_Write(ftHandle, buffer, strlen(argv[1]), &sizeTransfered,
			SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES |
			SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE |
			SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE);
		APP_CHECK_STATUS(status);
		
		printf("Transfered %i bytes\r\n", sizeTransfered);
		status = SPI_CloseChannel(ftHandle);
#ifdef _MSC_VER
		Cleanup_libMPSSE();
#endif
		return 0;


	}

#ifndef __linux__
	system("pause");
#endif
	return 0;
}
