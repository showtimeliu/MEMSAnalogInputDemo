#define NOMINMAX

#include <cmath>
#include <stdio.h>
#include <algorithm>
#include "MTIDevice.h"
#include "MTISwitchOS.h"
#include <highgui.h>
#include <cv.h>

int scanInt() {
	// Input function to handle user input across the MTIDevice-Demo
	char inptr[256], * endptr;
	fgets(inptr, 256, stdin);					// Get string from stdin and store in inptr
	int val = strtol(inptr, &endptr, 0);		// Get integer value of inptr string
	// strtol returns 0 on invalid input but if the pointers are the same then no conversion occurred
	if (inptr == endptr)
		return -1;
	else
		return val;
}

// Function to handle user's choice of a device at the beginning of the demo
// Utilizes MTIDevice::GetAvailableDevices(), MTIDevice::ListAvailableDevices
char* SelectIODevice(MTIDevice* mti)
{
	unsigned int portnumber = 0;
	char* comPortName = new char[12];

	printf("\nSearching for available Mirrorcle MEMS Controller devices. Please wait...\n");

	// Call to GetAvailableDevices returns table of devices and respective COM ports
	MTIAvailableDevices table;
	mti->GetAvailableDevices(&table);

	if (table.NumDevices == 0) {
		printf("\nThere are no devices available. Press any key to exit...");
		_getch();
		return NULL;
	}

	system(CLEARSCREEN);
	mti->ListAvailableDevices(&table); // Also prints the table of available devices to the command line.
	printf("\n\n");
	printf("Input serial port number to connect to: ");
	portnumber = scanInt();
	sprintf(comPortName, MTI_PORT_PREFIX "%d", portnumber);

	return comPortName;
}


int main() {

	// Begin by instantiating the MTIDevice class to access its methods,
	// attributes, and the target device at its serial port.
	MTIDevice* mti = new MTIDevice(); 

	char* portName = SelectIODevice(mti);	// Function to check COM ports for available devices and ask user to select one.
	//sprintf(portName,"COM3");

	if (portName == NULL) return 0;					// Leave the demo if no devices were found or chosen.

	mti->ConnectDevice(portName);					// Make a serial connection to the selected com port by name (e.g. "COM3").
	MTIError lastError = mti->GetLastError();		// Check for errors at any time. We check here to see if successfully connected.
	if (lastError != MTIError::MTI_SUCCESS)
	{
		printf("\n\nUnable to connect with any device at port %s.  Press any key to Exit.\n", portName);
		mti->SendSerialReset();
		_getch();
		return 0;										// Leave demo if not successfully connected
	}





	return 0; 
}


