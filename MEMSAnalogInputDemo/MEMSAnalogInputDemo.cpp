#define NOMINMAX

#include <cmath>
#include <stdio.h>
#include <algorithm>
#include "MTIDevice.h"
#include "MTISwitchOS.h"
// #include <OpenCV/highgui.h>
// #include <OpenCV/cv.h>

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

	char* portName = SelectIODevice(mti); 





	return 0; 
}


