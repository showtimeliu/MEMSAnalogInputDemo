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

bool IsRGBModule(MTIDevice* mti)
{
	MTIDataFormat dataFormat = (MTIDataFormat)(int)mti->GetDeviceParam(MTIParam::DataFormat);
	if ((dataFormat == MTIDataFormat::Reduced_XYRGB) || (dataFormat == MTIDataFormat::Standard_XYRGB) || (dataFormat == MTIDataFormat::Standard_XYMRGB))
		return true;
	else
		return false;
}

// AnalogInputValues demonstrates the use of GetAnalogValues to sample the Analog Input values
// at the Analog connectors at that point in time
void AnalogInputValues(MTIDevice* mti)
{
	int key;
	bool runFlag = true;
	float x = 0, y = 0;

	printf("\nPress any key to get analog input values\n");
	printf("from the device for channels AI0 and AI1.\n");
	printf("Hit ESC to exit this mode\n");

	while (runFlag)
	{
		key = (int)_getch();
		float AI0 = mti->GetAnalogInputValue(0);
		float AI1 = mti->GetAnalogInputValue(1);
		printf("\n\nAI0 Voltage = %4.2f   AI1 Voltage = %4.2f", AI0, AI1);
		if (key == 27) runFlag = 0;
	}
	mti->ResetDevicePosition(); 				// Stop then send analog outputs (and device) back to origin in 25ms
}

// AnalogInputBuffer demonstrates the use of both the MTIDataMode Sample_And_Analog_Input_Buffer
// and GetAnalogInputBuffer
// 
// GetAnalogInputBuffer will sample the Analog Input values to a depth of the specified npts
void AnalogInputBuffer(MTIDevice* mti)
{
	MTIDataGenerator* datagen = new MTIDataGenerator();
	unsigned int npts = 2500;
	float* x, * y, dt = (float)M_PI * 4.f / npts;	// Create data for 2 sinusoidal cycles
	unsigned char* m;

	printf("\nDevice will output a waveform and store analog inputs AI0 and AI1 in a buffer\n");
	printf("during the output.\n");
	printf("Press ANY KEY to execute a single run.  Analog input data will be stored into.\n");
	printf("an ASCII file Export.txt in first two columns.\n\n");

	x = new float[npts]; y = new float[npts]; m = new unsigned char[npts];
	for (unsigned int i = 0; i < npts; i++)
	{
		x[i] = 0.9f * sin(2.f * i * dt);				// X-Axis position follows a Sin curve from -0.9 to +0.9 normalized position
		y[i] = 0.9f * cos(3.f * i * dt);				// Y-Axis position follows a Cos curve from -0.9 to +0.9 normalized position at (3/2) freq of X axis
		m[i] = (unsigned char)(i / 2) % 256;					// Digital signals P0.0 to P0.7 doing 8-bit counter toggles every sample
	}

	mti->SetDeviceParam(MTIParam::DataMode, MTIDataMode::Sample_And_Analog_Input_Buffer);
	_getch();
	mti->SendDataStream(x, y, m, npts, 0, false);
	mti->StartDataStream(1);
	float* ai0 = new float[npts];
	float* ai1 = new float[npts];
	mti->GetAnalogInputBuffer(ai0, ai1, npts);

	// Store AI0 and AI1 values (float arrays) into an ASCII file in the first two columns.
	// M values are stored in 3rd column. Not used here so we write dummy values.
	datagen->ExportFile("Analog_Buffer_Export.txt", ai0, ai1, m, npts, mti->GetDeviceParam(MTIParam::SampleRate));
	mti->SetDeviceParam(MTIParam::DataMode, MTIDataMode::Sample_Output);
	mti->ResetDevicePosition();

	SAFE_DELETE_ARRAY(x);	SAFE_DELETE_ARRAY(y);	SAFE_DELETE_ARRAY(m);
	SAFE_DELETE(datagen);
}

// AnalogInputToOutputDemo demonstrates how to switch between a Controller's various
// data modes (MTIDataMode).
//	
// MTIDataMode.Analog_Input_To_Output allows the user to drive the MEMS via two analog
// input voltages to the Analog port of a USB Controller
void AnalogInputToOutputDemo(MTIDevice* mti)
{
	// Set any MEMS Driver Settings for Analog Input to Output Mode
	MTIDeviceParams sParams;
	mti->GetDeviceParams(&sParams);	// Back up current params from controller
	MTIDeviceParams tParams;
	mti->GetDeviceParams(&tParams);	// Store new temporary params in tParams
	// Set the Driver Settings
	tParams.DataScale = 1.0;
	tParams.Vbias = 70;
	tParams.VdifferenceMax = 100;
	tParams.HardwareFilterBw = 200;
	tParams.SampleRate = 50000;
	mti->SetDeviceParams(&tParams);

	mti->ResetDevicePosition(); // Reset the device position for safety
	mti->StopDataStream(); // Stop Data Stream BEFORE changing Data Modes
	mti->SetDeviceParam(MTIParam::DataMode, MTIDataMode::Analog_Input_To_Output);

	printf("\nDevice will read analog inputs and convert the signals to MEMS drive voltages\n");
	printf("Press ANY KEY to set controller into Analog_Input_To_Output DataMode...\n");
	_getch();
	mti->StartDataStream();

	// Switch back to Sample_Output DataMode
	printf("Press ANY KEY to return controller to Sample_Output DataMode...\n");
	_getch();
	mti->StopDataStream(); // Stop the Data Stream BEFORE changing Data Modes
	mti->SetDeviceParam(MTIParam::DataMode, MTIDataMode::Sample_Output);
	mti->StartDataStream();
	mti->ResetDevicePosition();
	mti->SetDeviceParams(&sParams);  // Restore original params back to controller
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

	// Check for any parameters provided in an ini file such as mtidevice.ini
	// Following two lines load the device parameters from the ini file and updates the controller if valid
	MTIDeviceParams lparams;
	mti->LoadDeviceParams(&lparams, "mtidevice - S46974.ini");
	mti->SetDeviceParams(&lparams);

	// Now we can check if all the parameters we wanted are set on the controller
	mti->GetDeviceParams(&lparams);		// Get current info and parameters from controller

	// Now we move to the demo.
	// Recommended start is to reset device position to origin (data and offsets zeroed out)
	mti->ResetDevicePosition(); 				// Send analog outputs (and device) back to origin in 25ms
	lastError = mti->GetLastError();			// User can check for errors after every call to the API

	mti->SetDeviceParam(MTIParam::MEMSDriverEnable, true);		// Turn the MEMS Driver on for all examples below

	bool menuFlag = true, runFlag = true;
	int mode = 0;
	while (runFlag) 
	{
		if (menuFlag) 
		{
			system(CLEARSCREEN);	menuFlag = false; 
			printf("\n***************** MTIDevice-Demo 10.7 - C++ SDK Examples ********************\n");
			printf("Demonstrates uses of MTIDevice and MTIDataGenerator classes.\n\n");
			printf("Device Name: %s     Firmware Name: %s     API version %s\n\n", lparams.DeviceName, lparams.FirmwareName, mti->GetAPIVersion().c_str());
			printf("** Warning: Users should provide proper Vbias, VdifferenceMax, and\n");
			printf("** HardwareFilterBw parameters to avoid MEMS Mirror damage.\n\n");
			printf("General Examples:\n");
			printf("\t0: Quit\n\t1: Point to Point Demo\n\t2: Scanning Demo\n\t3: Import File with Samples Demo\n\t4: Import File with Keypoints Demo\n");
			printf("\t5: Import ILDA File\n");
			printf("\t6: Import File with Keypoints and Time Demo\n\t7: Slow raster Demo\n\t8: Follow WASD Keys Demo\n\t9: Read Analog Input Values Demo\n\tA: Read Analog Input Buffer Demo\n");
			printf("\tB: Analog Input to Output DataMode Demo\n");			
		}

		if (_kbhit())
		{
			mode = (int)_getch() - 48; menuFlag = true;
			mti->ResetDevicePosition(); 				// Stop then send analog outputs (and device) back to origin in 25ms
			system(CLEARSCREEN);
			switch (mode)
			{
			case 0:		runFlag = false;			break;
			case 1:		menuFlag = true;	break; // if (rgbCapable) PointToPointDemoRGB(mti);		else PointToPointDemo(mti);	break;
			case 2:		menuFlag = true;	break; // if (rgbCapable) ScanningDemoRGB(mti);			else ScanningDemo(mti);		break;
			case 3:		menuFlag = true;	break; // ImportFileDemo(mti);		break;
			case 4:		menuFlag = true;	break; // KeyPointDemo(mti);		break;
			case 5:		menuFlag = true;	break; // if (rgbCapable) ImportILDAFileDemoRGB(mti);	else ImportILDAFileDemo(mti);	break;
			case 6:		menuFlag = true;	break; // ImportTimeFileDemo(mti);	break;
			case 7:		menuFlag = true;	break; // RasterDemo(mti);			break;
			case 8:		menuFlag = true;	break; // if (rgbCapable) ArrowKeysDemoRGB(mti);		else ArrowKeysDemo(mti);		break;
			case 9:		AnalogInputValues(mti);	break;
			case 17:	AnalogInputBuffer(mti);	break;
			case 18:    AnalogInputToOutputDemo(mti); break;
			case 19:	menuFlag = true;	break; // if (trackingSupport) SinglePointTracking_Lissajous(mti);	break;
			case 20:	menuFlag = true;	break; // if (trackingSupport) SinglePointTracking_Raster(mti);	break;
			case 21:	menuFlag = true;	break; // if (trackingSupport) DataMode7Raster(mti);	break;
			case 22:	menuFlag = true;	break; // if (trackingSupport) Imaging(mti);	break;
			case -21:	runFlag = false;			break;
			default:								break;
			}
		}
	}



	return 0; 
}


