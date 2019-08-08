/*****************************************************************************
Making Vibration for texture rendering
****************************************************************************/
#ifndef VIBMAKER_H
#define VIBMAKER_H

#include <cstdio>
#include <iostream>
#include <cmath>
#include <vector>
#include <direct.h>
#include "NIDAQmx.h"
#include <Windows.h>
#include <WinBase.h>

//Vibration Generating Models
#include "shared.h"
#include "LSFModel.h"
#include "NoiseModel.h"

#define HISTORY_LENGTH 500

#define MAX_V  3.0
#define MIN_V -3.0
#define OUTPUT_RATE 10000
#define EVERY_NSAMPLES 300
#define OUTPUT_BUFFER 1500

// For the actuator controller.
#define ACTUATOR_ACC_TO_VOLTAGE		0.2 //Find value

class VibMaker
{
	int32       error = 0;
	char        errBuff[2048];
	TaskHandle  VMTaskHandle;	
	LSFModel	lsfModel;
	NoiseModel	nModel;
	float64		out_data[EVERY_NSAMPLES];

	//Haptic Information
	double vTan = 0.0;
	double fNorm = 0.0;

	//Timer
	LARGE_INTEGER Frequency;
	LARGE_INTEGER BeginTime;
	LARGE_INTEGER Endtime;
	
	ofstream vibOutfile;
public:
	VibMaker(void) {
		vibOutfile.open("viboutput.txt");
		VMTaskHandle = 0;
		QueryPerformanceFrequency(&Frequency);
	};
	~VibMaker(void);

	void exitHandler();
	int32 init();
	int myDAQmxErrorHandler(int32 error);
	
	int32 EveryNSamplesCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples);

	static int32 CVICALLBACK everyNSamplesWrapper(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples, void *callbackData)
	{
		VibMaker* obj = static_cast<VibMaker*>(callbackData);
		return obj->EveryNSamplesCallback(taskHandle, everyNsamplesEventType, nSamples);
	}

	int32 startVibration();
	double updateVibration();
	void updateHapticInformation(double _force, double _vel);

	void stop();
};

#endif