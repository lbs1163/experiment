/*****************************************************************************
Making Vibration for texture rendering
****************************************************************************/

#include "VibMaker.h"
#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) return myDAQmxErrorHandler(error); else

VibMaker::~VibMaker(void)
{
	vibOutfile.close();
	exitHandler();
}

void VibMaker::exitHandler(void)
{
	printf("\nRunning Vib Exit Handler...\n");

	if(VMTaskHandle != 0) {  // Upon exit make sure that the task is stopped and cleared
		DAQmxStopTask(VMTaskHandle);
		DAQmxClearTask(VMTaskHandle);

		if( DAQmxFailed(error) ) {
			DAQmxGetExtendedErrorInfo(errBuff,2048);
			printf("DAQmx Error: %s\n",errBuff);
		}
	}
}

int32 VibMaker::init() 
{
	//Init DAQ function
	DAQmxErrChk(DAQmxCreateTask("VMDriver", &VMTaskHandle));
	DAQmxErrChk(DAQmxCreateAOVoltageChan(VMTaskHandle, "Dev2/ao1", "VMOUT", MIN_V, MAX_V, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxCfgSampClkTiming(VMTaskHandle, "", OUTPUT_RATE, DAQmx_Val_Rising, DAQmx_Val_ContSamps, EVERY_NSAMPLES));
	//DAQmxErrChk(DAQmxSetAODataXferReqCond(VMTaskHandle, "", DAQmx_Val_OnBrdMemNotFull));
	DAQmxErrChk(DAQmxSetAODataXferReqCond(VMTaskHandle, "", DAQmx_Val_OnBrdMemEmpty));
	//DAQmxErrChk(DAQmxSetAODataXferReqCond(VMTaskHandle, "", DAQmx_Val_OnBrdMemHalfFullOrLess));
	DAQmxErrChk(DAQmxCfgOutputBuffer(VMTaskHandle, OUTPUT_BUFFER));
	DAQmxErrChk(DAQmxSetWriteRegenMode(VMTaskHandle, DAQmx_Val_DoNotAllowRegen));
	DAQmxErrChk(DAQmxRegisterEveryNSamplesEvent(VMTaskHandle, DAQmx_Val_Transferred_From_Buffer, EVERY_NSAMPLES, 0, &everyNSamplesWrapper, this));
	
	//Initial zeros
	float64 data[OUTPUT_BUFFER];
	for (int i = 0; i < OUTPUT_BUFFER; i++)
	{
		data[i] = 0.0;
	}

	DAQmxErrChk(DAQmxWriteAnalogF64(VMTaskHandle, OUTPUT_BUFFER, false, 0.0, DAQmx_Val_GroupByChannel, data, NULL, NULL));

	//Init each models
	lsfModel.init();
	nModel.init();

	return 0;
}

int VibMaker::myDAQmxErrorHandler(int32 error)
{
	if( DAQmxFailed(error) )
		DAQmxGetExtendedErrorInfo(errBuff,2048);
	if( VMTaskHandle!=0 ) {
		/*********************************************/
		// DAQmx Stop Code
		/*********************************************/
		DAQmxStopTask(VMTaskHandle);
		DAQmxClearTask(VMTaskHandle);
	}
	if( DAQmxFailed(error) )
		printf("DAQmx Error: %s\n",errBuff);

	VibMaker::error = 0;
	vibError = true;
	return 1;
}

int32 VibMaker::startVibration()
{
	DAQmxErrChk(DAQmxStartTask(VMTaskHandle));
}

double VibMaker::updateVibration()
{
	double _sample;
	switch (vMethod)
	{
	case renderingV::LSF:
		_sample = lsfModel.LSFvibration() * ACTUATOR_ACC_TO_VOLTAGE;
		break;
	case renderingV::Noise:
		_sample = nModel.predictAcc() * ACTUATOR_ACC_TO_VOLTAGE;
		break;
	}

	cA = _sample / ACTUATOR_ACC_TO_VOLTAGE;

	//vibOutfile << fNorm << " " << vTan*100 << " " << cA << endl;

	//Check max-min values
	if (_sample > MAX_V)
		_sample = MAX_V;
	else if (_sample < MIN_V)
		_sample = MIN_V;
		
	return _sample;
	//cout << _sample << endl;
}

void VibMaker::stop()
{
	switch (vMethod)
	{
	case renderingV::LSF:
		lsfModel.stop();
		break;
	case renderingV::Noise:
		//nModel.stop();
		break;
	}
	vTan = 0.0;
	fNorm = 0.0;
}


int32 VibMaker::EveryNSamplesCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, uInt32 nSamples)
{
	//QueryPerformanceCounter(&BeginTime);

	for (int i = 0; i < EVERY_NSAMPLES; i++)
	{
		if (vTan == 0.0 || fNorm == 0.0)
			out_data[i] = 0.0;
		else
			out_data[i] = updateVibration();
	}
	DAQmxErrChk(DAQmxWriteAnalogF64(VMTaskHandle, EVERY_NSAMPLES, false, 0.0, DAQmx_Val_GroupByChannel, out_data, NULL, NULL));
	//printf("%d samples written\n", nSamples);


Error:
	if (DAQmxFailed(error))
	{
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
		printf("DAQmx Error: %s\n", errBuff);
	}
	//QueryPerformanceCounter(&Endtime);
	//int64 elapsed = Endtime.QuadPart - BeginTime.QuadPart;
	//double duringtime = (double)elapsed / (double)Frequency.QuadPart;
	//vibOutfile << duringtime << "sec" <<endl;
	return 0;
}

void VibMaker::updateHapticInformation(double _force, double _vel)
{
	
	vTan = _vel;
	fNorm = _force;

	switch (vMethod)
	{
	case renderingV::LSF:
		lsfModel.setForceandVelocity(fNorm, vTan);
		break;
	case renderingV::Noise:
		nModel.setForceandVelocity(fNorm, vTan * 100);	//change metric unit from m(chai3d standard) to cm(used in modeling)
		break;
	}
}