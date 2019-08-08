/***********************************************************************************************************************************
LPC Model for Haptic Texture Rendering Code

This code is based on the original TexturePad haptic rendering system designed by Joseph M. Romano.
************************************************************************************************************************************/

#ifndef LPCModel_H
#define LPCModel_H

#include "AccSynthHashMatrix.h"
#include "shared.h"
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include "pugixml.hpp"
#include <Windows.h>
#include <vector>
#include <errno.h>
#include <string>
#include <WinBase.h>

using namespace std;

class LPCModel
{
	//LPC variables
	AccSynthHashMatrix mymatrix;
	static boost::mt19937 rng;

	//pAudio variables
	static vector <float> outputHist;
	static vector <float> excitationHist;

	int samplingRate;
	float vTan, fNorm;

	//Performance counter
	LARGE_INTEGER lastTime, thisTime, deltaTime, frequency;
	float deltaTimeS;

	ofstream LPCout;

public:
	LPCModel(void)
	{
		samplingRate = 1000;
		generateHashMatrix(1000);
		QueryPerformanceFrequency(&frequency);
		LPCout.open("LPCout.txt");
	};
	LPCModel(int _samplingRate);
	~LPCModel(void)
	{
		LPCout.close();
	};
	void generateHashMatrix(int _samplingRate);
	double LPCvibration();
	double noiseVibration();
	void setForceandVelocity(float _fNorm, float _vTan);
};

#endif
