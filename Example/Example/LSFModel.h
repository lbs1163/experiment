/***********************************************************************************************************************************
LSF Model for Haptic Texture Rendering Code

This code is implimentation of two papers:
 * 1) Arsen Abdulali and Seokhee Jeon, ¡°Data-Driven Modeling of Anisotropic Haptic
 * Textures: Data-Segmentation and Interpolation," In Proceedings of  the Eurohaptics.
 * 2) Arsen Abdulali and Seokhee Jeon, ¡°Data-Driven Rendering of Anisotropic Haptic
 * Textures,¡± In Proceedings of  the  AsiaHaptics 2016.
 *
 * Some part of the code were taken from the work: Heather Culbertson, Juan Jose Lopez
 * Delgado, and Katherine J. Kuchenbecker. One Hundred Data-Driven Haptic Texture
 * Models and Open-Source Methods for Rendering on 3D Objects. In Proc. IEEE Haptics
 * Symposium, February 2014., which was used for LSF to AR conversion. If you
 * use current library, please, acknowladge the aforementioned authors as well.
 *  */

#ifndef LSFModel_H
#define LSFModel_H

#include "shared.h"
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <Windows.h>
#include <vector>
#include <errno.h>
#include <string>
#include <WinBase.h>

using namespace std;

class LSFModel
{
	//RBF Model Information
	int rbfNum[NUM_TEX];
	int nARCoeff[NUM_TEX];
	double rbfCoeff[NUM_TEX][MAX_NUM_RBF];
	double polyCoeff[NUM_TEX][80];
	double rbfCenters[NUM_TEX][MAX_NUM_RBF][3];
	double velNormCoeff[NUM_TEX];
	double forceNormCoeff[NUM_TEX];

	//LSF Coefficients
	int coeffNum[NUM_TEX];
	int MAcoeffNum;
	double filtLSF[NUM_TEX][MAX_COEFF];
	double filtMALSF[NUM_TEX][MAX_MACOEFF];
	bool SynthesisFlag_Buffer1[NUM_TEX];
	double filtCoeff_buf1[NUM_TEX][MAX_COEFF];
	double filtCoeff_buf2[NUM_TEX][MAX_COEFF];
	double filtMACoeff_buf1[NUM_TEX][MAX_MACOEFF];
	double filtMACoeff_buf2[NUM_TEX][MAX_MACOEFF];;
	double filtVariance_buf1[NUM_TEX];
	double filtVariance_buf2[NUM_TEX];
	double filtGain_buf1[NUM_TEX];
	double filtGain_buf2[NUM_TEX];

	//LSF variables
	static boost::mt19937 rng;

	//pAudio variables
	static vector <double> outputHist[200];
	static vector <double> excitationHist[200];

	float vTan, fNorm;

	double m_vibGain;
	//gain for force higher than max f in measurement

	//Performance counter
	LARGE_INTEGER lastTime, thisTime, deltaTime, frequency;
	float deltaTimeS;

	ofstream LSFout;

public:
	LSFModel(void)
	{
		QueryPerformanceFrequency(&frequency);
		m_vibGain = 1.0;
		LSFout.open("LSFout.txt");
	};
	~LSFModel(void);
	double LSFvibration();
	double noiseVibration();
	void setForceandVelocity(double _fNorm, double _vTan);
	void loadModels();
	double interWeightsRbf(float v_x, float f_n, int numARCoeff);
	double interPolyCoeff(float v_x, float f_n, int numARCoeff);
	double interpolateRbf(float v_x, float f_n, int numARCoeff);
	void normInput(float &v_x, float &f_n);
	void ARFromLsf(float v_x, float f_n);
	void init();
	void stop();
};

#endif
