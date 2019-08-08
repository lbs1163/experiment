/***********************************************************************************************************************************
Random Forest Model for Haptic Texture Rendering Code

*  */

#ifndef ErrorRF_H
#define ErrorRF_H

#include "shared.h"
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <Windows.h>
#include <vector>
#include <deque>
#include <errno.h>
#include <string>
#include <WinBase.h>
#include "mat.h"
#include "matrix.h"
using namespace std;

#define NODE_TERMINAL 255
#define NODE_TOSPLIT  254
#define NODE_INTERIOR 253
#define NULL 0

class ErrorRF
{
	int nrnodes;
	int ntree;

	//RF variables
	vector<vector<int> >  lDau;
	vector<vector<int> >  rDau;
	vector<vector<unsigned char> >  nodestatus;
	vector<vector<double> >  upper;
	vector<vector<double> >  avnode;
	vector<vector<int> >  mbest;
	vector<int>  ndtree;

	int mdim;
	int *cat;
	int keepPred;
	double *allPred;
	double *proxMat;
	int nodes;
	int *nodex;

	//Force, velocity, error table
	vector<double> vTable;
	vector<double> fTable;
	vector<vector<double> > eTable;
	double excitationPower = 0.0;

	//pAudio variables
	static vector <double> forceHist;
	static vector <double> accHist;
	static vector <double> velHist;

	static boost::mt19937 rng;

	float vTan, fNorm;

	//for error-powered
	bool errorPowered;

	int numSamplesAcc;
	int interval = 1;
	int numSamplesForce;
	int numSamplesVelocity;

public:
	ErrorRF(void)
	{
		//Init vectors
	};
	~ErrorRF(void)
	{
	};

	void loadModel(const char* filename);
	double calcModelPredictCpp();

	/////// Help methods
	void zeroInt(int *x, int length) {
		memset(x, 0, length * sizeof(int));
	}

	void zeroDouble(double *x, int length) {
		memset(x, 0, length * sizeof(double));
	}

	double predictRegTree(double *x, int mdim, int i);

	void setForceandVelocity(double force, double vel);

	void printTable()
	{
		std::cout << "F Table" << std::endl;
		for (int i = 0; i < 5; i++)
			std::cout << fTable[i] << " ";
		std::cout << std::endl;

		std::cout << "V Table" << std::endl;
		for (int i = 0; i < 9; i++)
			std::cout << vTable[i] << " ";
		std::cout << std::endl;

		std::cout << "E Table" << std::endl;
		for (int i = 0; i < 5; i++)
		{
			for (int j = 0; j < 9; j++)
				std::cout << eTable[i][j] << " ";
			std::cout << std::endl;
		}
	}

	double interpolateErrorPower(double vel, double force);

	void setErrorPowered(bool flag)
	{
		errorPowered = flag;
	}
};

#endif
