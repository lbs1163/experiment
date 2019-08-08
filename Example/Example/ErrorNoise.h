#pragma once
/***********************************************************************************************************************************
Noise Model for Haptic Texture Rendering Code

*  */

#ifndef ErrorN_H
#define ErrorN_H

#include "shared.h"
#include <vector>
#include "mat.h"
#include "matrix.h"
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;


class ErrorNoise
{
	//Force, velocity, error table
	vector<double> vTable;
	vector<double> fTable;
	vector<vector<double> > eTable;
	double excitationPower = 0.0;

	static boost::mt19937 rng;

public:
	ErrorNoise(void)
	{
		//load files
	};
	~ErrorNoise(void)
	{

	};

	void loadModel(const char* filename);
	double nextSample();
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
};

#endif
#pragma once
