#pragma once
/***********************************************************************************************************************************
Noise Model for Haptic Texture Rendering Code

*  */

#ifndef NoiseModel_H
#define NoiseModel_H

#include "shared.h"
#include <vector>
#include "ErrorNoise.h"

using namespace std;


class NoiseModel
{
	ErrorNoise *Models;
public:
	NoiseModel(void)
	{
		Models = new ErrorNoise[NUM_TEX];
		//load files
	};
	~NoiseModel(void)
	{
		delete[] Models;
	}

	void init();
	void setForceandVelocity(double force, double vel);
	double predictAcc();
	void stop()
	{
		setForceandVelocity(0.0, 0.0);
	}
};

#endif
