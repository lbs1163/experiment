/***********************************************************************************************************************************
Error-based Neural Network Model for Haptic Texture Rendering Code

*  */

#pragma once
#ifndef ErrorNN_H
#define ErrorNN_H
#include <Dense>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
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
#include "shared.h"
using namespace Eigen;


// NARX NN for acc modeling. exogeous input force and velocity.
class ErrorNN
{
private:
	//NN coefficients. X: exogeous, Y: target
	int numXtype;
	int numXsample;
	int numYsample;
	int numLayer;
	int* numNeuron;
	VectorXd* biasLayer;
	VectorXd* biasInput;
	MatrixXd* weightLayer;
	MatrixXd* weightInput;
	//For normalizing
	double* offset;
	double* gain;
	
	//Force, velocity table
	double* vTable,* fTable;
	//Excitation error power table
	double** eTable;
	double excitationPower = 0.0;

	//History
	static std::vector <double> forceHist;
	static std::vector <double> accHist;
	static std::vector <double> velHist;

	static boost::mt19937 rng;

	//for error-powered
	bool errorPowered;

public:
	~ErrorNN();
	int getNumLayer()
	{
		return numLayer;
	}
	double getOffset(int a_numVariable)
	{
		return offset[a_numVariable];
	}
	double getGain(int a_numVariable)
	{
		return gain[a_numVariable];
	}
	MatrixXd getWeightLayer(int layer)
	{
		return weightLayer[layer];
	}
	MatrixXd getWeightInput(int layer)
	{
		return weightInput[layer];
	}
	VectorXd getBiasLayer(int layer)
	{
		return biasLayer[layer];
	}
	VectorXd getBiasInput()
	{
		return biasInput[0];
	}
	void setLayer(int a_numLayer)
	{
		numLayer = a_numLayer;
		biasLayer = new VectorXd[numLayer];		//BIAS for each layer
		weightLayer = new MatrixXd[numLayer];	//WEIGHT for each layer
		numNeuron = new int[numLayer];	//for each layer
	}
	void setInput(int a_numXtype, int a_numXsample, int a_numYsample)
	{
		numXtype = a_numXtype;
		numXsample = a_numXsample;
		numYsample = a_numYsample;
		offset = new double[numXtype + 1];
		gain = new double[numXtype + 1];
		biasInput = new VectorXd;		//1 BIAS for input
		weightInput = new MatrixXd[2];	//WEIGHT for x,y
	}
	void setNeuron(int a_numNeuron, int at)
	{
		numNeuron[at] = a_numNeuron;
	}
	int getNeuron(int at) {
		return numNeuron[at];
	}
	void setBiasLayer(VectorXd* p_bias)
	{
		for (int i = 0; i < numLayer; i++)
		{
			biasLayer[i] = p_bias[i];
		}
	}
	void setBiasInput(VectorXd* p_bias)
	{
		for (int i = 0; i < 2; i++)
		{
			biasInput[i] = p_bias[i];
		}
	}

	void setWeightLayer(MatrixXd* p_weight)
	{
		for (int i = 0; i < numLayer; i++)
		{
			weightLayer[i] = p_weight[i];
		}
	}

	void setWeightInput(MatrixXd* p_weight)
	{
		for (int i = 0; i < 2; i++)
		{
			weightInput[i] = p_weight[i];
		}
	}

	void setOffset(double a_offset, int a_numVariable)
	{
		offset[a_numVariable] = a_offset;
	}

	void setGain(double a_gain, int a_numVariable)
	{
		gain[a_numVariable] = a_gain;
	}

	void initFromFile(const char* filename);
	double applyNN(VectorXd input1, VectorXd input2);
	VectorXd tansig(VectorXd v);
	VectorXd applyMinMax(VectorXd input, int numVar);
	VectorXd reverseMinMax(VectorXd input, int numVar);

	double predictNextValue();
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
