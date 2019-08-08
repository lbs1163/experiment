#pragma once
#include "shared.h"
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <Windows.h>
#include <vector>
#include <errno.h>
#include <string>
#include <WinBase.h>
#include <math.h>
#include <complex>

#ifndef rbfMath_h
#define rbfMath_h

using namespace std;

void loadModels();
void readLSFData(int mdl);
double interWeightsRbf(float v_x, float f_n, int numARCoeff, int mdl);
double interPolyCoeff(float v_x, float f_n, int numARCoeff, int mdl);
double interpolateRbf(float v_x, float f_n, int numARCoeff, int mdl);
void normInput(float &v_x, float &f_n, int mdl);
void ARFromLsf(float v_x, float f_n, int mdl);

#endif
