#ifndef _SHAREDMEMORYINIT_H_
#define _SHAREDMEMORYINIT_H_

std::string resourceRoot;

//Texture samples position
double planeX = -0.05;
double planeY = -0.06;
double planeZ = -0.02;
double planeSize = 0.25; //Size of textured plane: 0.2 m for each side


float filtLSF[200][MAX_COEFF];
float filtMALSF[200][MAX_MACOEFF];
int coeffNum[200];
int MAcoeffNum = MAX_MACOEFF;
int surfNum = 25;

int textNum = 0;

double velTH = 0.01;			//m/s

bool experimentRunning = false;

float textureLevel = 1.0;
float vibrationVolume = 1.0;

double fN = 0.0;
double vT = 0.0;
double cA = 0.0;

double monitor = 0.0;
bool vibError = false;

#endif