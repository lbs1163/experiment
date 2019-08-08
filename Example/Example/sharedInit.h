#ifndef _SHAREDMEMORYINIT_H_
#define _SHAREDMEMORYINIT_H_

std::string resourceRoot;

//Texture samples position
double planeX = 0.0;
double planeY = -0.06;
double planeZ = 0.05;
double planeSize = 0.10; //Size of textured plane: 0.1 m for each side

//For force rendering
double HEIGHT_SCALE[NUM_TEX];
double kE[NUM_TEX];
double bE[NUM_TEX];
double m[NUM_TEX];
double sigma[NUM_TEX];
double zMax[NUM_TEX];
double zStick[NUM_TEX];


float filtLSF[200][MAX_COEFF];
float filtMALSF[200][MAX_MACOEFF];
int coeffNum[200];
int MAcoeffNum = MAX_MACOEFF;
int surfNum = 25;

int textNum = 0;
char* texArray[NUM_TEX]; // stores names of textures

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