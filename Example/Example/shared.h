/*shared memory file */
#pragma once
#ifndef _SHARED_H_
#define _SHARED_H_

#define MAX_COEFF 25
#define MAX_MACOEFF 25

#define NUM_TEX 10
#define NUM_REPETITION 3

#define IM_SIZE 3145728
#define MAX_NUM_RBF 4000

// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())

#include <string>
#include <vector>
#include "FOAW.h"
#include <string>

struct ExpSession {
	int material;
};

struct ExpTraining {
	int textureL;
	int textureR;
};

struct ExpResponse {
	char friction[64];
};

struct ExpSessionResponse {
	struct ExpResponse resp[NUM_REPETITION];
};

extern std::string resourceRoot;

//Texture samples position
extern double planeX;
extern double planeY;
extern double planeZ;
extern double planeSize; //Size of textured plane: 0.1 m for each side

//For force rendering
extern double sigma[NUM_TEX];
extern double zMax[NUM_TEX];
extern double zStick[NUM_TEX];

extern int textNum;
extern char* texArray[NUM_TEX]; // stores names of textures

extern double velTH;		//Velocity threshold for moving

extern bool experimentRunning;

extern float textureLevel;		//FF volume
extern float vibrationVolume;	//VT volume

//For display
extern double fN;
extern double vT;
extern double cA;

extern double monitor;
extern bool vibError;

#endif