#include "NoiseModel.h"

void NoiseModel::init()
{
	for (int i = 0; i < NUM_TEX; i++) {
		string textureName(texArray[i]);
		string cmd = "../../resources/textures/Noise/" + textureName + ".mat";
		//std::cout << cmd << " start reading" << std::endl;
		Models[i].loadModel(cmd.c_str());
		//std::cout << cmd << "Read done" << std::endl;
	}
	cout << "Noise Load succeed" << endl;
}
void NoiseModel::setForceandVelocity(double force, double vel)
{
	Models[textNum].setForceandVelocity(force, vel);
}
double NoiseModel::predictAcc()
{
	return Models[textNum].nextSample();
}