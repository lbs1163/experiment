#include "ErrorNoise.h"

boost::mt19937	ErrorNoise::rng;

void ErrorNoise::loadModel(const char* filename) {
	std::ifstream myfile;
	std::string str;
	std::string buf;
	std::stringstream ss;
	std::vector<std::string> tokens;
	int cnt = 0;
	MATFile *pmat;
	mxArray *pa;
	double* pD;

	//cout << "------ Model loading: " << textureName << " -------" << endl;
	pmat = matOpen(filename, "r");
	//cout << "------ Model var loading! -----------------" << endl;

	int num_f_table, num_v_table;

	//Load force table
	pa = matGetVariable(pmat, "F_table");
	num_f_table = mxGetNumberOfElements(pa);
	pD = mxGetDoubles(pa);
	fTable.reserve(num_f_table);
	fTable.assign(pD, pD + num_f_table);
	fTable.insert(fTable.begin(), 0.0); //additional zero
	pD = NULL;

	//Load velocity table
	pa = matGetVariable(pmat, "V_table");
	num_v_table = mxGetNumberOfElements(pa);
	pD = mxGetDoubles(pa);
	vTable.reserve(num_v_table);
	vTable.assign(pD, pD + num_v_table);
	vTable.insert(vTable.begin(), 0.0); //additional zero
	pD = NULL;

	//Load error table
	pa = matGetVariable(pmat, "noiseLevel");
	pD = mxGetDoubles(pa);
	vector<double> eTableTemp(num_v_table + 1, 0.0); //additional zero
	eTable.push_back(eTableTemp);
	for (int j = 0; j < num_f_table; j++)
	{
		eTableTemp.reserve(num_v_table);
		eTableTemp.assign(pD + j * num_v_table, pD + (j + 1) * num_v_table);
		eTableTemp.insert(eTableTemp.begin(), 0.0); //additional zero
		eTable.push_back(eTableTemp);
	}
	pD = NULL;
}

void ErrorNoise::setForceandVelocity(double force, double vel)
{
	excitationPower = interpolateErrorPower(vel, force);
}

double ErrorNoise::interpolateErrorPower(double vel, double force)
{
	//Find lower/upper bound in table
	int vIndex1, vIndex2, fIndex1, fIndex2;//Lower and upper bound
	double fFrac1, fFrac2, vFrac1, vFrac2;//fraction of each point
	double result;
	vIndex1 = 0;
	vIndex2 = 8;
	fIndex1 = 0;
	fIndex2 = 4;

	while (vTable[vIndex1] <= vel)
	{
		if (vIndex1 < 8 && vTable[vIndex1 + 1] <= vel)
			vIndex1++;
		else
			break;
	}

	while (vTable[vIndex2] >= vel)
	{
		if (vIndex2 > 0 && vTable[vIndex2 - 1] >= vel)
			vIndex2--;
		else
			break;
	}

	while (fTable[fIndex1] <= force)
	{
		if (fIndex1 < 4 && fTable[fIndex1 + 1] <= force)
			fIndex1++;
		else
			break;
	}

	while (fTable[fIndex2] >= force)
	{
		if (fIndex2 > 0 && fTable[fIndex2 - 1] >= force)
			fIndex2--;
		else
			break;
	}

	if (fIndex1 == fIndex2)
	{
		fFrac1 = 1.0;
		fFrac2 = 0.0;
	}
	else
	{
		fFrac1 = (fTable[fIndex2] - force) / (fTable[fIndex2] - fTable[fIndex1]);
		fFrac2 = (force - fTable[fIndex1]) / (fTable[fIndex2] - fTable[fIndex1]);
	}
	if (vIndex1 == vIndex2)
	{
		vFrac1 = 1.0;
		vFrac2 = 0.0;
	}
	else
	{
		vFrac1 = (vTable[vIndex2] - vel) / (vTable[vIndex2] - vTable[vIndex1]);
		vFrac2 = (vel - vTable[vIndex1]) / (vTable[vIndex2] - vTable[vIndex1]);
	}

	result = eTable[fIndex1][vIndex1] * (fFrac1 * vFrac1) + eTable[fIndex1][vIndex2] * (fFrac1 * vFrac2) + eTable[fIndex2][vIndex1] * (fFrac2 * vFrac1) + eTable[fIndex2][vIndex2] * (fFrac2 * vFrac2);

	/*
	std::cout << "F Index" << fIndex1 << "," << fIndex2 << std::endl;
	std::cout << "V Index" << vIndex1 << "," << vIndex2 << std::endl;

	std::cout << "F frac" << fFrac1 << "," << fFrac2 << std::endl;
	std::cout << "V frac" << vFrac1 << "," << vFrac2 << std::endl;

	std::cout << "Error power" << result << std::endl;
	*/
	return result;
}

double ErrorNoise::nextSample()
{
	boost::normal_distribution<> nd(0.0, excitationPower);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, nd);
	return var_nor();
}