#include "ErrorNN.h"

std::vector <double> ErrorNN::forceHist;
std::vector <double> ErrorNN::accHist;
std::vector <double> ErrorNN::velHist;
boost::mt19937 ErrorNN::rng;

ErrorNN::~ErrorNN()
{
	//std::cout << "NN Model Destructor" << std::endl;
	delete[] numNeuron;
	delete[] biasLayer;
	delete biasInput;
	delete[] weightLayer;
	delete[] weightInput;
	delete[] offset;
	delete[] gain;

	delete[] vTable;
	delete[] fTable;

	for (int i = 0; i < 5; i++)
	{
		delete[] eTable[i];
	}
	delete[] eTable;
}

void ErrorNN::initFromFile(const char* filename)
{
	std::ifstream myfile;
	myfile.open(filename);
	std::string str;
	std::string buf;
	std::stringstream ss;
	std::vector<std::string> tokens;
	int cnt = 0;
	int a_numX, a_numY;

	getline(myfile, str);
	//std::cout << str << "ENDL" << std::endl;
	ss << str;
	while (ss >> buf)
		tokens.push_back(buf);
	for (std::vector<std::string>::iterator it = tokens.begin(); it != tokens.end(); it++)
	{
		//std::cout << *it << ", ";
		if (it == tokens.begin())
		{
			a_numX = std::stoi(*it);
		}
		else
		{
			a_numY = std::stoi(*it);
		}
		//std::cout <<cnt << std::endl;
	}
	//std::cout << std::endl;
	setInput(2, a_numX, a_numY);
	ss.clear();
	tokens.clear();
	//1st line: numXsample and  numYsample. setting inputs

	getline(myfile, str);
	//std::cout << str << "ENDL" << std::endl;
	ss << str;
	while (ss >> buf)
		tokens.push_back(buf);
	for (std::vector<std::string>::iterator it = tokens.begin(); it != tokens.end(); it++)
	{
		if (it == tokens.begin())
		{
			setLayer(std::stoi(*it));
		}
		else
		{
			setNeuron(std::stoi(*it), cnt++);
		}
		//std::cout <<cnt << std::endl;
	}
	//std::cout << std::endl;
	ss.clear();
	tokens.clear();
	cnt = 0;
	//2nd line: numLayer and  numNeuron

	getline(myfile, str);
	//std::cout << str << "ENDL" << std::endl;
	ss << str;
	while (ss >> buf)
		tokens.push_back(buf);
	for (std::vector<std::string>::iterator it = tokens.begin(); it != tokens.end(); it++)
	{
		if (cnt % 2 == 0)
		{
			setOffset(std::stod(*it), cnt / 2);
		}
		else
		{
			double a_gain = (std::stod(*it) - offset[cnt / 2]) / 2;
			setGain(a_gain, cnt / 2);
		}
		cnt++;
		//std::cout <<cnt << std::endl;
	}
	//std::cout << std::endl;
	ss.clear();
	tokens.clear();
	cnt = 0;
	//3rd line: Normalize constants

	fTable = new double[5];//Size 5,9: add zero condition
	fTable[0] = 0.0;
	getline(myfile, str);
	//std::cout << str << "ENDL" << std::endl;
	ss << str;
	while (ss >> buf)
		tokens.push_back(buf);
	for (std::vector<std::string>::iterator it = tokens.begin(); it != tokens.end(); it++)
	{
		fTable[++cnt] = std::stod(*it);
		//std::cout <<cnt << std::endl;
	}
	//std::cout << std::endl;
	ss.clear();
	tokens.clear();
	cnt = 0;
	//4th line: Force table

	vTable = new double[9];//Size 5,9: add zero condition
	vTable[0] = 0.0;
	getline(myfile, str);
	//std::cout << str << "ENDL" << std::endl;
	ss << str;
	while (ss >> buf)
		tokens.push_back(buf);
	for (std::vector<std::string>::iterator it = tokens.begin(); it != tokens.end(); it++)
	{
		vTable[++cnt] = std::stod(*it);
		//std::cout <<cnt << std::endl;
	}
	//std::cout << std::endl;
	ss.clear();
	tokens.clear();
	cnt = 0;
	//5th line: Velocity table
	
	eTable = new double*[5];		//Size 5,9: add zero condition
	for (int i = 0; i < 5; i++)
	{
		eTable[i] = new double[9];
	}

	for (int i = 0; i < 5; i++)
	{
		eTable[i][0] = 0.0;
	}

	for (int i = 0; i < 9; i++)
	{
		eTable[0][i] = 0.0;
	}

	getline(myfile, str);
	//std::cout << str << "ENDL" << std::endl;
	ss << str;
	while (ss >> buf)
		tokens.push_back(buf);
	for (std::vector<std::string>::iterator it = tokens.begin(); it != tokens.end(); it++)
	{
		int idx1, idx2;
		idx1 = (cnt / 8) + 1;
		idx2 = (cnt % 8) + 1;
		cnt++;
		eTable[idx1][idx2] = std::stod(*it);
		//std::cout <<cnt << std::endl;
	}
	//std::cout << std::endl;
	ss.clear();
	tokens.clear();
	cnt = 0;
	//6th line: Error power

	weightInput[0].resize(numNeuron[0], numXsample * numXtype);
	getline(myfile, str);
	ss << str;
	while (ss >> buf)
		tokens.push_back(buf);
	for (std::vector<std::string>::iterator it = tokens.begin(); it != tokens.end(); it++)
	{
		weightInput[0](cnt++) = stod(*(it));
	}
	ss.clear();
	tokens.clear();
	cnt = 0;
	//Input Weight 1: for exogeous inputs

	weightInput[1].resize(numNeuron[0], numYsample);
	if (numYsample > 0)
	{
		getline(myfile, str);
		ss << str;
		while (ss >> buf)
			tokens.push_back(buf);
		for (std::vector<std::string>::iterator it = tokens.begin(); it != tokens.end(); it++)
		{
			weightInput[1](cnt++) = stod(*(it));
		}
		ss.clear();
		tokens.clear();
		cnt = 0;
		//Input Weight 2: for regressive input
	}
	

	biasInput[0].resize(numNeuron[0], 1);
	getline(myfile, str);
	ss << str;
	while (ss >> buf)
		tokens.push_back(buf);
	for (std::vector<std::string>::iterator it = tokens.begin(); it != tokens.end(); it++)
	{
		biasInput[0](cnt++) = stod(*(it));
	}
	ss.clear();
	tokens.clear();
	cnt = 0;
	//Input Bias

	for (int i = 0; i < numLayer; i++)
	{
		if (numLayer - 1 == i)
			weightLayer[i].resize(1, numNeuron[i]);
		else
			weightLayer[i].resize(numNeuron[i + 1], numNeuron[i]);
		getline(myfile, str);
		ss << str;
		while (ss >> buf)
			tokens.push_back(buf);
		for (std::vector<std::string>::iterator it = tokens.begin(); it != tokens.end(); it++)
		{
			weightLayer[i](cnt++) = stod(*(it));
		}
		ss.clear();
		tokens.clear();
		cnt = 0;

		//std::cout << weight[i] << std::endl;
	}
	//Layer weight matrix reading


	for (int i = 0; i < numLayer; i++)
	{
		if (numLayer - 1 == i)
			biasLayer[i].resize(1, 1);
		else
			biasLayer[i].resize(numNeuron[i + 1], 1);
		getline(myfile, str);
		ss << str;
		while (ss >> buf)
			tokens.push_back(buf);
		for (std::vector<std::string>::iterator it = tokens.begin(); it != tokens.end(); it++)
		{
			biasLayer[i](cnt++) = stod(*(it));
		}
		ss.clear();
		tokens.clear();
		cnt = 0;

		//std::cout << bias[i] << std::endl;
	}
	//Bias matrix reading

	myfile.close();
}


double ErrorNN::applyNN(VectorXd input1, VectorXd input2)
{
	//Apply neural network. Inputs are normalized.
	VectorXd result;
	VectorXd* outputs;

	//Check input size
	if (input1.size() != numXtype * numXsample)
	{
		std::cout << "Input1 size error!!" << std::endl;
		return 0.0;
	}
	else if (input2.size() != numYsample)
	{
		std::cout << "Input2 size error!!" << std::endl;
		return 0.0;
	}

	outputs = new VectorXd[numLayer + 1]; //outputs of each layer, including input layer

	if (numYsample > 0)
		outputs[0] = tansig(weightInput[0] * input1 + weightInput[1] * input2 + biasInput[0]);
	else
		outputs[0] = tansig(weightInput[0] * input1 + biasInput[0]);

	//std::cout << outputs[0] << std::endl;
	//std::cout << weightInput[0] * input1 << std::endl;

	for (int i = 0; i < numLayer; i++)
	{
		if (i == numLayer - 1)
			outputs[i + 1] = weightLayer[i] * outputs[i] + biasLayer[i];
		else
			outputs[i + 1] = tansig(weightLayer[i] * outputs[i] + biasLayer[i]);
	}
	//Apply each output
	result = reverseMinMax(outputs[numLayer], 2);

	return *(result.data());
	//minmax reverse!!
}

VectorXd ErrorNN::tansig(VectorXd v)
{
	VectorXd result;
	result = (2 / (exp(v.array() * -2) + 1) - 1).matrix();
	//std::cout << result << std::endl;
	return result;
}

VectorXd ErrorNN::applyMinMax(VectorXd input, int numVar)
{
	VectorXd result;
	result = (((input.array() - getOffset(numVar)) / getGain(numVar)) - 1).matrix();
	return result;
}

VectorXd ErrorNN::reverseMinMax(VectorXd input, int numVar)
{
	VectorXd result;
	result = (((input.array() + 1) * getGain(numVar)) + getOffset(numVar)).matrix();
	return result;

}

void ErrorNN::setForceandVelocity(double force, double vel)
{
	velHist.insert(velHist.begin(), vel);
	forceHist.insert(forceHist.begin(), force);

	if (velHist.size() > numXsample)
		velHist.pop_back();
	if (forceHist.size() > numXsample)
		forceHist.pop_back();

	excitationPower = interpolateErrorPower(vel, force);
}

double ErrorNN::interpolateErrorPower(double vel, double force)
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

double ErrorNN::predictNextValue()
{
	int inputSize1 = numXtype * numXsample;
	int inputSize2 = numYsample;

	//Data size check
	//if the size of output history is less than the number of AR coefficients, append zeros
	if (accHist.size() < numYsample) {
		int subt = numYsample - accHist.size();
		for (int j = 0; j < subt; j++) {
			accHist.push_back(0.0);
			//std::cout << "ADD at ACC" << std::endl;
		}
	}
	//Same to the force history
	if (forceHist.size() < numXsample) {
		int subt = numXsample - forceHist.size();
		for (int j = 0; j < subt; j++) {
			forceHist.push_back(0.0);
			//std::cout << "ADD at FORCE" << std::endl;
		}
	}

	//Same to the velocity history
	if (velHist.size() < numXsample) {
		int subt = numXsample - velHist.size();
		for (int j = 0; j < subt; j++) {
			velHist.push_back(0.0);
			//std::cout << "ADD at VEL" << std::endl;
		}
	}

	//Allocate force samples (input vector f(t-1) v(t-1) f(v-2) v(t-2)...)
	VectorXd x1_f, x1_f_n, x1_v, x1_v_n, x2_a, x2_a_n;
	MatrixXd x1_fv;
	x1_f.resize(numXsample);
	x1_v.resize(numXsample);
	x1_fv.resize(numXtype, numXsample);
	x2_a.resize(numYsample);

	for (int i = 0; i < numXsample; i++) {
		x1_f(i) = forceHist[i];
		x1_v(i) = velHist[i];
	}

	//Apply minmax mapping and combine into one, then reshaping
	x1_f_n = applyMinMax(x1_f, 0);
	x1_v_n = applyMinMax(x1_v, 1);
	x1_fv.row(0) = x1_f_n;
	x1_fv.row(1) = x1_v_n;
	x1_fv.resize(numXtype * numXsample, 1);

	//Allocate acc samples
	for (int i = 0; i < numYsample; i++) {
		x2_a(i) = accHist[i];
	}
	x2_a_n = applyMinMax(x2_a, 2);

	double result = applyNN(x1_fv, x2_a_n);

	if (errorPowered)
	{
		boost::normal_distribution<> nd(0.0, excitationPower);
		boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, nd);
		result += var_nor();
	}

	accHist.insert(accHist.begin(), result);
	if (accHist.size() > numYsample)
		accHist.pop_back();
	return result * 0.1;
}