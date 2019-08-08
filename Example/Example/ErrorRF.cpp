#include "ErrorRF.h"
#include "Shared.h"

vector <double> ErrorRF::forceHist;
vector <double> ErrorRF::accHist;
vector <double> ErrorRF::velHist;

boost::mt19937	ErrorRF::rng;

void ErrorRF::loadModel(const char* filename) {
	std::ifstream myfile;
	std::string str;
	std::string buf;
	std::stringstream ss;
	std::vector<std::string> tokens;
	int cnt = 0;
	MATFile *pmat;
	mxArray *pa;
	int* pI;
	double* pD;
	unsigned char* pC;
	
	//cout << "------ Model loading: " << filename << " -------" << endl;
	pmat = matOpen(filename, "r");
	//cout << "------ Model var loading! -----------------" << endl;

	double* temp;

	pa = matGetVariable(pmat, "num_force");
	numSamplesForce = *mxGetPr(pa);
	pa = matGetVariable(pmat, "num_vel");
	numSamplesVelocity = *mxGetPr(pa);
	pa = matGetVariable(pmat, "num_acc");
	numSamplesAcc = *mxGetPr(pa);
	pa = matGetVariable(pmat, "ntree");
	ntree = *mxGetPr(pa);
	pa = matGetVariable(pmat, "nrnode");
	nrnodes = *mxGetPr(pa);

	//Load lDau
	pa = matGetVariable(pmat, "lDau");
	pI = mxGetInt32s(pa);
	for (int j = 0; j < ntree; j++)
	{
		vector<int> lDauTemp;
		lDauTemp.reserve(nrnodes);
		lDauTemp.assign(pI + j * nrnodes, pI + (j + 1) * nrnodes);
		lDau.push_back(lDauTemp);
	}
	pI = NULL;

	//Load rDau
	pa = matGetVariable(pmat, "rDau");
	pI = mxGetInt32s(pa);
	for (int j = 0; j < ntree; j++)
	{
		vector<int> rDauTemp;
		rDauTemp.reserve(nrnodes);
		rDauTemp.assign(pI + j * nrnodes, pI + (j + 1) * nrnodes);
		rDau.push_back(rDauTemp);
	}
	pI = NULL;
	
	//Load nodestatus
	pa = matGetVariable(pmat, "nS");
	pC = (unsigned char*)mxGetChars(pa);
	for (int j = 0; j < ntree; j++)
	{
		vector<unsigned char> nSTemp;
		nSTemp.reserve(nrnodes);
		nSTemp.assign(pC + j * nrnodes, pC + (j + 1) * nrnodes);
		nodestatus.push_back(nSTemp);
	}
	pC = NULL;
	
	//Load upper
	pa = matGetVariable(pmat, "upper");
	pD = mxGetDoubles(pa);
	for (int j = 0; j < ntree; j++)
	{
		vector<double> upperTemp;
		upperTemp.reserve(nrnodes);
		upperTemp.assign(pD + j * nrnodes, pD + (j + 1) * nrnodes);
		upper.push_back(upperTemp);
	}
	pD = NULL;
	
	//Load avnode
	pa = matGetVariable(pmat, "avnode");
	pD = mxGetDoubles(pa);
	for (int j = 0; j < ntree; j++)
	{
		vector<double> avTemp;
		avTemp.reserve(nrnodes);
		avTemp.assign(pD + j * nrnodes, pD + (j + 1) * nrnodes);
		avnode.push_back(avTemp);
	}
	pD = NULL;
	
	//Load mbest
	pa = matGetVariable(pmat, "mbest");
	pI = mxGetInt32s(pa);
	for (int j = 0; j < ntree; j++)
	{
		vector<int> mbestTemp;
		mbestTemp.reserve(nrnodes);
		mbestTemp.assign(pI + j * nrnodes, pI + (j + 1) * nrnodes);
		mbest.push_back(mbestTemp);
	}
	pI = NULL;
	
	//Load ndtree
	pa = matGetVariable(pmat, "ndtree");
	pI = mxGetInt32s(pa);
	ndtree.reserve(ntree);
	ndtree.assign(pI, pI + ntree);
	pI = NULL;
	
	int num_f_table, num_v_table;
	
	//Load force table
	pa = matGetVariable(pmat, "F_table");
	num_f_table = mxGetNumberOfElements(pa);
	pD = mxGetDoubles(pa);
	fTable.reserve(num_f_table);		
	fTable.assign(pD, pD+num_f_table);
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
	pa = matGetVariable(pmat, "error_power");
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

double ErrorRF::calcModelPredictCpp() {
	//// data size: input size to the model
	int p_size = numSamplesAcc + numSamplesForce + numSamplesVelocity;
	int n_size = 1;

	//// DATA
	double* x;
	double ypred;

	x = (double*)calloc(p_size, sizeof(double));

	//Data size check
	//if the size of output history is less than the number of AR coefficients, append zeros
	if (accHist.size() < numSamplesAcc * interval) {
		int subt = numSamplesAcc * interval - accHist.size();
		for (int j = 0; j < subt; j++) {
			accHist.push_back(0.0);
			//cout << "ADD at ACC" << endl;
		}
	}
	//Same to the force history
	if (forceHist.size() < numSamplesForce * interval) {
		int subt = numSamplesForce * interval - forceHist.size();
		for (int j = 0; j < subt; j++) {
			forceHist.push_back(0.0);
			//cout << "ADD at FORCE" << endl;
		}
	}

	//Same to the velocity history
	if (velHist.size() < numSamplesVelocity * interval) {
		int subt = numSamplesVelocity * interval - velHist.size();
		for (int j = 0; j < subt; j++) {
			velHist.push_back(0.0);
			//cout << "ADD at FORCE" << endl;
		}
	}

	//Allocate force samples
	for (int i = 0; i < numSamplesForce; i++) {
		int pos = (i + 1) * interval - 1;
		x[i] = forceHist[pos];
	}

	//Allocate velocity samples
	for (int i = 0; i < numSamplesVelocity; i++) {
		int pos = (i + 1) * interval - 1;
		x[numSamplesForce + i] = velHist[pos];
	}

	//Allocate acc samples
	for (int i = 0; i < numSamplesAcc; i++) {
		int pos = (i + 1) * interval - 1;
		x[numSamplesForce + numSamplesVelocity + i] = accHist[pos];
	}

	//Memory allocate for the predicted values from each tree
	ypred = 0;

	//// mexFunction
	mdim = p_size;
	cat = (int*)calloc(p_size, sizeof(int));
	for (int i = 0; i<p_size; i++)
		cat[i] = 1;
	int maxcat = 1;		//Cat: variables for categorization
	keepPred = 0;		//Flag to turn on and off allPred. 0: No, 1: Yes
	allPred = 0;			//A matrix to save all the predicted values from each tree
	int doProx = 0;	//Proximity calculation
	proxMat = 0;		//A matrix to save the proximity values
	nodes = 0;	//TODO: for WHAT? Trace back the nodes values? 0: Not to trace, 1: trace

				//// regForest function
	int i;

	for (i = 0; i < ntree; ++i) {
		ypred += predictRegTree(x, mdim, i);
	};

	ypred /= ntree;

	free(x);

	if (errorPowered)
	{
		boost::normal_distribution<> nd(0.0, excitationPower);
		boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > var_nor(rng, nd);
		ypred+= var_nor();
	}
	
	accHist.insert(accHist.begin(), ypred);
	if ( accHist.size()> numSamplesAcc)
	{
		accHist.pop_back();	
	}
	
	return ypred;
}
double ErrorRF::predictRegTree(double *x, int mdim, int idx1) {
	int k = 0, m;

	while (nodestatus[idx1][k] != NODE_TERMINAL) { /* go down the tree */
		m = (int)mbest[idx1][k] - 1;
		k = (x[m] <= (double)upper[idx1][k]) ? (int)lDau[idx1][k] - 1 : (int)rDau[idx1][k] - 1;
	}
	/* terminal node: assign prediction and move on to next */
	//cout << ((double)avnode[idx1][k]) << endl;
	return ((double)avnode[idx1][k]);
}

void ErrorRF::setForceandVelocity(double force, double vel)
{
	velHist.insert(velHist.begin(), vel);
	forceHist.insert(forceHist.begin(), force);

	if (velHist.size() > numSamplesVelocity)
		velHist.pop_back();
	if (forceHist.size() > numSamplesForce)
		forceHist.pop_back();

	excitationPower = interpolateErrorPower(vel, force);
}

double ErrorRF::interpolateErrorPower(double vel, double force)
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

	result = (eTable[fIndex1][vIndex1] * (fFrac1 * vFrac1) + eTable[fIndex1][vIndex2] * (fFrac1 * vFrac2) + eTable[fIndex2][vIndex1] * (fFrac2 * vFrac1) + eTable[fIndex2][vIndex2] * (fFrac2 * vFrac2));


	/*
	std::cout << "F Index" << fIndex1 << "," << fIndex2 << std::endl;
	std::cout << "V Index" << vIndex1 << "," << vIndex2 << std::endl;

	std::cout << "F frac" << fFrac1 << "," << fFrac2 << std::endl;
	std::cout << "V frac" << vFrac1 << "," << vFrac2 << std::endl;

	std::cout << "Error power" << result << std::endl;
	*/
	return result;
}