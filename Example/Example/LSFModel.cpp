#include "LSFModel.h"
#include "shared.h"

boost::mt19937	LSFModel::rng;
vector <double> LSFModel::outputHist[200];
vector <double> LSFModel::excitationHist[200];

void LSFModel::init()
{
	loadModels();

	for (int i = 0; i < NUM_TEX; i++)
	{
		filtVariance_buf1[i] = 0.0;
		filtVariance_buf2[i] = 0.0;
	}
}

void LSFModel::stop()
{
	filtVariance_buf1[textNum] = 0.0;
	filtVariance_buf2[textNum] = 0.0;
}

double LSFModel::LSFvibration()
{
	double output = 0.0;
	double excitation = 0.0;
	double rgen_mean = 0.0;
	boost::mt19937 generator;

	//Double buffered, if buffer 1:
	if (SynthesisFlag_Buffer1[textNum]) {
		//generate Gaussian random number with power equal to interpolation model variance
		boost::normal_distribution<> nd(rgen_mean, sqrt(filtVariance_buf1[textNum]));
		boost::variate_generator<boost::mt19937&,
			boost::normal_distribution<> > var_nor(rng, nd);
		excitation = var_nor();
		output = 0.0;

		//if the size of output history is less than the number of AR coefficients, append zeros
		if (outputHist[textNum].size()<(unsigned int)MAX_COEFF) {
			int subt = MAX_COEFF - outputHist[textNum].size();
			for (int j = 0; j < subt; j++) {
				outputHist[textNum].push_back(0.0);
			}
		}
		//if the size of excitation history is less than the number of MA coefficients, append zeros
		if (excitationHist[textNum].size()<(unsigned int)MAX_MACOEFF) {
			int subt = MAX_MACOEFF - excitationHist[textNum].size();
			for (int j = 0; j < subt; j++) {
				excitationHist[textNum].push_back(0.0);
			}
		}

		//apply AR coefficients to history of output values
		for (int i = 0; i < coeffNum[textNum]; i++) {
			output += outputHist[textNum].at(i) * (-filtCoeff_buf1[textNum][i]);
		}

		//Debug
		//LSFout << fNorm << ", " << vTan << ", " << output << ", " << excitation << endl;

		output += excitation;

		//if the size of output history is greater than the number of AR coefficients, make the extra values zero so we're not storing junk
		if (outputHist[textNum].size()>(unsigned int) coeffNum[textNum]) {
			for (unsigned int kk = coeffNum[textNum]; kk < outputHist[textNum].size(); kk++)
				outputHist[textNum].at(kk) = 0.0;
		}
		//if the size of excitation history is greater than the number of MA coefficients, make the extra values zero so we're not storing junk
		if (excitationHist[textNum].size()>(unsigned int) MAcoeffNum) {
			for (unsigned int kk = MAcoeffNum; kk < excitationHist[textNum].size(); kk++)
				excitationHist[textNum].at(kk) = 0.0;
		}

	}
	else {//if buffer 2
		  //generate Gaussian random number with power equal to interpolation model variance
		boost::normal_distribution<> nd(rgen_mean, sqrt(filtVariance_buf2[textNum]));
		boost::variate_generator<boost::mt19937&,
			boost::normal_distribution<> > var_nor(rng, nd);
		excitation = var_nor();
		output = 0.0;

		//if the size of output history is less than the number of AR coefficients, append zeros
		if (outputHist[textNum].size()<(unsigned int)MAX_COEFF) {
			int subt = MAX_COEFF - outputHist[textNum].size();
			for (int j = 0; j < subt; j++) {
				outputHist[textNum].push_back(0.0);
			}
		}
		//if the size of excitation history is less than the number of MA coefficients, append zeros
		if (excitationHist[textNum].size()<(unsigned int)MAX_MACOEFF) {
			int subt = MAX_MACOEFF - excitationHist[textNum].size();
			for (int j = 0; j < subt; j++) {
				excitationHist[textNum].push_back(0.0);
			}
		}

		//apply AR coefficients to history of output values
		for (int i = 0; i < coeffNum[textNum]; i++) {
			output += outputHist[textNum].at(i) * (-filtCoeff_buf2[textNum][i]);
		}

		//Debug
		//LSFout << fNorm << ", " << vTan << ", " << output << ", " << excitation << endl;

		//if applicable, also apply MA coefficients to history of excitation values
		output += excitation;


		//if the size of output history is greater than the number of AR coefficients, make the extra values zero so we're not storing junk
		if (outputHist[textNum].size()>(unsigned int) coeffNum[textNum]) {
			for (unsigned int kk = coeffNum[textNum]; kk < outputHist[textNum].size(); kk++) {
				outputHist[textNum].at(kk) = 0.0;
			}
		}
		//if the size of excitation history is greater than the number of MA coefficients, make the extra values zero so we're not storing junk
		if (excitationHist[textNum].size()>(unsigned int) MAcoeffNum) {
			for (unsigned int kk = MAcoeffNum; kk < excitationHist[textNum].size(); kk++)
				excitationHist[textNum].at(kk) = 0.0;
		}
	}

	// remove the last element of our output vector
	outputHist[textNum].pop_back();
	excitationHist[textNum].pop_back();
	// push our new ouput value onto the front of our vector stack
	outputHist[textNum].insert(outputHist[textNum].begin(), output);
	excitationHist[textNum].insert(excitationHist[textNum].begin(), excitation);

	return (output * m_vibGain); //this is the output vibration value (in m/s^2)
}

void LSFModel::setForceandVelocity(double _fNorm, double _vTan)
{
	if (_fNorm <= forceNormCoeff[textNum])
	{
		fNorm = _fNorm;
		m_vibGain = 1.0;
	}
	else
	{
		fNorm = forceNormCoeff[textNum];
		m_vibGain = _fNorm / forceNormCoeff[textNum];
	}
		

	if (_vTan <= velNormCoeff[textNum])
		vTan = _vTan;
	else
		vTan = velNormCoeff[textNum];

	ARFromLsf(vTan, fNorm);
}

double LSFModel::noiseVibration()
{
	double noise = 0.0;
	double rgen_mean = 0.;
	boost::mt19937 generator;

	//generate Gaussian random number with power equal to interpolation model variance
	if (SynthesisFlag_Buffer1)
	{
		boost::normal_distribution<> nd(rgen_mean, sqrt(filtVariance_buf1[textNum]));
		boost::variate_generator<boost::mt19937&,
			boost::normal_distribution<> > var_nor(rng, nd);
		noise = var_nor();
	}
	else
	{
		boost::normal_distribution<> nd(rgen_mean, sqrt(filtVariance_buf2[textNum]));
		boost::variate_generator<boost::mt19937&,
			boost::normal_distribution<> > var_nor(rng, nd);
		noise = var_nor();
	}

	return noise;
}

void LSFModel::loadModels()
{	
	for (int i = 0; i < NUM_TEX; i++)
	{
		string textureName(texArray[i]);
		string textureFileName = "../../resources/textures/LSF/" + textureName + ".txt";

		FILE * fail = fopen(textureFileName.c_str(), "r");

		fscanf(fail, "%lf", &velNormCoeff[i]);
		fscanf(fail, "%lf", &forceNormCoeff[i]);


		fscanf(fail, "%d", &rbfNum[i]);
		fscanf(fail, "%d", &nARCoeff[i]);


		if (rbfNum[i] * nARCoeff[i] <= MAX_NUM_RBF) {
			for (int j = 0; j < rbfNum[i] * nARCoeff[i]; j++) {
				fscanf(fail, "%lf", &rbfCoeff[i][j]);
			}

			for (int j = 0; j<rbfNum[i]; j++) {
				fscanf(fail, "%f %f", &rbfCenters[i][j][0], &rbfCenters[i][j][1]);
			}

		}
		else {
			printf("Number of RBF exceeded maximum");
		}

		for (int j = 0; j < 3 * nARCoeff[i]; j++)
			fscanf(fail, "%lf", &polyCoeff[i][j]);

		// Make competible with old code
		//coeffNum[200] = nARCoeff[NUM_TEX] - 2;
	} 

	cout << "LSF Load succeed" << endl;
}

double LSFModel::interWeightsRbf(float v_x, float f_n, int numARCoeff) {
	double result = 0;
	for (int i = 0; i<rbfNum[textNum]; i++) {
		double r = (v_x - rbfCenters[textNum][i][0]) * (v_x - rbfCenters[textNum][i][0]) +
			(f_n - rbfCenters[textNum][i][1]) * (f_n - rbfCenters[textNum][i][1]);
		// to be used kernel
		result += sqrt(r)*rbfCoeff[textNum][i + numARCoeff * rbfNum[textNum]];
	}
	return result;
}

double LSFModel::interPolyCoeff(float v_x, float f_n, int numARCoeff) {
	double result = 0;
	result = (1 * polyCoeff[textNum][0 + 3 * numARCoeff] +
		v_x * polyCoeff[textNum][1 + 3 * numARCoeff] +
		f_n * polyCoeff[textNum][2 + 3 * numARCoeff]);
	return result;
}

double LSFModel::interpolateRbf(float v_x, float f_n, int numARCoeff) {
	return interWeightsRbf(v_x, f_n, numARCoeff)
		+ interPolyCoeff(v_x, f_n, numARCoeff);
}

void LSFModel::normInput(float &v_x, float &f_n) {
	v_x /= velNormCoeff[textNum];
	f_n /= forceNormCoeff[textNum];
}

void LSFModel::ARFromLsf(float v_x, float f_n) {

	normInput(v_x, f_n);
	coeffNum[textNum] = nARCoeff[textNum] - 2;

	for (int i = 0; i < coeffNum[textNum]; i++)
	{
		filtLSF[textNum][i] = interpolateRbf(v_x, f_n, i);
	}

	complex<float> pAR[MAX_COEFF]; //roots of P
	complex<float> rQ[MAX_COEFF]; //roots of Q
	complex<float> Q[MAX_COEFF + 1];
	complex<float> P[MAX_COEFF + 1];
	complex<float> Q1[MAX_COEFF + 2]; //sum filter
	complex<float> P1[MAX_COEFF + 2]; //difference filter
	float AR[MAX_COEFF + 1];
	complex<float> rP[MAX_COEFF];

	for (int i = 0; i< coeffNum[textNum]; i++) {

		complex<float> mycomplex(0, filtLSF[textNum][i]);
		pAR[i] = exp(mycomplex);// e^i*lsf

		if (i % 2 == 0) { // separate the odd index results from the even index results
			rQ[i / 2] = pAR[i];
		}
		else {
			rP[(i - 1) / 2] = pAR[i];
		}
	}

	//if even number of coefficients
	if (coeffNum[textNum] % 2 == 0) {
		for (int i = coeffNum[textNum] / 2; i<coeffNum[textNum]; i++) {
			rQ[i] = conj(rQ[i - coeffNum[textNum] / 2]);//add the conjugates of the values to the end of the lists
			rP[i] = conj(rP[i - coeffNum[textNum] / 2]);
		}
		P[0] = 1;//P and Q are vectors of 0, starting with a 1
		Q[0] = 1;
		for (int i = 1; i <= coeffNum[textNum]; i++) {
			Q[i] = 0;
			P[i] = 0;
		}

		//Form the polynomials P and Q, these should be real
		for (int kQ = 0; kQ<coeffNum[textNum]; kQ++) {
			for (int i = kQ + 1; i >= 1; i--) {
				Q[i] = Q[i] - rQ[kQ] * Q[i - 1];
			}
		}
		for (int kP = 0; kP<coeffNum[textNum]; kP++) {
			for (int i = kP + 1; i >= 1; i--) {
				P[i] = P[i] - rP[kP] * P[i - 1];
			}
		}

		float vp[2];
		vp[0] = 1;
		vp[1] = -1;
		int mp = coeffNum[textNum] + 1;
		int np = 2;
		//form difference filter by including root at z=1
		//P1 = conv(P,[1 -1])
		for (int kp = 1; kp<mp + np; kp++) {
			int j1p = max(1, kp + 1 - np);
			int j2p = min(kp, mp);
			P1[kp - 1] = 0;
			for (int jp = j1p; jp <= j2p; jp++) {
				P1[kp - 1] = P1[kp - 1] + P[jp - 1] * vp[kp - jp];
			}
		}

		float vq[2];
		vq[0] = 1;
		vq[1] = 1;
		int mq = mp;
		int nq = np;
		//form sum filter by including root at z=-1
		//Q1 = conv(Q,[1 1])
		for (int kq = 1; kq<mq + nq; kq++) {
			int j1q = max(1, kq + 1 - nq);
			int j2q = min(kq, mq);
			Q1[kq - 1] = 0;
			for (int jq = j1q; jq <= j2q; jq++) {
				Q1[kq - 1] = Q1[kq - 1] + Q[jq - 1] * vq[kq - jq];
			}
		}

		//Average the real values for P and Q
		for (int i = 0; i<coeffNum[textNum] + 1; i++) {
			AR[i] = (P1[i].real() + Q1[i].real()) / 2;
		}
	}//end even number of coefficients case

	 //odd number of coefficients... same thing, but shifted
	else {
		//add the conjugates of the values to the end of the lists
		for (int i = (coeffNum[textNum] + 1) / 2; i<coeffNum[textNum] + 1; i++) {
			rQ[i] = conj(rQ[i - (coeffNum[textNum] + 1) / 2]);
		}
		for (int i = (coeffNum[textNum] - 1) / 2; i<coeffNum[textNum] - 1; i++) {
			rP[i] = conj(rP[i - (coeffNum[textNum] - 1) / 2]);
		}
		P[0] = 1; //P and Q are vectors of 0, starting with a 1
		Q[0] = 1;
		for (int i = 1; i <= coeffNum[textNum] + 1; i++) {
			Q[i] = 0;
		}

		//Form the polynomials P and Q, these should be real
		for (int i = 1; i <= coeffNum[textNum] - 1; i++) {
			P[i] = 0;
		}

		for (int kQ = 0; kQ<coeffNum[textNum] + 1; kQ++) {// the order in which it is looped matters
			for (int i = kQ + 1; i >= 1; i--) {
				Q[i] = Q[i] - rQ[kQ] * Q[i - 1];
			}
		}

		for (int kP = 0; kP<coeffNum[textNum] - 1; kP++) {
			for (int i = kP + 1; i >= 1; i--) {
				P[i] = P[i] - rP[kP] * P[i - 1];
			}
		}

		float v[3];
		v[0] = 1;
		v[1] = 0;
		v[2] = -1;
		int m = coeffNum[textNum];
		int n = 3;
		//form difference filter by including root at z=+1 and z=-1
		//P1 = conv(P,[1 0 -1])
		for (int k = 1; k<m + n; k++) {
			int j1 = max(1, k + 1 - n);
			int j2 = min(k, m);
			P1[k - 1] = 0;
			for (int j = j1; j <= j2; j++) {
				P1[k - 1] = P1[k - 1] + P[j - 1] * v[k - j];
			}
		}
		//Average the real values for P and Q
		for (int i = 0; i<coeffNum[textNum] + 1; i++) {
			AR[i] = (P1[i].real() + Q[i].real()) / 2;
		}
	}// end of odd number of coefficients

	 //Update appropriate buffer with AR coefficients
	for (int i = 1; i <= coeffNum[textNum]; i++) {
		if (SynthesisFlag_Buffer1[textNum]) {
			filtCoeff_buf2[textNum][i - 1] = AR[i];
		}
		else {
			filtCoeff_buf1[textNum][i - 1] = AR[i];
		}
	}

	//
	if (SynthesisFlag_Buffer1[textNum])
	{
		// interpolate the variance
		filtVariance_buf2[textNum] = abs(interpolateRbf(v_x, f_n, coeffNum[textNum] + 1)); // last coeff from rbf is variance (2n from end is gain(unused))
		SynthesisFlag_Buffer1[textNum] = false;
	}

	else
	{
		// interpolate the variance
		filtVariance_buf1[textNum] = abs(interpolateRbf(v_x, f_n, coeffNum[textNum] + 1)); // last coeff from rbf is variance (2n from end is gain(unused))
		SynthesisFlag_Buffer1[textNum] = true;
	}
}


LSFModel::~LSFModel()
{
	LSFout.close();
};