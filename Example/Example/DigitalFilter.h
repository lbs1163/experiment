#pragma once

// MATLAB Filter�� C ����

#define nA 6 
#define nB 6 //input x, y parameter�� order ��. ���Ϳ� �°� ������ ��.

class DigitalFilter
{
public:
	DigitalFilter()
	{
		parameterA[0] = 1.0;
		parameterA[1] = -4.89833714571160;
		parameterA[2] = 9.59849709080560;
		parameterA[3] = -9.40530798919573;
		parameterA[4] = 4.60847635853691;
		parameterA[5] = -0.903328285338000;
		
		parameterB[0] = 9.09286590466962e-10;
		parameterB[1] = 4.54643295233481e-09;
		parameterB[2] = 9.09286590466962e-09;
		parameterB[3] = 9.09286590466962e-09;
		parameterB[4] = 4.54643295233481e-09;
		parameterB[5] = 9.09286590466962e-10;
		//Filter parameter �ʱ�ȭ

		for (int index = 0; index < nA; index++)
		{
			outputQueue[index] = 0.0;
		}
		for (int index = 0; index < nB; index++)
		{
			inputQueue[index] = 0.0;
		}
		//Input, output queue �ʱ�ȭ
	}

	double Filtering(double input_value)
	{
		double output_value = 0.0;

		//printf_s("Input value %f, ", input_value);

		// for: Queue���׿��ִ�������������ĭ���о, �ǵ帱�ʿ����
		for (int index = 0; index < nA - 1; index++)
		{
			outputQueue[index] = outputQueue[index + 1];
		}
		
		for (int index = 0; index < nB-1; index++)
		{
			inputQueue[index] = inputQueue[index + 1];
		}

		inputQueue[nB - 1] = input_value;
		outputQueue[nA - 1] = 0.0;

		for (int index = 1; index < nA; index++)
		{
			outputQueue[nA-1] = outputQueue[nA - 1] - (outputQueue[nA-1-index] * parameterA[index]);
		}

		//printf_s("After output sum: %f, ", outputQueue[nA - 1]);

		for (int index = 0; index < nB; index++)
		{
			outputQueue[nA -1] = outputQueue[nA - 1] + ( inputQueue[nB-1-index] * parameterB[index]);
		}
		outputQueue[nA - 1] = outputQueue[nA - 1] / parameterA[0];

		//printf_s("After input sum: %f\n", outputQueue[nA - 1]);


		return output_value = outputQueue[nA-1]; // Filtering �� �������� ��ȯ
	}

	~DigitalFilter() {}

private:
	double parameterA[nA];
	double parameterB[nB];
	double inputQueue[nB];
	double outputQueue[nA];
}; 
#pragma once
