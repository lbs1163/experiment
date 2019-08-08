#pragma once
#include <vector>
#include <numeric>
// Moving average filter

namespace chai3d {

	class MAF
	{
	public:
		MAF() : length(5)
		{
			data_array.assign(length, 0.0);
		}
		MAF(int a_length)
		{
			length = a_length;
			data_array.assign(a_length, 0.0);
		}

		double Filtering(double input_value)
		{
			double result;
			data_array.insert(data_array.begin(), input_value);
			data_array.pop_back();
			result = std::accumulate(data_array.begin(), data_array.end(), 0.0) / (double)length;
			return result;
		}

		void init()
		{
			data_array.assign(length, 0.0);
		}

		~MAF() {	}

	private:
		std::vector<double> data_array;
		int length;
	};
}
#pragma once
