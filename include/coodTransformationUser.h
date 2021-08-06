#ifndef _COODTRANSFORMATIONUSER_
#define _COODTRANSFORMATIONUSER_

#include <coodTransformation.h>
#include <iostream>
class coodTransformationUser :public coodTransformation
{
	private:
		coodTransformation* ob;
	public:
		void SetCoodTransformation(coodTransformation* x);
		coodTransformationUser();
		std::vector<float> PostureFusion(std::vector<float> q, float gx, float gy, float gz, float ax, float ay, float az);
		std::vector<float> Current_fixed_quaternion(std::vector<coodTransformation*> x, std::vector<float> q, float gx, float gy, float gz, float ax, float ay, float az);

};
#endif

