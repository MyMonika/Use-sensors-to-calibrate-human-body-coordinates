#ifndef _COODTRANSFORMATION_
#define _COODTRANSFORMATION_
#include <iostream>
#include<math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include <vector>
class coodTransformation
{

	public:

	//º¯ÊýÉùÃ÷£º
		virtual std::vector<float> PostureFusion(std::vector<float> q, float gx, float gy, float gz, float ax, float ay, float az)=0;
		coodTransformation(void)= default;
};
#endif 
