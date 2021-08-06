#ifndef _MADGWICKAHRSUPDATEIMU_
#define _MADGWICKAHRSUPDATEIMU_
#include "coodtransformation.h"
#include <atomic>
#include<math.h>
#define PI acos(-1)
#define sampleFreq	400.0f			// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain
class MadgwickAHRSupdateIMU :
	public coodTransformation
{

	public:
		std::atomic<float> beta;				// algorithm gain
		MadgwickAHRSupdateIMU();
		std::vector<float> PostureFusion(std::vector<float> q, float gx, float gy, float gz, float ax, float ay, float az);
};
#endif 
