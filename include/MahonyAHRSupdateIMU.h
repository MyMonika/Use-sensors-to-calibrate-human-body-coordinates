#ifndef _MAHONYAHRSUPDATEIMU_
#define _MAHONYAHRSUPDATEIMU_
#include "coodtransformation.h"
#include <atomic>
#define sampleFreq	400.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.3f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.1f)	// 2 * integral gain
class MahonyAHRSupdateIMU :
	public coodTransformation
{

	public:
		std::atomic<float> twoKp;											// 2 * proportional gain (Kp)
		std::atomic<float> twoKi;											// 2 * integral gain (Ki)
		std::atomic<float> integralFBx;
		std::atomic<float> integralFBy;
		std::atomic<float> integralFBz;	// integral error terms scaled by Ki

		MahonyAHRSupdateIMU();
		std::vector<float> PostureFusion(std::vector<float> q, float gx, float gy, float gz, float ax, float ay, float az) ;

};
#endif 
