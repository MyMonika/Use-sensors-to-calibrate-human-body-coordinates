#include "MahonyAHRSupdateIMU.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include<math.h>
#include <atomic>
#include <iostream>
#include "Calculator.h"
MahonyAHRSupdateIMU::MahonyAHRSupdateIMU() {
	twoKp = twoKpDef;
	twoKi = twoKiDef;
	integralFBx = 0.0f;
	integralFBy = 0.0f;
	integralFBz = 0.0f;
}
std::vector<float> MahonyAHRSupdateIMU::PostureFusion(std::vector<float> q, float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// 只在加速度计数据有效时才进行运算
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// 将加速度单位化
		recipNorm = Calculator::invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// 通过四元数得到理论重力加速度向量g 
		// 注意，这里实际上是矩阵第三列*1/2，所以在开头对Kp Ki的宏定义均为2*增益
		// 这样处理目的是减少乘法运算量
		halfvx = q[1] * q[3] - q[0] * q[2];
		halfvy = q[0] * q[1] + q[2] * q[3];
		halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];


		// 对实际重力加速度向量v与理论重力加速度向量g做外积,从而计算误差
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// 在PI补偿器中积分项使能情况下计算并应用积分项
		if (twoKi > 0.0f) {
			// 积分过程
			integralFBx = twoKi * halfex * (1.0f / sampleFreq) + integralFBx;
			integralFBy = twoKi * halfey * (1.0f / sampleFreq) + integralFBy;
			integralFBz = twoKi * halfez * (1.0f / sampleFreq) + integralFBy;

			// 应用误差补偿中的积分项
			gx += integralFBx;
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			// 避免为负值的Ki时积分异常饱和
			integralFBx = 0.0f;
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// 应用误差补偿中的比例项
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// 使用龙格库塔解法得到四元数结果
	gx *= (0.5f * (1.0f / sampleFreq));
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q[0];
	qb = q[1];
	qc = q[2];
	q[0] += (-qb * gx - qc * gy - q[3] * gz);
	q[1] += (qa * gx + qc * gz - q[3] * gy);
	q[2] += (qa * gy - qb * gz + q[3] * gx);
	q[3] += (qa * gz + qb * gy - qc * gx);

	// 单位化四元数 保证四元数在迭代过程中保持单位性质
	recipNorm = Calculator::invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	for (int i = 0; i < 4; i++) {
		q[i] *= recipNorm;
	}
	//std::cout << "结果是mehony的0 1 2 3 是" << q[0] << "    " <<
	//	q[1] << "    " << q[2] << "    " << q[3] << std::endl;
	return q;
}