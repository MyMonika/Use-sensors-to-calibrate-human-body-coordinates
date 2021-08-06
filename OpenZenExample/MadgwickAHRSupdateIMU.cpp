#include "MadgwickAHRSupdateIMU.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include<math.h>
#include <atomic>
#include <iostream>
#include "Calculator.h"

MadgwickAHRSupdateIMU::MadgwickAHRSupdateIMU() {


}
std::vector<float> MadgwickAHRSupdateIMU::PostureFusion(std::vector<float> q, float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// 陀螺仪四元数变化率
	qDot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
	qDot2 = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
	qDot3 = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
	qDot4 = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);

	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		recipNorm = Calculator::invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// 避免重复运算的辅助变量
		_2q0 = 2.0f * q[0];
		_2q1 = 2.0f * q[1];
		_2q2 = 2.0f * q[2];
		_2q3 = 2.0f * q[3];
		_4q0 = 4.0f * q[0];
		_4q1 = 4.0f * q[1];
		_4q2 = 4.0f * q[2];
		_8q1 = 8.0f * q[1];
		_8q2 = 8.0f * q[2];
		q0q0 = q[0] * q[0];
		q1q1 = q[1] * q[1];
		q2q2 = q[2] * q[2];
		q3q3 = q[3] * q[3];

		//梯度下降算法校正步骤
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q[3] - _2q1 * ax + 4.0f * q2q2 * q[3] - _2q2 * ay;
		recipNorm = Calculator::invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		//根据梯度计算
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q[0] += qDot1 * (1.0f / sampleFreq);
	q[1] += qDot2 * (1.0f / sampleFreq);
	q[2] += qDot3 * (1.0f / sampleFreq);
	q[3] += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = Calculator::invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
//	std::cout << "结果是madwick的0 1 2 3 是" << q[0] << "    " <<
		//q[1] << "    " << q[2] << "    " << q[3] << std::endl;
	return q;
}