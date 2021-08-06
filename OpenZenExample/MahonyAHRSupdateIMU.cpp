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

	// ֻ�ڼ��ٶȼ�������Чʱ�Ž�������
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// �����ٶȵ�λ��
		recipNorm = Calculator::invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// ͨ����Ԫ���õ������������ٶ�����g 
		// ע�⣬����ʵ�����Ǿ��������*1/2�������ڿ�ͷ��Kp Ki�ĺ궨���Ϊ2*����
		// ��������Ŀ���Ǽ��ٳ˷�������
		halfvx = q[1] * q[3] - q[0] * q[2];
		halfvy = q[0] * q[1] + q[2] * q[3];
		halfvz = q[0] * q[0] - 0.5f + q[3] * q[3];


		// ��ʵ���������ٶ�����v�������������ٶ�����g�����,�Ӷ��������
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// ��PI�������л�����ʹ������¼��㲢Ӧ�û�����
		if (twoKi > 0.0f) {
			// ���ֹ���
			integralFBx = twoKi * halfex * (1.0f / sampleFreq) + integralFBx;
			integralFBy = twoKi * halfey * (1.0f / sampleFreq) + integralFBy;
			integralFBz = twoKi * halfez * (1.0f / sampleFreq) + integralFBy;

			// Ӧ�������еĻ�����
			gx += integralFBx;
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			// ����Ϊ��ֵ��Kiʱ�����쳣����
			integralFBx = 0.0f;
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Ӧ�������еı�����
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// ʹ����������ⷨ�õ���Ԫ�����
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

	// ��λ����Ԫ�� ��֤��Ԫ���ڵ��������б��ֵ�λ����
	recipNorm = Calculator::invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	for (int i = 0; i < 4; i++) {
		q[i] *= recipNorm;
	}
	//std::cout << "�����mehony��0 1 2 3 ��" << q[0] << "    " <<
	//	q[1] << "    " << q[2] << "    " << q[3] << std::endl;
	return q;
}