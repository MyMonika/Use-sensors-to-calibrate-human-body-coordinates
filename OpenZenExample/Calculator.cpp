#include "Calculator.h"
#include <iostream>
#include<math.h>
#include "Calculator.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include <queue>
#define PI acos(-1)
Eigen::Matrix3d Calculator::last_matrix_bb(3,3);
float* Calculator::Quaterniond2Euler(std::vector<float> q)
{
	float euler[3] = { 0,0,0 };
	Eigen::Quaterniond quaternion4(q[0], q[1], q[2], q[3]);
	//	float q0, q1, q2, q3;
		//q0 = q[0];
		//q1 = q[1];
		//q2 = q[2];
		//q3 = q[3];

	euler[0] = asin(2 * (q[0] * q[2] - q[1] * q[3]));
	euler[1] = atan2((q[0] * q[3] + q[1] * q[2]), (1 - 2 * (pow(q[2], 2) + pow(q[3], 2))));
	euler[2] = atan2((q[0] * q[1] + q[3] * q[2]), (1 - 2 * (pow(q[2], 2) + pow(q[1], 2))));
	std::cout << " roll(x) = " << euler[2] * 360 / (2 * PI) << std::endl;
	std::cout << " pitch(y) = " << euler[1] * 360 / (2 * PI) << std::endl;
	std::cout << " yaw(z)  = " << euler[0] * 360 / (2 * PI) << std::endl;
	return euler;
}

float  Calculator::invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
void Calculator::RotationToEular(Eigen::Matrix3f mat) {
	// 使用Eigen将旋转矩阵转换为欧拉角
	Eigen::Vector3f eulerAngle1 = mat.eulerAngles(2, 1, 0); // ZYX顺序，yaw,pitch,roll

	std::cout << "roll_1 pitch_1 yaw_1 = " << eulerAngle1[2] << " " << eulerAngle1[1]
		<< " " << eulerAngle1[0] << std::endl;
}
Eigen::Matrix3d Calculator::QuaternionToRotation(std::vector<float> q) {

	Eigen::Quaterniond quaternion_es(q[0], q[1], q[2], q[3]);
	Eigen::Matrix3d rotationMatrix = quaternion_es.toRotationMatrix();
	return rotationMatrix;
}
//基坐标系下的两个旋转坐标系的相对旋转方阵
Eigen::Matrix3d Calculator::RotationTwoCood(Eigen::Matrix3d matrix_bb_parents,Eigen::Matrix3d matrix_bb_child) {
	Eigen::Matrix3d res = matrix_bb_parents*matrix_bb_child.inverse();
	//std::cout <<"对称性= \n   "<< matrix_bb_child.inverse()*matrix_bb_child << std::endl;;
	return res;
}
//内蕴标定算法
std::queue<Eigen::Matrix3d> Calculator::IntrinsicAverage(std::queue<Eigen::Matrix3d> parent) {
	int len_p = parent.size();
	std::vector<Eigen::Matrix3d> p2;
	p2.reserve(2);
	std::queue<Eigen::Matrix3d> parent2;
	Eigen::Matrix3d sum2= Eigen::MatrixXd::Zero(3, 3);
	//std::cout << sum2 << std::endl;;
	Eigen::Matrix3d rotationMatrix_es_parents_1;
	while (!parent.empty()) {

		rotationMatrix_es_parents_1 = parent.back();
		//std::cout << "parent1=" << rotationMatrix_es_parents_1 << std::endl;;
		Eigen::Matrix3d rotationMatrix_es_parents_2 = rotationMatrix_es_parents_1.transpose()*rotationMatrix_es_parents_1;
		//std::cout << "parent2=" << rotationMatrix_es_parents_2<< std::endl;;
		rotationMatrix_es_parents_2 = rotationMatrix_es_parents_1.array().log();
		p2.push_back(rotationMatrix_es_parents_1);
		parent.pop();
		sum2 += rotationMatrix_es_parents_2;
		//std::cout << sum2 <<std::endl;;

	}
	sum2 = (sum2 / len_p).array().exp();
	for(auto it:p2)
		parent2.push(it*sum2);
	auto res = sum2.norm();
	if (res<1) return IntrinsicAverage(parent2);
	return parent2;

}
Eigen::Matrix3f* Calculator::EyeInHand(std::queue<Eigen::Matrix3d> parent, std::queue<Eigen::Matrix3d> child, Calculator* ca) {
	Eigen::Matrix3d rotationMatrix_es_parents_1;
	Eigen::Matrix3d rotationMatrix_es_parents_2;
	Eigen::Matrix3d rotationMatrix_es_child_1;
	Eigen::Matrix3d rotationMatrix_es_child_2;
	Eigen::Matrix3f A;
	Eigen::Matrix3f B;
	Eigen::Matrix3d RSB(3, 3);

	//把es的四元数转化为旋转矩阵
	//由于需要两个连续时间点的数据，我们可以使用
	//确定队列里面有超过两个元素时候
	if (parent.size() > 1 && child.size() > 1) {
		rotationMatrix_es_parents_1 = (parent.back());
		rotationMatrix_es_child_1 = (child.back());
		parent.pop();
		child.pop();
		rotationMatrix_es_parents_2 = (parent.back());
		rotationMatrix_es_child_2 = (child.back());
		//std::cout << "rotationMatrix_es_parents_2=   " << rotationMatrix_es_parents_2 << std::endl;
		//std::cout << "rotationMatrix_es_child_2=   " << rotationMatrix_es_child_2 << std::endl;

		//得到标定矩阵A
		A = (rotationMatrix_es_parents_2.transpose()*rotationMatrix_es_child_2*
			rotationMatrix_es_child_1.transpose()*rotationMatrix_es_parents_1).cast<float>();
	}
	//Eigen::Matrix3f A = A1.cast<float>();
	//得到标定矩阵B
	Eigen::Matrix3d new_matrix = Calculator::RotationTwoCood(rotationMatrix_es_parents_2, rotationMatrix_es_child_2);
	//std::cout << "new_matrix=   " << new_matrix << std::endl;
	B = (new_matrix * Calculator::last_matrix_bb.transpose()).cast<float>();
	//std::cout << "A=   " << A << std::endl;
	//std::cout << "B=   " << B << std::endl;
	//更新标定的矩阵
	Calculator::last_matrix_bb = new_matrix;
	//得到X的解
	RSB = ca->tari.sovleAXequalXB(A, B).cast<double>();
	//std::cout << "RSB= \n   " << RSB << std::endl;
	//R_child,我们这里使用第二次进行标定
	Eigen::Matrix3d R_child_S = (rotationMatrix_es_child_2.transpose()*rotationMatrix_es_parents_2*RSB*new_matrix);
	Eigen::Matrix3d R_Parents_S = (rotationMatrix_es_parents_2.transpose()*rotationMatrix_es_child_2*R_child_S*new_matrix.transpose());

	Eigen::Matrix3f YX[2] = { R_child_S.cast<float>(),R_Parents_S.cast<float>()};
	//std::cout << "R_child_S= \n   " << R_child_S << std::endl;
	//std::cout << "R_Parents_S= \n   " << R_Parents_S << std::endl;
	//打印结果：：

	Calculator::RotationToEular(YX[0]);
	Calculator::RotationToEular(YX[1]);

	return YX;
}
//将旋转矩阵从传感器坐标转化为世界坐标
Eigen::Matrix3d Calculator::SensorToEnvir(Eigen::Matrix3d Sensor) {
	//传感器是起初与世界坐标重合，之后再绕世界的Y轴顺时针90度，再绕世界的Z轴逆时针90度
	Eigen::AngleAxisd A1(PI / 4, Eigen::Vector3d(0, -1, 0));//以（0,1,0）为旋转轴，旋转90度(默认逆时针)
	Eigen::AngleAxisd A2(PI / 4, Eigen::Vector3d(0, 0, 1));//以（0,0,1）为旋转轴，旋转90度(默认逆时针)
	Eigen::Matrix3d senToEnv = A2.matrix()*A1.matrix()*Sensor;
	return senToEnv;
}