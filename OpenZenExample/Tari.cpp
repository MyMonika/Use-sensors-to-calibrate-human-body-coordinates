#include "Tari.h"
#include <process.h>
#include "coodTransformation.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include<math.h>
#include <atomic>
#include <iostream>
#include <algorithm>
#include <vector>
#include "Calculator.h"

Tari::Tari() {

}
//用svd奇异值分解求伪逆，伪逆是行空间与列空间的
//伪逆相当于给非方阵求逆
Eigen::MatrixXf Tari::svdInverse(Eigen::MatrixXf  A)
{
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);//M=USV*
	float  pinvtoler = 1.e-6; //tolerance
	int row = A.rows();
	int col = A.cols();
	int k = std::min(row, col);
	Eigen::MatrixXf X = Eigen::MatrixXf::Zero(col, row);
	Eigen::MatrixXf singularValues_inv = svd.singularValues();//奇异值
	Eigen::MatrixXf singularValues_inv_mat = Eigen::MatrixXf::Zero(col, row);
	for (long i = 0; i < k; ++i) {
		if (singularValues_inv(i) > pinvtoler)
			singularValues_inv(i) = 1.0 / singularValues_inv(i);
		else singularValues_inv(i) = 0;
	}
	for (long i = 0; i < k; ++i)
	{
		singularValues_inv_mat(i, i) = singularValues_inv(i);
	}
	X = (svd.matrixV())*(singularValues_inv_mat)*(svd.matrixU().transpose());//X=VS+U*

	return X;
}
//转化为罗德里格斯（Rodrigues）旋转方程
Geo3d Tari::rodrigues2(RotMat matrix) {
	Eigen::JacobiSVD<Eigen::Matrix3f> svd(matrix, Eigen::ComputeFullV | Eigen::ComputeFullU);
	RotMat R = svd.matrixU() * svd.matrixV().transpose();
	double rx = R(2, 1) - R(1, 2);
	double ry = R(0, 2) - R(2, 0);
	double rz = R(1, 0) - R(0, 1);

	double s = sqrt((rx*rx + ry * ry + rz * rz)*0.25);
	double c = (R.trace() - 1) * 0.5;
	c = c > 1. ? 1. : c < -1. ? -1. : c;
	double theta = acos(c);

	if (s < XEPS)
	{
		double t;

		if (c > 0)
			rx = ry = rz = 0;
		else
		{
			t = (R(0, 0) + 1)*0.5;
			rx = sqrt(std::max(t, 0.0));
			t = (R(1, 1) + 1)*0.5;
			ry = sqrt(std::max(t, 0.0)) * (R(0, 1) < 0 ? -1.0 : 1.0);
			t = (R(2, 2) + 1)*0.5;
			rz = sqrt(std::max(t, 0.0)) * (R(0, 2) < 0 ? -1.0 : 1.0);

			if (fabs(rx) < fabs(ry) && fabs(rx) < fabs(rz) && (R(1, 2) > 0) != (ry*rz > 0))
				rz = -rz;
			theta /= sqrt(rx*rx + ry * ry + rz * rz);
			rx *= theta;
			ry *= theta;
			rz *= theta;
		}
	}
	else
	{
		double vth = 1 / (2 * s);
		vth *= theta;
		rx *= vth; ry *= vth; rz *= vth;
	}
	return Eigen::Vector3d(rx, ry, rz).cast<float>();
}
RotMat Tari::skew(Geo3d v)
{
	RotMat rot;
	rot.setZero();

	rot(0, 1) = -v(2);
	rot(0, 2) = v(1);
	rot(1, 2) = -v(0);

	rot(1, 0) = -rot(0, 1);
	rot(2, 0) = -rot(0, 2);
	rot(2, 1) = -rot(1, 2);

	return rot;
}

Eigen::Matrix3f Tari::sovleAXequalXB(Eigen::Matrix3f vA, Eigen::Matrix3f vB) {

	Eigen::Matrix3f H=Eigen::Matrix3f::Zero(3, 3);
	if (vA.rows()!= vB.rows()|| vA.cols() != vB.cols())
	{
		std::cout<<("A and B must be same size.\n");
		return H;
	}
	RotMat R_a, R_b;
	Geo3d r_a, r_b;
	//旋转矩阵,用来存储
	Eigen::MatrixXf A(3, 3);
	//平移矩阵，用来存储
	Eigen::MatrixXf b(3, 1);
	A.setZero();
	b.setZero();

	//得到两个位姿变换的三D矩阵,类型转化为float
	R_a = vA;
	R_b = vB;
	//位姿变换使用罗德里克特公式转化为向量
	Geo3d rod_a = rodrigues2(R_a);
	Geo3d rod_b = rodrigues2(R_b);
	std::cout << "rod_a= \n   " << rod_a << std::endl;
	std::cout << " rod_b= \n   " << rod_b << std::endl;
	//返回向量二范数的平方根,（开方再开方）
	float theta_a = rod_a.norm();
	float theta_b = rod_b.norm();
	//std::cout << "theta_a= \n   " << theta_a << std::endl;
	if(theta_a!=0) rod_a /= theta_a;
	if (theta_b != 0)rod_b /= theta_b;

	Geo3d P_a = 2 * sin(theta_a / 2)*rod_a;
	Geo3d P_b = 2 * sin(theta_b / 2)*rod_b;

	//求伪逆
	Eigen::Matrix3f rot = skew(Geo3d(P_b + P_a));
	Geo3d v = P_b - P_a;

	//从第0行开始往后三行
	A.middleRows(0, 3) = rot;
	b.middleRows(0, 3) = v;

	// A的伪逆矩阵
	Eigen::MatrixXf pinA = svdInverse(A);

	// 计算初始旋转向量
	Geo3d H_ba_prime = pinA * b;

	Geo3d H_ba = 2 * H_ba_prime / sqrt(1 + pow(H_ba_prime.norm(),2));
	Eigen::MatrixXf H_ba_Trs = H_ba.transpose();

	// 旋转向量转换成旋转矩阵
	RotMat R_ba = (1 - pow((H_ba.norm()) / 2,2)) * RotMat::Identity()
		+ 0.5 * (H_ba * H_ba_Trs + sqrt(4 - pow(H_ba.norm(),2))*skew(H_ba));
	//std::cout << " R_ba= \n   " << R_ba << std::endl;

	H = R_ba;

	//std::cout << "H: \n" << H<< std::endl;
	Eigen::Matrix3f	 AX = vA * H;
	Eigen::Matrix3f	 XB = H * vB;

	//计算误差
	Geo3d angles1 = AX.eulerAngles(0, 1, 2);
	Geo3d angles2 = XB.eulerAngles(0, 1, 2);
	//std::cout <<  ". Rotation Error: " << (angles1 - angles2).transpose() <<std:: endl;
	return H;
}

