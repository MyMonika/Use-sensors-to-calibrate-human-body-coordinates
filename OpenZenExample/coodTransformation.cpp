#include "coodTransformation.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include<math.h>
#include <atomic>
#include <iostream>
//�ҵ���������ȥ����Ƕȣ�����ѡ���˵ڶ�������
//ʹ��һ���������ı궨�������Խ�����б궨



//Eigen::Vector3d coodTransformation::Quaterniond2Euler(std::vector<float> q[4])
//{
//
//	Eigen::Quaterniond quaternion4(q[0],q[1],q[2],q[3]);
//
//	Eigen::Vector3d euler = quaternion4.matrix().eulerAngles(2, 1, 0);
//	//std::cout << "yaw(z) pitch(y) roll(x) = " << euler.transpose() <<std:: endl;
//	std::cout << " roll(x) = " << euler[2]*360/(2*PI) << std::endl;
//	std::cout << " pitch(y) = " << euler[1] * 360 / (2 * PI) << std::endl;
//	std::cout << " yaw(z)  = " << euler[0] * 360 / (2 * PI) << std::endl;
//	return euler;
//}