#include "ProcessUser.h"
#include <coodTransformation.h>
#include "Calculator.h"
#include <iostream>
ProcessUser::ProcessUser(int i, int j) {
	index1 = i;
	index2 = j;
}
std::vector<float>  ProcessUser::Current_fixed_quaternion(std::vector<coodTransformation*> x, std::vector<float> q, float gx, float gy, float gz, float ax, float ay, float az)
{
	return coodTransformationUser::Current_fixed_quaternion( x,  q, gx, gy, gz, ax, ay, az);
}

Eigen::Matrix3f* ProcessUser::EyeInHand( std::queue<Eigen::Matrix3d> parent, std::queue<Eigen::Matrix3d> child)
{
	return Process::EyeInHand(parent, child);
}

void ProcessUser::Update(int index) {
	//求解内蕴标定方程，统计IA次数
	//下一步是得到并求解手眼标定方程,内部是Y与X的新解

	//只接受index1与index2的参数
	if (index == index1 || index == index2) {

		//std::queue<Eigen::Matrix3d> mem = Calculator::IntrinsicAverage(Scanner::sensorData[index]);
		//Scanner::IA[index]=mem;
		//std::cout << "mem[0]=   " << mem.back() << std::endl;
		Scanner::IA[index] = Scanner::sensorData[index];
		if (Scanner::IA[index].size() < 2)
			return;
		else {
			Scanner::IAisFull[index] = true;
		}
		if (Scanner::IAisFull[index1] == true && Scanner::IAisFull[index2] == true) {
			//IA內部存储了计算的两次A或者B，所以
			std::cout << "可以计数了" << std::endl;
			Eigen::Matrix3f* YX = EyeInHand(Scanner::IA[0], Scanner::IA[1]);

			Scanner::IAisFull[index1] = false;
			Scanner::IAisFull[index2] = false;
		}
	}
}

