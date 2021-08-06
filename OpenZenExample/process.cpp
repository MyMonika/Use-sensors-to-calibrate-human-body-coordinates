#include <process.h>
#include "coodTransformation.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include<math.h>
#include <atomic>
#include <iostream>
#include <queue>
#include "Scanner.h"
#include "Calculator.h"
void Process::Start(Scanner* s, std::vector<coodTransformation*> method) {
	//首先是读取
	static bool once = true;
	if (once)
		Read(s,method);
	once = false;
	//然后将读取到的数据进行校准。也就是位姿融合，由于要读够20组，所以把位姿融合之间放到扫描与读取里面了
	//使用内蕴平均算法

	//等到这里就会由电脑自动处理数据了
	//我们来进行截取
}

void Process::Read(Scanner* s,std::vector<coodTransformation*> method) {
	s->ScanAndRead( method);
}
Eigen::Matrix3f* Process::EyeInHand(std::queue<Eigen::Matrix3d> parent, std::queue<Eigen::Matrix3d> child) {

	return Calculator::EyeInHand( parent, child, ca);
}
void Process::Update(int index) {
	std::cout<<"由子类实现调用"<<std::endl;
}
