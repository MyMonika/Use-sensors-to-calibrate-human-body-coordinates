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
	//�����Ƕ�ȡ
	static bool once = true;
	if (once)
		Read(s,method);
	once = false;
	//Ȼ�󽫶�ȡ�������ݽ���У׼��Ҳ����λ���ںϣ�����Ҫ����20�飬���԰�λ���ں�֮��ŵ�ɨ�����ȡ������
	//ʹ������ƽ���㷨

	//�ȵ�����ͻ��ɵ����Զ�����������
	//���������н�ȡ
}

void Process::Read(Scanner* s,std::vector<coodTransformation*> method) {
	s->ScanAndRead( method);
}
Eigen::Matrix3f* Process::EyeInHand(std::queue<Eigen::Matrix3d> parent, std::queue<Eigen::Matrix3d> child) {

	return Calculator::EyeInHand( parent, child, ca);
}
void Process::Update(int index) {
	std::cout<<"������ʵ�ֵ���"<<std::endl;
}
