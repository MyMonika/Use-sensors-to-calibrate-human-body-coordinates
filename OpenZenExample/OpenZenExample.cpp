//����һ��ʼmain�̱߳�������һ�Σ�������ʱ���������̲߳��еġ�
//gyro�������ǣ�����һ���Ǽ��ٶȼ�
//һ��process������Դ�������������������
#include <processUser.h>
#include <iostream>
#include <string>
#include <vector>
#include "Calculator.h"
#include "Scanner.h"
#include "MahonyAHRSupdateIMU.h"
#include "MadgwickAHRSupdateIMU.h"
using namespace zen;
int main(int argc, char* argv[])
{
	Calculator::last_matrix_bb << 0, 0, 1,
		0, 1, 0,
		-1, 0, 0;
	std::vector<coodTransformation*> method = { new MahonyAHRSupdateIMU(),new MadgwickAHRSupdateIMU() };

	//scanner��Process����scannerֻ������һ����Processȴ�����ж���
	Scanner* scan = new Scanner();
	Process* P = new ProcessUser(0,1);
	scan->SetProcess(P);
	//�򿪿�ʼ�߳�
	P->Start(scan, method);

    return 0;
}

