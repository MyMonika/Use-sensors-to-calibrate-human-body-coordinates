//除了一开始main线程被堵塞了一次，其他的时候都是两个线程并行的。
//gyro是陀螺仪，另外一个是加速度计
//一个process对象可以处理两个传感器的数据
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

	//scanner对Process负责，scanner只允许有一个，Process却允许有多条
	Scanner* scan = new Scanner();
	Process* P = new ProcessUser(0,1);
	scan->SetProcess(P);
	//打开开始线程
	P->Start(scan, method);

    return 0;
}

