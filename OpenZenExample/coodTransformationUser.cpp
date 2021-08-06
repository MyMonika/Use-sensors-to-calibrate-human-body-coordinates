#include "coodTransformationUser.h"
#include <coodTransformation.h>
//这个类的作用是姿态融合,现在多了一个,理论上应该分开，但我不会重命名文件
//这个类的真实名称应该叫位姿融合
//在修改后，我会把功能完全分隔开
coodTransformationUser::coodTransformationUser() {

}
void coodTransformationUser:: SetCoodTransformation(coodTransformation* x) {
	ob = x;
}
std::vector<float> coodTransformationUser::PostureFusion(std::vector<float> q, float gx, float gy, float gz, float ax, float ay, float az) {
	q=ob->PostureFusion(q, gx, gy, gz, ax, ay, az);
	return q;
}

std::vector<float> coodTransformationUser::Current_fixed_quaternion(std::vector<coodTransformation*> x, std::vector<float> q, float gx, float gy, float gz, float ax, float ay, float az) {
	int len = x.size();
	if (len != 0) {
		std::vector<std::vector<float>> q1(len,q);
		for (int i=0;i<len ;i++)
		{
			this->SetCoodTransformation(x[i]);
			q1[i]=this->PostureFusion(q1[i], gx, gy, gz, ax, ay, az);
		//	std::cout << "结果是q1的0 1 2 3 是" << q1[i][0] << "    " <<
			//	q1[i][1] << "    " << q1[i][2] << "    " << q1[i][3] << std::endl;
		}

		//之后算q
		q = { 0.0f,0.0f,0.0f,0.0f };
		for (int i = 0; i < len; i++) {
			q[0] += q1[i][0] / len;
			q[1] += q1[i][1] / len;
			q[2] += q1[i][2] / len;
			q[3] += q1[i][3] / len;

		}
	}
	return q;


}
