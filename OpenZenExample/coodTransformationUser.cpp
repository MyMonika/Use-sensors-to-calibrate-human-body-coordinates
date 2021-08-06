#include "coodTransformationUser.h"
#include <coodTransformation.h>
//��������������̬�ں�,���ڶ���һ��,������Ӧ�÷ֿ������Ҳ����������ļ�
//��������ʵ����Ӧ�ý�λ���ں�
//���޸ĺ��һ�ѹ�����ȫ�ָ���
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
		//	std::cout << "�����q1��0 1 2 3 ��" << q1[i][0] << "    " <<
			//	q1[i][1] << "    " << q1[i][2] << "    " << q1[i][3] << std::endl;
		}

		//֮����q
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
