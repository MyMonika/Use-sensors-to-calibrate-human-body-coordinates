#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include<math.h>
#include <queue>
#include <vector>
#include"Observer.h"
#include <Calculator.h>
class Scanner;
class Process :public Observer {

	public:
		 int index1;
		 int index2;
		 Calculator* ca = new Calculator();
		//��һ��������ϵת�����������ɱ�������ת�����̶����꣩Ҳ����R es
	//ת���ķ����ӳ�����������ĵ��ã�����������������ʲô�취ȥת��
		virtual std::vector<float>  Current_fixed_quaternion(std::vector<coodTransformation*> x, std::vector<float> q, float gx, float gy, float gz, float ax, float ay, float az) = 0;
		
		void Start(Scanner* s, std::vector<coodTransformation*> method);

		void Read(Scanner* s, std::vector<coodTransformation*> method);
		//���õķ���ʵ�ֻ�������ʵ��
		//�ڵõ�����ϵת������������ͬһ������ϵ�µ���Ԫ�����нǶȼ��㣬�õ�����������֮�����ԽǶ�
		//�õ����������������ϵ����ת����
		//������ʱʹ����̬�����б궨
		//δ֪����Rsb
		//��������ǹؽ���ת����bb��esת������
		//���ڼ���������������2��7��һ������һ���ĸ��ף���ʼʱ���������Ĺؽ���ת����Ĭ����һ����
		Eigen::Matrix3f* EyeInHand(std::queue<Eigen::Matrix3d> parent, std::queue<Eigen::Matrix3d> child);
		void Update(int index);
		inline int getIndex1() {
			return index1;
		}
		inline int getIndex2() {
			return index2;
		}
};