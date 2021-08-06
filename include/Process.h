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
		//第一步，坐标系转化（传感器由本地坐标转化到固定坐标）也就是R es
	//转化的方法延长到交给子类的调用，由子类决定父类调用什么办法去转化
		virtual std::vector<float>  Current_fixed_quaternion(std::vector<coodTransformation*> x, std::vector<float> q, float gx, float gy, float gz, float ax, float ay, float az) = 0;
		
		void Start(Scanner* s, std::vector<coodTransformation*> method);

		void Read(Scanner* s, std::vector<coodTransformation*> method);
		//公用的方法实现会在这里实现
		//在得到坐标系转化后，两个处于同一个坐标系下的四元数进行角度计算，得到两个传感器之间的相对角度
		//得到的是两个相对坐标系的旋转矩阵
		//这里暂时使用姿态三进行标定
		//未知数是Rsb
		//输入参数是关节旋转矩阵bb，es转化矩阵
		//现在假设有两个传感器2与7，一个是另一个的父亲，初始时刻这两个的关节旋转矩阵默认是一样的
		Eigen::Matrix3f* EyeInHand(std::queue<Eigen::Matrix3d> parent, std::queue<Eigen::Matrix3d> child);
		void Update(int index);
		inline int getIndex1() {
			return index1;
		}
		inline int getIndex2() {
			return index2;
		}
};