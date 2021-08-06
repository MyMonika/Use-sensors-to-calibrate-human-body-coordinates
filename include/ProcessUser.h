#pragma once
#include "process.h"
#include "coodTransformationUser.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include<math.h>
#include "Scanner.h"
#include "Tari.h"
class ProcessUser :

	public Process, public coodTransformationUser
{
	public:
		//这个类的作用是实现对于位姿的描述
		std::vector<float>  Current_fixed_quaternion(std::vector<coodTransformation*> x, std::vector<float> q, float gx, float gy, float gz, float ax, float ay, float az);

		Eigen::Matrix3f* EyeInHand( std::queue<Eigen::Matrix3d> parent, std::queue<Eigen::Matrix3d> child);
		
		void Update(int index);

		ProcessUser(int i, int j);

		inline int getIndex1() {
			return index1;
		}

		inline int getIndex2() {
			return index2;
		}
};
