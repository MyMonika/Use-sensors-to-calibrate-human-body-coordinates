#ifndef _CALCULATOR_
#define _CALCULATOR_
#include<math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include <queue>
#include "Tari.h"
class Calculator
{
	public :
		Tari tari = Tari();
		static Eigen::Matrix3d last_matrix_bb;
		static float* Quaterniond2Euler(std::vector<float> q);
		static float  invSqrt(float x);
		static void RotationToEular(Eigen::Matrix3f mat);
		static Eigen::Matrix3d QuaternionToRotation(std::vector<float> q);
		static std::queue<Eigen::Matrix3d> IntrinsicAverage(std::queue<Eigen::Matrix3d> parent);
		static Eigen::Matrix3d RotationTwoCood(Eigen::Matrix3d matrix_bb_parents, Eigen::Matrix3d matrix_bb_child);
		static Eigen::Matrix3d SensorToEnvir(Eigen::Matrix3d Sensor);
		static Eigen::Matrix3f* EyeInHand(std::queue<Eigen::Matrix3d> parent, std::queue<Eigen::Matrix3d> child, Calculator* ca);
};
#endif

