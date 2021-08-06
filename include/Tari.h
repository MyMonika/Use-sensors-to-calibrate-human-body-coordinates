#ifndef _TARI_
#define _TARI_
#include <coodTransformation.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include<math.h>

#include <queue>
#define XEPS					1e-6
typedef Eigen::Vector3f			Geo3d;
typedef Eigen::Matrix3f			GeoMat3;
typedef GeoMat3					RotMat;
typedef Eigen::Isometry3f		GeoTransform;
class Tari
{
	public:
		Tari();
		Geo3d rodrigues2(RotMat matrix);
		RotMat skew(Geo3d v);
		Eigen::MatrixXf svdInverse(Eigen::MatrixXf  A);
		Eigen::Matrix3f sovleAXequalXB(Eigen::Matrix3f vA, Eigen::Matrix3f vB);
};
#endif
