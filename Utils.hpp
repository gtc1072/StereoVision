#pragma once
#include <iostream>
#include <vector>
#include <functional>
#include <random>
#include <algorithm>
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <Eigen/Eigen>
#include <Eigen/Eigenvalues>


#define INVALID_MAP 9.99e+04

template <typename _Tp> class Point_
{
public:
	typedef _Tp value_type;

	Point_()
	{
		x = y = 0;
	}
	Point_(value_type _x, value_type _y)
	{
		this->x = _x;
		this->y = _y;
	}
	Point_(const Point_& pt)
	{
		this->x = pt.x;
		this->y = pt.y;
	}
	Point_& operator = (const Point_& pt)
	{
		if (this == &pt)
			return *this;
		this->x = pt.x;
		this->y = pt.y;
		return *this;
	}

	value_type x;
	value_type y;
};

typedef Point_<float> Point_2f;
typedef Point_<double> Point_2d;

template <typename _Tp> class Point3_
{
public:
	typedef _Tp value_type;

	Point3_()
	{
		x = y = z = 0;
	}
	Point3_(value_type _x, value_type _y, value_type _z)
	{
		this->x = _x;
		this->y = _y;
		this->z = _z;
	}
	Point3_(const Point3_& pt)
	{
		this->x = pt.x;
		this->y = pt.y;
		this->z = pt.z;
	}
	Point3_& operator = (const Point3_& pt)
	{
		if (this == &pt)
			return *this;
		this->x = pt.x;
		this->y = pt.y;
		this->z = pt.z;
		return *this;
	}

	value_type x;
	value_type y;
	value_type z;
};

typedef Point3_<float> Point_3f;
typedef Point3_<double> Point_3d;

#ifdef ENABLE_DOUBLE_TYPE
//#define Point3_Type Point_3d
//#define Point2_Type Point_2d
//#define Data_Type double
//#define Matrix_Type Eigen::MatrixXd
//#define Vetor3_Type Eigen::Vector3d
//#define Quat_Type Eigen::Vector4d
typedef Point_3d Point3_Type;
typedef Point_2d Point2_Type;
typedef double Data_Type;
typedef Eigen::MatrixXd Matrix_Type;
typedef Eigen::Vector3d Vetor3_Type;
typedef Eigen::Vector4d Quat_Type;
#else
typedef Point_3f Point3_Type;
typedef Point_2f Point2_Type;
typedef float Data_Type;
typedef Eigen::MatrixXf Matrix_Type;
typedef Eigen::Vector3f Vetor3_Type;
typedef Eigen::Vector4f Quat_Type;
#endif

typedef struct { Quat_Type qr; Quat_Type qe; } Dualqual_Type;

class CameraCalibrationParas
{
public:
	CameraCalibrationParas()
	{
		intrinsicsMatrix = Matrix_Type::Identity(3, 3);
	}
	CameraCalibrationParas(Matrix_Type in, std::vector<Matrix_Type> ex, std::vector<Data_Type> d)
	{
		intrinsicsMatrix = in;
		extrinsicsMatrixs = ex;
		distortionVec = d;
	}
	CameraCalibrationParas(const CameraCalibrationParas& para)
	{
		this->intrinsicsMatrix = para.intrinsicsMatrix;
		this->extrinsicsMatrixs = para.extrinsicsMatrixs;
		this->distortionVec = para.distortionVec;
	}
	CameraCalibrationParas& operator = (const CameraCalibrationParas& para)
	{
		if (this == &para)
			return *this;
		this->intrinsicsMatrix = para.intrinsicsMatrix;
		this->extrinsicsMatrixs = para.extrinsicsMatrixs;
		this->distortionVec = para.distortionVec;
		return *this;
	}

	Matrix_Type intrinsicsMatrix;
	std::vector<Matrix_Type> extrinsicsMatrixs;
	std::vector<Data_Type> distortionVec;
};

enum HandeyeCalibrateMethod
{
	TSAI_LENZ = 0,
	DANIILIDIS = 1
};
