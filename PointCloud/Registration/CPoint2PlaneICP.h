#pragma once
#include "flann\\flann.hpp"
#include "Eigen\\Core"
#include "Eigen\\Dense"
#include "Eigen\\Eigenvalues"
#include "CKNNSearch.h"

class CPoint2PlaneICP
{
public:
	CPoint2PlaneICP();
	~CPoint2PlaneICP();
	void setReferenceData(flann::Matrix<float> ref);
	void setMoveData(flann::Matrix<float> move);
	void setEpsilon(float eps);
	void setMaxIterateCount(int count);
	void run();
private:
	void singleRun(flann::Matrix<float>& move, Eigen::MatrixXd& R, Eigen::MatrixXd& T);
	void calcNormals();
private:
	flann::Matrix<float> m_refData, m_moveData;
	float m_eps;
	int m_count;
	CKNNSearch m_knn;
	float* m_pNormals;
};

