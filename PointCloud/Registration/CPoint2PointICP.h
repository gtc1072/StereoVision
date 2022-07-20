#pragma once
#include "flann\\flann.hpp"
#include "Eigen\\Core"
#include "Eigen\\Dense"
#include "Eigen\\Eigenvalues"
#include "CKNNSearch.h"

class CPoint2PointICP
{
public:
	CPoint2PointICP();
	~CPoint2PointICP();
	void setReferenceData(flann::Matrix<float> ref);
	void setMoveData(flann::Matrix<float> move);
	void setEpsilon(float eps);
	void setMaxIterateCount(int count);
	void run();
private:
	void singleRun(flann::Matrix<float>& move, Eigen::MatrixXd& R, Eigen::MatrixXd& T);
private:
	flann::Matrix<float> m_refData, m_moveData;
	float m_eps;
	int m_count;
	CKNNSearch m_knn;
	float* m_pSelRef, * m_pSelMove, * m_pWeight;
	int m_curRows;
};

