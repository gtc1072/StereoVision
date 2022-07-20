#pragma once
#include <string>
#include <fstream>
#include "flann\\flann.hpp"
#include "Eigen\\Core"
#include "Eigen\\Dense"
#include "Eigen\\Eigenvalues"

class CKNNSearch
{
public:
	CKNNSearch();
	virtual ~CKNNSearch();
	void setTreeData(flann::Matrix<float>& data);
	void setQueryData(flann::Matrix<float>& querydata);
	void setKCount(int count);
	std::pair<flann::Matrix<int>, flann::Matrix<float>> getKNeighbour();
private:
	int m_k;
	flann::Matrix<float> m_treeData, m_queryData;
	flann::Matrix<int> m_indices;
	flann::Matrix<float> m_dists;
	flann::Index<flann::L2<float>> *m_ptrIndex;
	int m_curRows;
};