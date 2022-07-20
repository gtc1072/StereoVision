#include "CKNNSearch.h"

CKNNSearch::CKNNSearch()
{
    m_k = 1;
    m_ptrIndex = nullptr;
    m_curRows = 0;
}

CKNNSearch::~CKNNSearch()
{
    if (m_ptrIndex)
    {
        delete m_ptrIndex;
        m_ptrIndex = nullptr;
    }
    if (m_indices.ptr())
    {
        delete[]m_indices.ptr();
    }
    if (m_dists.ptr())
    {
        delete[]m_dists.ptr();
    }
}

void CKNNSearch::setTreeData(flann::Matrix<float>& data)
{
    m_treeData = data;
    flann::IndexParams para;
    para["algorithm"] = flann::FLANN_INDEX_KDTREE_SINGLE;
    para["trees"] = 1;
    m_ptrIndex = new flann::Index<flann::L2<float>>(m_treeData, /*flann::KDTreeIndexParams(1)*/para);
    m_ptrIndex->buildIndex();
}

void CKNNSearch::setQueryData(flann::Matrix<float>& querydata)
{
    m_queryData = querydata; 
    if (m_queryData.rows > m_curRows)
    {
        if (m_indices.ptr())
        {
            delete[]m_indices.ptr();
        }
        if (m_dists.ptr())
        {
            delete[]m_dists.ptr();
        }
        m_indices = flann::Matrix<int>(new int[m_queryData.rows * m_k], m_queryData.rows, m_k);
        m_dists = flann::Matrix<float>(new float[m_queryData.rows * m_k], m_queryData.rows, m_k);
        m_curRows = m_queryData.rows;
    }   
}





void CKNNSearch::setKCount(int count)
{
    if (count > 128) m_k = 128;
    else
        m_k = count;
}

std::pair<flann::Matrix<int>, flann::Matrix<float>> CKNNSearch::getKNeighbour()
{
    m_ptrIndex->knnSearch(m_queryData, m_indices, m_dists, m_k, flann::SearchParams(32));
    return std::make_pair(m_indices, m_dists);
}