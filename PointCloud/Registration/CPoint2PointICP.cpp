#include "CPoint2PointICP.h"

CPoint2PointICP::CPoint2PointICP()
{
    m_curRows = 0;
    m_pSelRef = nullptr;
    m_pSelMove = nullptr;
    m_pWeight = nullptr;
}

CPoint2PointICP::~CPoint2PointICP()
{

}

void CPoint2PointICP::setReferenceData(flann::Matrix<float> ref)
{
    m_refData = ref;
    m_knn.setKCount(1);
    m_knn.setTreeData(ref);
}

void CPoint2PointICP::setMoveData(flann::Matrix<float> move)
{
    m_moveData = move;
    if (move.rows > m_curRows)
    {
        if (m_pSelRef)
        {
            delete m_pSelRef;
            delete m_pSelMove;
            delete m_pWeight;
        }
        m_pSelRef = new float[move.rows * 3];
        m_pSelMove = new float[move.rows * 3];
        m_pWeight = new float[move.rows * 3];
    }
}

void CPoint2PointICP::setEpsilon(float eps)
{
    m_eps = eps;
}

void CPoint2PointICP::setMaxIterateCount(int count)
{
    m_count = count;
}

void CPoint2PointICP::run()
{
    Eigen::MatrixXd Trans = Eigen::MatrixXd::Identity(4, 4);
    Eigen::MatrixXd R, T;
    singleRun(m_moveData, R, T);
    Eigen::MatrixXd tempTrans = Eigen::MatrixXd::Identity(4, 4);
    tempTrans.block(0, 0, 3, 3) = R;
    tempTrans.block(0, 3, 3, 1) = T;
    Trans = tempTrans * Trans;
    for (int k = 1; k < m_count; ++k)
    {
        for (int j = 0; j < m_moveData.rows; ++j)
        {
            float x = R(0, 0) * m_moveData[j][0] + R(0, 1) * m_moveData[j][1] + R(0, 2) * m_moveData[j][2] + T(0, 0);
            float y = R(1, 0) * m_moveData[j][0] + R(1, 1) * m_moveData[j][1] + R(1, 2) * m_moveData[j][2] + T(1, 0);
            float z = R(2, 0) * m_moveData[j][0] + R(2, 1) * m_moveData[j][1] + R(2, 2) * m_moveData[j][2] + T(2, 0);
            m_moveData[j][0] = x;
            m_moveData[j][1] = y;
            m_moveData[j][2] = z;
        }
        singleRun(m_moveData, R, T);
        tempTrans = Eigen::MatrixXd::Identity(4, 4);
        tempTrans.block(0, 0, 3, 3) = R;
        tempTrans.block(0, 3, 3, 1) = T;
        Trans = tempTrans * Trans;

    }
    std::cout << "Current Transformation:" << std::endl << Trans << std::endl;
    Eigen::Matrix3d rotation_matrix = Trans.block(0, 0, 3, 3);
    Eigen::Vector3d ea = rotation_matrix.eulerAngles(0, 1, 2) / 3.1415926 * 180.0;
    std::cout << "Euler Angle X Y Z: \n" << ea << std::endl;
}

void CPoint2PointICP::singleRun(flann::Matrix<float>& move, Eigen::MatrixXd& R, Eigen::MatrixXd& T)
{
    m_knn.setQueryData(move);

    std::pair<flann::Matrix<int>, flann::Matrix<float>> ret = m_knn.getKNeighbour();
    int count = 0;
    float sumweight = 0.0;
    float distthreshold = 15.0;
    float refux = 0, refuy = 0, refuz = 0;
    float moveux = 0, moveuy = 0, moveuz = 0;
    std::vector<float> weight(ret.first.rows, 0.0f);
    for (int i = 0; i < ret.second.rows; ++i)
    {
        if (ret.second[i][0] <= distthreshold)
        {
            float sl = ret.second[i][0] / distthreshold;
            weight[i] = (1.0 - sl * sl) * (1.0 - sl * sl);
        }
        else
            weight[i] = 0;
    }
    for (int i = 0; i < ret.first.rows; ++i)
    {
        if (ret.second[i][0] < distthreshold)
        {
            m_pSelMove[count * 3] = move[i][0];
            m_pSelMove[count * 3 + 1] = move[i][1];
            m_pSelMove[count * 3 + 2] = move[i][2];
            moveux += m_pSelMove[count * 3] * weight[i];
            moveuy += m_pSelMove[count * 3 + 1] * weight[i];
            moveuz += m_pSelMove[count * 3 + 2] * weight[i];
            m_pSelRef[count * 3] = m_refData[ret.first[i][0]][0];
            m_pSelRef[count * 3 + 1] = m_refData[ret.first[i][0]][1];
            m_pSelRef[count * 3 + 2] = m_refData[ret.first[i][0]][2];
            refux += m_pSelRef[count * 3] * weight[i];
            refuy += m_pSelRef[count * 3 + 1] * weight[i];
            refuz += m_pSelRef[count * 3 + 2] * weight[i];
            m_pWeight[count] = weight[i];
            sumweight += weight[i];
            count++;
        }
    }
    refux /= sumweight; refuy /= sumweight; refuz /= sumweight;
    moveux /= sumweight; moveuy /= sumweight; moveuz /= sumweight;
    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(3, 3);
    for (int i = 0; i < count; ++i)
    {
        m_pSelRef[i * 3] = m_pSelRef[i * 3] - refux;
        m_pSelRef[i * 3 + 1] = m_pSelRef[i * 3 + 1] - refuy;
        m_pSelRef[i * 3 + 2] = m_pSelRef[i * 3 + 2] - refuz;
        m_pSelMove[i * 3] = m_pSelMove[i * 3] - moveux;
        m_pSelMove[i * 3 + 1] = m_pSelMove[i * 3 + 1] - moveuy;
        m_pSelMove[i * 3 + 2] = m_pSelMove[i * 3 + 2] - moveuz;
    }

    for (int k = 0; k < count; ++k)
    {
        for (int j = 0; j < 3; ++j)
        {
            for (int i = 0; i < 3; ++i)
            {
                W(j, i) += m_pWeight[k] * (m_pSelMove[k * 3 + j] * m_pSelRef[k * 3 + i]) / count;
            }
        }
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::MatrixXd S = Eigen::MatrixXd::Identity(3, 3);
    S(2, 2) = (V * U.transpose()).determinant() < 0 ? -1 : 1;
    R = V * S * U.transpose();
    Eigen::MatrixXd A(3, 1), B(3, 1);
    A << refux, refuy, refuz;
    B << moveux, moveuy, moveuz;
    T = A - R * B;
}
