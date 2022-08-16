#include "CPoint2PlaneICP.h"

CPoint2PlaneICP::CPoint2PlaneICP()
{

}
CPoint2PlaneICP::~CPoint2PlaneICP()
{

}
void CPoint2PlaneICP::setReferenceData(flann::Matrix<float> ref)
{
    m_refData = ref;
    m_knn.setKCount(1);
    m_knn.setTreeData(ref);
    m_pNormals = new float[ref.rows * 3];
    calcNormals();
}
void CPoint2PlaneICP::setMoveData(flann::Matrix<float> move)
{
    m_moveData = move;
}
void CPoint2PlaneICP::setEpsilon(float eps)
{
    m_eps = eps;
}
void CPoint2PlaneICP::setMaxIterateCount(int count)
{
    m_count = count;
}
void CPoint2PlaneICP::run()
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
void CPoint2PlaneICP::singleRun(flann::Matrix<float>& move, Eigen::MatrixXd& R, Eigen::MatrixXd& T)
{
    m_knn.setQueryData(move);

    std::pair<flann::Matrix<int>, flann::Matrix<float>> ret = m_knn.getKNeighbour();
    float distthreshold = 5.0;
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

    Eigen::MatrixXd A(ret.first.rows, 6);
    Eigen::MatrixXd AW(6, ret.first.rows);
    Eigen::MatrixXd B(ret.first.rows, 1);

    for (int i = 0; i < ret.first.rows; ++i)
    {
        int idx = ret.first[i][0];
        float nx = m_pNormals[idx * 3];
        float ny = m_pNormals[idx * 3 + 1];
        float nz = m_pNormals[idx * 3 + 2];
        float sx = move[i][0];
        float sy = move[i][1];
        float sz = move[i][2];
        float dx = m_refData[idx][0];
        float dy = m_refData[idx][1];
        float dz = m_refData[idx][2];
        A(i, 0) = sy * nz - sz * ny; A(i, 1) = sz * nx - sx * nz; A(i, 2) = sx * ny - sy * nx; A(i, 3) = nx; A(i, 4) = ny; A(i, 5) = nz;
        B(i, 0) = (dx - sx) * nx + (dy - sy) * ny + (dz - sz) * nz;
    }

    for (int i = 0; i < 6; ++i)
    {
        for (int j = 0; j < ret.first.rows; ++j)
        {
            AW(i, j) = A(j, i) * weight[j];
        }
    }

    Eigen::MatrixXd X = (AW * A).inverse() * AW * B;

    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(X(0, 0), Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(X(1, 0), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(X(2, 0), Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = yawAngle * pitchAngle * rollAngle;
    R = Eigen::MatrixXd(3, 3);
    R << rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2), rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2), rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2);
    T = Eigen::MatrixXd(3, 1);
    T << X(3, 0), X(4, 0), X(5, 0);
}

void CPoint2PlaneICP::calcNormals()
{
    CKNNSearch knn;
    knn.setKCount(20);
    knn.setTreeData(m_refData);
    knn.setQueryData(m_refData);
    std::pair<flann::Matrix<int>, flann::Matrix<float>> ret = knn.getKNeighbour();
    float* pWeight = new float[20];
    for (int m = 0; m < ret.first.rows; ++m)
    {
        Eigen::MatrixXd W = Eigen::MatrixXd::Zero(3, 3);
        for (int k = 0; k < 20; ++k)
        {
            pWeight[k] = 1.0;
        }
        for (int k = 0; k < 20; ++k)
        {
            int idx = ret.first[m][k];
            for (int j = 0; j < 3; ++j)
            {
                for (int i = 0; i < 3; ++i)
                {
                    W(j, i) += pWeight[k] * (m_refData[idx][j] * m_refData[idx][i]) / 20;
                }
            }
        }
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(W);
        m_pNormals[m * 3] = es.eigenvectors()(0, 0);
        m_pNormals[m * 3 + 1] = es.eigenvectors()(1, 0);
        m_pNormals[m * 3 + 2] = es.eigenvectors()(2, 0);
    }
    delete pWeight;
}
