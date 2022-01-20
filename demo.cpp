// 3DDemo.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "CMonocularCameraCalibration.h"
#include "CStereoCameraCalibration.h"
#include "CStereoCameraRectification.h"
#include "testdata.h"

void singleCameraTest()
{
    Matrix_Type src(3, 48);
    Matrix_Type dst(3, 48);

    /*Eigen::Matrix<double, 9, 1> p_j = Eigen::Matrix<double, 9, 1>::Zero();
    std::cout << "p_j:\n" << p_j << std::endl;

    Eigen::MatrixXd c_tp = Eigen::MatrixXd::Random(3, 3);
    std::cout << "c_tp:\n" << c_tp << std::endl;
    std::cout << "Diag:\n" << c_tp.diagonal() << std::endl;
    std::cout << "Diag norm:\n" << c_tp.diagonal().norm() << std::endl;
    std::cout << "Diag norm:\n" << c_tp.middleCols(1,1) << std::endl;*/

    /*Eigen::MatrixXd c_tst(1, 3);
    c_tst << 0, 3, 4;
    std::cout << "c_tst:\n" << c_tst << std::endl;
    std::cout << "c_tst norm 1:\n" << c_tst.norm() << std::endl;
    std::cout << "c_tst norm 2:\n" << c_tst.transpose().norm() << std::endl;
    std::cout << "c_tst 2:\n" << c_tst/c_tst.norm() << std::endl;
    Eigen::MatrixXd c_test2 = Eigen::MatrixXd::Random(4, 4);
    std::cout << "c_test2:\n" << c_test2 << std::endl;
    std::cout << "c_test2 diagonal:\n" << c_test2.diagonal() << std::endl;
    c_test2.diagonal() << 1, 2, 3, 4;
    std::cout << "c_test2:\n" << c_test2 << std::endl;*/

    CMonocularCameraCalibration cal;
    std::vector<Point3_Type> real_data;
    for (int j = 0; j < 6; ++j)
    {
        for (int i = 0; i < 8; ++i)
        {
            Point3_Type temp;
            temp.x = i * 25;
            temp.y = j * 25;
            temp.z = 0;
            real_data.push_back(temp);
        }
    }

    /*for (int j = 0; j < 48; ++j)
    {
        src(0, j) = real_data[j].getAt(0);
        src(1, j) = real_data[j].getAt(1);
        src(2, j) = 1.0;
        dst(0, j) = data3[j][0];
        dst(1, j) = data3[j][1];
        dst(2, j) = 1.0;
    }*/

    /* Eigen::Matrix<double, 3, 3> m1_tt;
     m1_tt << 4.83804, 0.0148163, 583.058,
         0.386141, 4.54394, 274.194,
         0.000459522, 0.000168648, 1;


     std::cout << "src:\n" << m1_tt * src << std::endl;
     std::cout << "dst:\n" << dst << std::endl;

     std::cout << "src mean:\n" << src.row(0).mean() << std::endl;
     std::cout << "dst mean:\n" << dst.row(0).mean() << std::endl;

     std::cout << "src mean:\n" << src.row(1).mean() << std::endl;
     std::cout << "dst mean:\n" << dst.row(1).mean() << std::endl;

     Eigen::MatrixXd src_mean(2, 1);
     Eigen::MatrixXd dst_mean(2, 1);
     src_mean << src.row(0).mean(), src.row(1).mean();
     dst_mean << dst.row(0).mean(), dst.row(1).mean();
     Eigen::MatrixXd src2(3,48);
     Eigen::MatrixXd dst2(3,48);
     Eigen::MatrixXd src3(3,48);
     Eigen::MatrixXd dst3(3,48);

     for (int i = 0; i < 48; ++i)
     {
         src2(0, i) = src(0, i) - src_mean(0, 0);
         src2(1, i) = src(1, i) - src_mean(1, 0);
         src2(2, i) = 1.0;
         dst2(0, i) = dst(0, i) - dst_mean(0, 0);
         dst2(1, i) = dst(1, i) - dst_mean(1, 0);
         dst2(2, i) = 1.0;
         src3(0, i) = src2(0, i) * src2(0, i);
         src3(1, i) = src2(1, i) * src2(1, i);
         src3(2, i) = 1.0;
         dst3(0, i) = dst2(0, i) * dst2(0, i);
         dst3(1, i) = dst2(1, i) * dst2(1, i);
         dst3(2, i) = 1.0;
     }

     double ss0, ss1, sd0, sd1;
     ss0 = sqrt(2.0) / sqrt(src3.row(0).mean());
     ss1 = sqrt(2.0) / sqrt(src3.row(1).mean());
     sd0 = sqrt(2.0) / sqrt(dst3.row(0).mean());
     sd1 = sqrt(2.0) / sqrt(dst3.row(1).mean());

     Eigen::MatrixXd ts(3, 3);
     ts << ss0, 0, -ss0 * src_mean(0, 0), 0, ss1, -ss1 * src_mean(1, 0), 0, 0, 1;

     Eigen::MatrixXd td(3, 3);
     td << sd0, 0, -sd0 * dst_mean(0, 0), 0, sd1, -sd1 * dst_mean(1, 0), 0, 0, 1;

     Eigen::MatrixXd src4 = ts * src;
     Eigen::MatrixXd dst4 = td * dst;

     std::cout << "src2:\n" << src4 << std::endl;
     std::cout << "dst2:\n" << dst4 << std::endl;

     Eigen::MatrixXd A = Eigen::MatrixXd::Zero(96, 9);
     for (int i = 0; i < 48; ++i)
     {
         A(i * 2, 0) = -src4(0, i); A(i * 2, 1) = -src4(1, i); A(i * 2, 2) = -1;
         A(i * 2, 3) = 0; A(i * 2, 4) = 0; A(i * 2, 5) = 0;
         A(i * 2, 6) = src4(0, i) * dst4(0, i); A(i * 2, 7) = src4(1, i) * dst4(0, i); A(i * 2, 8) = dst4(0, i);

         A(i * 2 + 1, 0) = 0; A(i * 2 + 1, 1) = 0; A(i * 2 + 1, 2) = 0;
         A(i * 2 + 1, 3) = -src4(0, i); A(i * 2 + 1, 4) = -src4(1, i); A(i * 2 + 1, 5) = -1;
         A(i * 2 + 1, 6) = src4(0, i) * dst4(1, i); A(i * 2 + 1, 7) = src4(1, i) * dst4(1, i); A(i * 2 + 1, 8) = dst4(1, i);
     }

     std::cout << "A:\n" << A << std::endl;

     Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
     Eigen::MatrixXd sv = svd.matrixV();
     std::cout << "sv:\n" << sv << std::endl;

     Eigen::MatrixXd H0 = sv.col(8);
     H0.resize(3, 3);
     Eigen::MatrixXd H = H0.transpose();
     std::cout << "H:\n" << H << std::endl;

     H(0, 0) = H(0, 0) / H(2, 2); H(0, 1) = H(0, 1) / H(2, 2); H(0, 2) = H(0, 2) / H(2, 2);
     H(1, 0) = H(1, 0) / H(2, 2); H(1, 1) = H(1, 1) / H(2, 2); H(1, 2) = H(1, 2) / H(2, 2);
     H(2, 0) = H(2, 0) / H(2, 2); H(2, 1) = H(2, 1) / H(2, 2); H(2, 2) = 1.0;

     std::cout << "H:\n" << H << std::endl;

     Eigen::MatrixXd RH = td.inverse() * H * ts;

     RH(0, 0) = RH(0, 0) / RH(2, 2); RH(0, 1) = RH(0, 1) / RH(2, 2); RH(0, 2) = RH(0, 2) / RH(2, 2);
     RH(1, 0) = RH(1, 0) / RH(2, 2); RH(1, 1) = RH(1, 1) / RH(2, 2); RH(1, 2) = RH(1, 2) / RH(2, 2);
     RH(2, 0) = RH(2, 0) / RH(2, 2); RH(2, 1) = RH(2, 1) / RH(2, 2); RH(2, 2) = 1.0;

     std::cout << "RH:\n" << RH << std::endl;

     Eigen::MatrixXd RR = RH * src;
     std::cout << "RR:\n" << RR.transpose() << std::endl;


     std::vector<Eigen::Matrix<double, 3, 3>> homographyMatrixs;

     Eigen::Matrix<double, 3, 3> m1;
     m1 << 4.38047, -0.312019, 583.053,
         0.0855997, 4.32927, 274.19,
         0.000459522, 0.000168648, 1;
     homographyMatrixs.push_back(m1);
     Eigen::Matrix<double, 3, 3> m2;
     m2<< 3.41471, -2.42332, 688.124,
         0.078611, 0.817269, 415.422,
         9.86047e-05, -0.00251794, 1;
     homographyMatrixs.push_back(m2);
     Eigen::Matrix<double, 3, 3> m3;
     m3<< 2.06505, -1.43795, 541.494,
         -1.04366, 2.22668, 367.791,
         -0.00118488, -0.00147444, 1;
     homographyMatrixs.push_back(m3);
     Eigen::Matrix<double, 3, 3> m4;
     m4<< 6.9233, -1.65553, 932.902,
         2.03772, 2.92115, 201.422,
         0.00273498, -0.00216801, 1;
     homographyMatrixs.push_back(m4);

     Eigen::MatrixXd V(4 * 2, 6);
     for (int i = 0; i < 4; ++i)
     {
         V(i * 2, 0) = homographyMatrixs[i](0, 0) * homographyMatrixs[i](0, 1);
         V(i * 2, 1) = homographyMatrixs[i](1, 0) * homographyMatrixs[i](0, 1) + homographyMatrixs[i](0, 0) * homographyMatrixs[i](1, 1);
         V(i * 2, 2) = homographyMatrixs[i](1, 0) * homographyMatrixs[i](1, 1);
         V(i * 2, 3) = homographyMatrixs[i](2, 0) * homographyMatrixs[i](0, 1) + homographyMatrixs[i](0, 0) * homographyMatrixs[i](2, 1);
         V(i * 2, 4) = homographyMatrixs[i](2, 0) * homographyMatrixs[i](1, 1) + homographyMatrixs[i](1, 0) * homographyMatrixs[i](2, 1);
         V(i * 2, 5) = homographyMatrixs[i](2, 0) * homographyMatrixs[i](2, 1);

         V(i * 2 + 1, 0) = homographyMatrixs[i](0, 0) * homographyMatrixs[i](0, 0) - homographyMatrixs[i](0, 1) * homographyMatrixs[i](0, 1);
         V(i * 2 + 1, 1) = (homographyMatrixs[i](1, 0) * homographyMatrixs[i](0, 0) + homographyMatrixs[i](0, 0) * homographyMatrixs[i](1, 0)) - (homographyMatrixs[i](1, 1) * homographyMatrixs[i](0, 1) + homographyMatrixs[i](0, 1) * homographyMatrixs[i](1, 1));
         V(i * 2 + 1, 2) = homographyMatrixs[i](1, 0) * homographyMatrixs[i](1, 0) - homographyMatrixs[i](1, 1) * homographyMatrixs[i](1, 1);
         V(i * 2 + 1, 3) = (homographyMatrixs[i](2, 0) * homographyMatrixs[i](0, 0) + homographyMatrixs[i](0, 0) * homographyMatrixs[i](2, 0)) - (homographyMatrixs[i](2, 1) * homographyMatrixs[i](0, 1) + homographyMatrixs[i](0, 1) * homographyMatrixs[i](2, 1));
         V(i * 2 + 1, 4) = (homographyMatrixs[i](2, 0) * homographyMatrixs[i](1, 0) + homographyMatrixs[i](1, 0) * homographyMatrixs[i](2, 0)) - (homographyMatrixs[i](2, 1) * homographyMatrixs[i](1, 1) + homographyMatrixs[i](1, 1) * homographyMatrixs[i](2, 1));
         V(i * 2 + 1, 5) = homographyMatrixs[i](2, 0) * homographyMatrixs[i](2, 0) - homographyMatrixs[i](2, 1) * homographyMatrixs[i](2, 1);
     }
     std::cout << "V:\n" << V << std::endl;

     Eigen::JacobiSVD<Eigen::MatrixXd> svd1(V.transpose() * V, Eigen::ComputeFullU | Eigen::ComputeFullV);
     Eigen::MatrixXd sv1 = svd1.matrixV();
     std::cout << "sv\n" << sv1 << std::endl;

     double B11 = sv1(0, 5); double B12 = sv1(1, 5); double B22 = sv1(2, 5);
     double B13 = sv1(3, 5); double B23 = sv1(4, 5); double B33 = sv1(5, 5);

     std::cout << B11 << " " << B12 << " " << B22 << " " << B13 << " " << B23 << " " << B33 << std::endl;

     Eigen::MatrixXd test(3, 3);
     test << B11, B12, B13, B12, B22, B23, B13, B23, B33;
     std::cout << "test:\n" << test << std::endl;

     std::cout << test.determinant() << std::endl;

     Eigen::MatrixXd ml = test.llt().matrixL();

     Eigen::MatrixXd testml = ml * ml.transpose();

     std::cout << "ml:\n" << ml << std::endl;
     std::cout << "testml:\n" << testml << std::endl;

     double v0 = (B12 * B13 - B11 * B23) / (B11 * B22 - B12 * B12);
     std::cout << "V0 " << v0 << std::endl;
     double lamda = B33 - (B13 * B13 + v0 * (B12 * B13 - B11 * B23)) / B11;
     std::cout << "lamda " << lamda << std::endl;
     double alpha = sqrt(lamda / B11);
     std::cout << "alpha " << alpha << std::endl;
     double belta = sqrt(lamda * B11 / (B11 * B22 - B12 * B12));
     std::cout << "belta " << belta << std::endl;
     double gamma = -B12 * alpha * alpha * belta / lamda;
     std::cout << "gamma " << gamma << std::endl;
     double mu0 = gamma * v0 / alpha - B13 * alpha * alpha / lamda;
     std::cout << "mu0 " << mu0 << std::endl;*/

    std::vector<std::vector<Point2_Type>> image_points;
    std::vector<Point2_Type> image_point;
    for (int i = 0; i < 48; ++i)
    {
        Point2_Type temp;
        temp.x = data0[i][0];
        temp.y = data0[i][1];
        image_point.push_back(temp);
    }
    image_points.push_back(image_point);
    image_point.clear();

    for (int i = 0; i < 48; ++i)
    {
        Point2_Type temp;
        temp.x = data1[i][0];
        temp.y = data1[i][1];
        image_point.push_back(temp);
    }
    image_points.push_back(image_point);
    image_point.clear();

    for (int i = 0; i < 48; ++i)
    {
        Point2_Type temp;
        temp.x = data2[i][0];
        temp.y = data2[i][1];
        image_point.push_back(temp);
    }
    image_points.push_back(image_point);
    image_point.clear();

    for (int i = 0; i < 48; ++i)
    {
        Point2_Type temp;
        temp.x = data3[i][0];
        temp.y = data3[i][1];
        image_point.push_back(temp);
    }
    image_points.push_back(image_point);
    image_point.clear();

    cal.doCalibrate(real_data, image_points, 1920, 1080);
}

void stereoCameraTest()
{
    Data_Type corner_points[][3] = { 
                                     {  0,   0,   0},
                                     { 25,   0,   0},
                                     { 50,   0,   0},
                                     { 75,   0,   0},
                                     {100,   0,   0},
                                     {125,   0,   0},
                                     {150,   0,   0},
                                     {175,   0,   0},
                                     {200,   0,   0},
                                     {  0,  25,   0},
                                     { 25,  25,   0},
                                     { 50,  25,   0},
                                     { 75,  25,   0},
                                     {100,  25,   0},
                                     {125,  25,   0},
                                     {150,  25,   0},
                                     {175,  25,   0},
                                     {200,  25,   0},
                                     {  0,  50,   0},
                                     { 25,  50,   0},
                                     { 50,  50,   0},
                                     { 75,  50,   0},
                                     {100,  50,   0},
                                     {125,  50,   0},
                                     {150,  50,   0},
                                     {175,  50,   0},
                                     {200,  50,   0},
                                     {  0,  75,   0},
                                     { 25,  75,   0},
                                     { 50,  75,   0},
                                     { 75,  75,   0},
                                     {100,  75,   0},
                                     {125,  75,   0},
                                     {150,  75,   0},
                                     {175,  75,   0},
                                     {200,  75,   0},
                                     {  0, 100,   0},
                                     { 25, 100,   0},
                                     { 50, 100,   0},
                                     { 75, 100,   0},
                                     {100, 100,   0},
                                     {125, 100,   0},
                                     {150, 100,   0},
                                     {175, 100,   0},
                                     {200, 100,   0},
                                     {  0, 125,   0},
                                     { 25, 125,   0},
                                     { 50, 125,   0},
                                     { 75, 125,   0},
                                     {100, 125,   0},
                                     {125, 125,   0},
                                     {150, 125,   0},
                                     {175, 125,   0},
                                     {200, 125,   0}, 
                                    };

    std::vector<Point3_Type> world_points;
    std::vector<std::vector<Point2_Type>> left_image_points;
    std::vector<std::vector<Point2_Type>> right_image_points;
    for (int i = 0; i < 54; ++i)
    {
        world_points.push_back(Point3_Type(corner_points[i][0], corner_points[i][1], corner_points[i][2]));
    }
    std::vector<Point2_Type> img_point;
    for (int i = 0; i < 54; ++i)
    {
        img_point.push_back(Point2_Type(left_data1[i][0], left_data1[i][1]));
    }
    left_image_points.push_back(img_point);
    img_point.clear();
    for (int i = 0; i < 54; ++i)
    {
        img_point.push_back(Point2_Type(left_data2[i][0], left_data2[i][1]));
    }
    left_image_points.push_back(img_point);
    img_point.clear();
    for (int i = 0; i < 54; ++i)
    {
        img_point.push_back(Point2_Type(left_data3[i][0], left_data3[i][1]));
    }
    left_image_points.push_back(img_point);
    img_point.clear();
    for (int i = 0; i < 54; ++i)
    {
        img_point.push_back(Point2_Type(left_data4[i][0], left_data4[i][1]));
    }
    left_image_points.push_back(img_point);
    img_point.clear();
    for (int i = 0; i < 54; ++i)
    {
        img_point.push_back(Point2_Type(right_data1[i][0], right_data1[i][1]));
    }
    right_image_points.push_back(img_point);
    img_point.clear();
    for (int i = 0; i < 54; ++i)
    {
        img_point.push_back(Point2_Type(right_data2[i][0], right_data2[i][1]));
    }
    right_image_points.push_back(img_point);
    img_point.clear();
    for (int i = 0; i < 54; ++i)
    {
        img_point.push_back(Point2_Type(right_data3[i][0], right_data3[i][1]));
    }
    right_image_points.push_back(img_point);
    img_point.clear();
    for (int i = 0; i < 54; ++i)
    {
        img_point.push_back(Point2_Type(right_data4[i][0], right_data4[i][1]));
    }
    right_image_points.push_back(img_point);
    img_point.clear();

    CMonocularCameraCalibration left_cal, right_cal;
    left_cal.doCalibrate(world_points, left_image_points, 640, 480);
    right_cal.doCalibrate(world_points, right_image_points, 640, 480);
    CameraCalibrationParas left_paras = left_cal.getCameraCalibrateParas();
    CameraCalibrationParas right_paras = right_cal.getCameraCalibrateParas();

    CStereoCameraCalibration stereo_cal;
    Matrix_Type rot_mat, trans_mat, essential_mat, fundamental_mat;
    stereo_cal.doStereoCalibrate(world_points, left_image_points, right_image_points, left_paras, right_paras, true, rot_mat, trans_mat, essential_mat, fundamental_mat);
    std::cout << "rot_mat:\n" <<rot_mat << std::endl;
    std::cout << "trans_mat:\n" << trans_mat << std::endl;
    std::cout << "essential_mat:\n" << essential_mat << std::endl;
    std::cout << "fundamental_mat:\n" << fundamental_mat << std::endl;
    CStereoCameraRectification rectify;
    Matrix_Type left_proj, right_proj, Q;
    rectify.doStereoRectify(left_paras.intrinsicsMatrix, left_paras.distortionVec, right_paras.intrinsicsMatrix, right_paras.distortionVec,
        640, 480, rot_mat, trans_mat, left_proj, right_proj, Q);
    std::cout << "left_proj:\n" << left_proj << std::endl;
    std::cout << "right_proj:\n" << right_proj << std::endl;
    std::cout << "Q:\n" << Q << std::endl;
}

void testEigen()
{
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX顺序，即yaw-pitch-roll顺序
    std::cout.precision(3);
    std::cout << "yaw pitch roll = " << euler_angles.transpose() << std::endl;
    Eigen::AngleAxisd rotation_vector(3.1415926 / 4, Eigen::Vector3d(0, 0, 1));     //沿 Z 轴旋转 45 度
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();                // 虽然称为3d，实质上是4＊4的矩阵
    T.rotate(rotation_vector);                                     // 按照rotation_vector进行旋转
    T.pretranslate(Eigen::Vector3d(1, 3, 4));                     // 把平移向量设成(1,3,4)
    std::cout << "Transform matrix = \n" << T.matrix() << std::endl;
}

int main()
{
    //testEigen();
    //singleCameraTest();
    stereoCameraTest();
    std::cout << "Hello World!\n";
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
