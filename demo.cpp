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

int main()
{
    //singleCameraTest();
    stereoCameraTest();
}
