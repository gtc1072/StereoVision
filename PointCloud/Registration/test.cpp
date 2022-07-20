#include <iostream>
#include <string>
#include <fstream>
#include <chrono>
#include "Eigen\\Core"
#include "Eigen\\Geometry"
#include "flann\\flann.hpp"

#include "CTxtReader.h"
#include "CPoint2PointICP.h"
#include "CPoint2PlaneICP.h"


int main()
{
	CTxtReader reader;
    flann::Matrix<float> dataset = reader.read("17.txt");
    flann::Matrix<float> query = reader.read("15.txt");
    CPoint2PlaneICP icp;
    icp.setReferenceData(dataset);
    auto t1 = std::chrono::steady_clock::now();
    icp.setMoveData(query);
    icp.setMaxIterateCount(5);
    icp.run();
    auto t2 = std::chrono::steady_clock::now();
    double dr_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
    std::cout << "searchint spend " << dr_ms << " ms" << std::endl;
	return 0;
}