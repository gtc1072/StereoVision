#pragma once
#include <string>
#include <fstream>
#include "flann\\flann.hpp"
class CTxtReader
{
public:
	CTxtReader();
	virtual ~CTxtReader();
	flann::Matrix<float> read(std::string path);
private:
	std::vector<float> split(std::string str);
};

