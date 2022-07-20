#include "CTxtReader.h"

CTxtReader::CTxtReader()
{
	
}

CTxtReader::~CTxtReader()
{

}

std::vector<float> CTxtReader::split(std::string str)
{
    size_t pos = 0, offset = 0;
    std::string temp;
    std::vector<float> ret;
    while ((pos = str.find_first_of(" ", offset)) != std::string::npos)
    {
        temp = str.substr(offset, pos - offset);
        if (temp.length() > 0)
        {
            ret.push_back(atof(temp.c_str()));
        }
        offset = pos + 1;
    }
    if (offset < str.length())
    {
        temp = str.substr(offset, str.length() - offset);
        ret.push_back(atof(temp.c_str()));
    }
    return ret;
}

flann::Matrix<float> CTxtReader::read(std::string path)
{
    std::ifstream f(path);
    flann::Matrix<float> data;
    if (f.is_open())
    {
        std::string temp;
        std::vector<float> retdata;
        while (std::getline(f, temp))
        {
            std::vector<float> v = split(temp);
            for (size_t i = 0; i < v.size(); ++i)
            {
                retdata.push_back(v[i]);
            }
        }
        size_t rows = retdata.size() / 3;
        float* pTemp = new float[retdata.size()];
        for (int i = 0; i < retdata.size(); ++i)
        {
            pTemp[i] = retdata[i];
        }
        data = flann::Matrix<float>(pTemp, rows, 3);
    }
    f.close();
    return data;
}
