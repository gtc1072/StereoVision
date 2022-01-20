#pragma once
#include "Utils.hpp"

class CStereoCameraRectification
{
public:
	CStereoCameraRectification();
	virtual ~CStereoCameraRectification();
	bool doStereoRectify(Matrix_Type left_intrinsic, std::vector<Data_Type> left_distort,
		Matrix_Type right_intrinsic, std::vector<Data_Type> right_distort,
		int image_width, int image_height,
		Matrix_Type rotation_mat, Matrix_Type translation_vector,
		Matrix_Type &left_proj, Matrix_Type &right_proj, Matrix_Type &Q);
	bool doImageRectify(int imageWidth, int imageHeight, Data_Type** pXmap_left, Data_Type** pYmap_left,
		Data_Type** pXmap_right, Data_Type** pYmap_right,
		Matrix_Type left_intrinsic, std::vector<Data_Type> left_distort,
		Matrix_Type right_intrinsic, std::vector<Data_Type> right_distort,
		Matrix_Type left_proj, Matrix_Type right_proj);
private:
	void rodriguesTransform(Matrix_Type src, Matrix_Type& dst);
};

