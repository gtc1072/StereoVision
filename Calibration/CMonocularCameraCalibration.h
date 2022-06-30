#pragma once
#include "Misc.hpp"

class CMonocularCameraCalibration
{
public:
	CMonocularCameraCalibration();
	virtual ~CMonocularCameraCalibration();
	bool									doCalibrate(std::vector<Point3_Type> world_points, std::vector<std::vector<Point2_Type>> image_points, int imageWidth, int imageHeight);
	CameraCalibrationParas					getCameraCalibrateParas();
private:
	Matrix_Type								getHomography(int idx);
	std::pair<Matrix_Type, Matrix_Type>		normalizing_frame(int idx);
	void									refineHomography(int idx, Matrix_Type& homoMat);
	Matrix_Type								getIntrinsic();
	Matrix_Type								getIntrinsic2();
	std::vector<Matrix_Type>				getExtrinsics();
	std::vector<Data_Type>					getDistortion();
	void									refineAllPara();
	//void									rodriguesTransform(Matrix_Type src, Matrix_Type& dst);
	void									composeCameraPara(Matrix_Type intrinsic, std::vector<Data_Type> distortion, std::vector<Matrix_Type> extrinsic, std::vector<Data_Type>& out);
	void									decomposeCameraPara(std::vector<Data_Type>& para, Matrix_Type& intrinsic, std::vector<Data_Type>& distortion, std::vector<Matrix_Type>& extrinsic);
	int										getIntrinsicsKOffset();
private:
	std::vector<Point3_Type>				m_worldPoints;
	std::vector<std::vector<Point2_Type>>	m_imagePoints;
	std::vector<Matrix_Type>				m_homographyMatrixs;
	Matrix_Type								m_intrinsicsMatrix;
	std::vector<Matrix_Type>				m_extrinsicsMatrixs;
	std::vector<Data_Type>					m_distortionVec;
	int										m_imageWidth, m_imageHeight;
	std::vector<Data_Type>					m_allParas;
	bool									m_bForceFs2zero;
	bool									m_bKFirst, m_bk2;
};

