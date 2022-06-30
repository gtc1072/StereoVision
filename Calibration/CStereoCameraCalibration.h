#pragma once
#include "Misc.hpp"

class CStereoCameraCalibration
{
public:
	CStereoCameraCalibration();
	virtual ~CStereoCameraCalibration();
	/// <summary>
	/// doStereoCalibrate
	/// </summary>
	/// <param name="world_points"> </param>
	/// <param name="left_image_points"></param>
	/// <param name="right_image_points"></param>
	/// <param name="left_calibration_paras"></param>
	/// <param name="right_calibration_paras"></param>
	/// <param name="image_width"></param>
	/// <param name="image_height"></param>
	/// <param name="rotation_mat"></param>
	/// <param name="translation_vector"></param>
	/// <param name="essential_mat"></param>
	/// <param name="fundamental_mat"></param>
	/// <returns></returns>
	bool doStereoCalibrate(	std::vector<Point3_Type> world_points,
							std::vector<std::vector<Point2_Type>> left_image_points, std::vector<std::vector<Point2_Type>> right_image_points,
							CameraCalibrationParas& left_calibration_paras, CameraCalibrationParas& right_calibration_paras,
							/*int image_width, int image_height, */bool update_intrinsics,
							Matrix_Type& rotation_mat, Matrix_Type& translation_vector, Matrix_Type& essential_mat, Matrix_Type& fundamental_mat);
	bool doStereoCalibrate(	CameraCalibrationParas& left_calibration_paras, CameraCalibrationParas& right_calibration_paras,
							Matrix_Type& rotation_mat, Matrix_Type& translation_vector, Matrix_Type& essential_mat, Matrix_Type& fundamental_mat);
private:
	//void rodriguesTransform(Matrix_Type src, Matrix_Type& dst);
	Matrix_Type guessInitialPose(std::vector<Matrix_Type>& left_camera, std::vector<Matrix_Type>& right_camera);
	void composeStereoPara(CameraCalibrationParas& left_calibration_paras, CameraCalibrationParas& right_calibration_paras, Matrix_Type pose, std::vector<Data_Type>& vec);
	void deComposeStereoPara(std::vector<Data_Type> vec, CameraCalibrationParas& left_calibration_paras, CameraCalibrationParas& right_calibration_paras, Matrix_Type& pose);
private:
	size_t m_kSize, m_extSize;
};

