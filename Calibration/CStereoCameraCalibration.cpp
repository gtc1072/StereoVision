#include "CStereoCameraCalibration.h"

CStereoCameraCalibration::CStereoCameraCalibration()
{

}

CStereoCameraCalibration::~CStereoCameraCalibration()
{

}

bool CStereoCameraCalibration::doStereoCalibrate(	std::vector<Point3_Type> world_points,
	std::vector<std::vector<Point2_Type>> left_image_points, std::vector<std::vector<Point2_Type>> right_image_points,
	CameraCalibrationParas& left_calibration_paras, CameraCalibrationParas& right_calibration_paras,
	/*int image_width, int image_height, */bool update_intrinsics,
	Matrix_Type& rotation_mat, Matrix_Type& translation_vector, Matrix_Type& essential_mat, Matrix_Type& fundamental_mat)
{
	bool ret = false;
	////////////////////////////////////////////////////
	if (world_points.size() < 4 || left_image_points.size() < 3 || right_image_points.size() < 3 || left_image_points.size() != right_image_points.size()) return ret;
	if (world_points.size() != left_image_points[0].size() || world_points.size() != right_image_points[0].size()) return ret;
	if (left_calibration_paras.extrinsicsMatrixs.size() != left_image_points.size() || right_calibration_paras.extrinsicsMatrixs.size() != right_image_points.size()) return ret;
	////////////////////////////////////////////////////lamda
	std::function<Matrix_Type(CameraCalibrationParas, int, int)> value = [&](CameraCalibrationParas para, int frameId, int pointId)->Matrix_Type {
		Matrix_Type real_c(4, 1);
		Matrix_Type ret(2, 1);
		real_c(0, 0) = world_points[pointId].x; real_c(1, 0) = world_points[pointId].y; real_c(2, 0) = 0; real_c(3, 0) = 1;
		Matrix_Type cam_c = para.extrinsicsMatrixs[frameId] * real_c;
		Data_Type nx = cam_c(0, 0) / cam_c(2, 0);
		Data_Type ny = cam_c(1, 0) / cam_c(2, 0);
		Data_Type r = nx * nx + ny * ny;
		if (m_kSize == 2)
		{
			Data_Type xe = nx * (1.0 + para.distortionVec[0] * r + para.distortionVec[1] * r * r);
			Data_Type ye = ny * (1.0 + para.distortionVec[0] * r + para.distortionVec[1] * r * r);

			ret(0, 0) = para.intrinsicsMatrix(0, 0) * xe + para.intrinsicsMatrix(0, 1) * ye + para.intrinsicsMatrix(0, 2);
			ret(1, 0) = para.intrinsicsMatrix(1, 1) * ye + para.intrinsicsMatrix(1, 2);
		}
		else
		{
			Data_Type xe = nx * (1.0 + para.distortionVec[0] * r + para.distortionVec[1] * r * r + para.distortionVec[4] * r * r * r) + 2.0 * para.distortionVec[2] * nx * ny + para.distortionVec[3] * (r * r + 2.0 * nx * nx);
			Data_Type ye = ny * (1.0 + para.distortionVec[0] * r + para.distortionVec[1] * r * r + para.distortionVec[4] * r * r * r) + para.distortionVec[2] * (r * r + 2.0 * ny * ny) + 2.0 * para.distortionVec[3] * nx * ny;

			ret(0, 0) = para.intrinsicsMatrix(0, 0) * xe + para.intrinsicsMatrix(0, 1) * ye + para.intrinsicsMatrix(0, 2);
			ret(1, 0) = para.intrinsicsMatrix(1, 1) * ye + para.intrinsicsMatrix(1, 2);
		}
		return ret;
	};
	std::function<Matrix_Type(std::vector<Data_Type>)> residual = [&](std::vector<Data_Type> vec)->Matrix_Type {
		size_t count = left_calibration_paras.extrinsicsMatrixs.size() * world_points.size() * 4;
		CameraCalibrationParas cur_left_calibration_paras;
		CameraCalibrationParas cur_right_calibration_paras;
		Matrix_Type cur_pose;
		deComposeStereoPara(vec, cur_left_calibration_paras, cur_right_calibration_paras, cur_pose);
		Matrix_Type ret = Matrix_Type::Zero(count, 1);
		int loop = 0;
		for (size_t i = 0; i < left_image_points.size(); ++i)
		{
			for (size_t j = 0; j < left_image_points[i].size(); ++j)
			{
				Matrix_Type cur_value = value(cur_left_calibration_paras, i, j);
				ret(loop, 0) = cur_value(0, 0) - left_image_points[i][j].x;
				ret(loop + 1, 0) = cur_value(1, 0) - left_image_points[i][j].y;
				loop += 2;
			}
		}
		for (size_t i = 0; i < right_image_points.size(); ++i)
		{
			for (size_t j = 0; j < right_image_points[i].size(); ++j)
			{
				Matrix_Type cur_value = value(cur_right_calibration_paras, i, j);
				ret(loop, 0) = cur_value(0, 0) - right_image_points[i][j].x;
				ret(loop + 1, 0) = cur_value(1, 0) - right_image_points[i][j].y;
				loop += 2;
			}
		}
		return ret;
	};
	std::function<Matrix_Type(std::vector<Data_Type>, bool)> jacobian = [&](std::vector<Data_Type> vec, bool update_intrinsics)->Matrix_Type {
		Data_Type eps;
		if (typeid(Data_Type) == typeid(double))
			eps = 1.0e-6;
		else
			eps = 0.00005;
		CameraCalibrationParas upper_cur_left_calibration_paras;
		CameraCalibrationParas upper_cur_right_calibration_paras;
		Matrix_Type upper_cur_pose;
		CameraCalibrationParas lower_cur_left_calibration_paras;
		CameraCalibrationParas lower_cur_right_calibration_paras;
		Matrix_Type lower_cur_pose;
		//deComposeStereoPara(vec, cur_left_calibration_paras, cur_right_calibration_paras, cur_pose);
		size_t count = left_calibration_paras.extrinsicsMatrixs.size() * world_points.size() * 4;
		if (update_intrinsics)
		{
			int cols = 8 + m_kSize * 2 + left_image_points.size() * 6 + 6;
			Matrix_Type ret = Matrix_Type::Zero(count, cols);
			std::vector<Data_Type> vec_upper;
			std::vector<Data_Type> vec_lower;
			for (int k = 0; k < cols; ++k)
			{
				vec_upper = vec;
				vec_lower = vec;
				vec_upper[k] += eps;
				vec_lower[k] -= eps;
				deComposeStereoPara(vec_upper, upper_cur_left_calibration_paras, upper_cur_right_calibration_paras, upper_cur_pose);
				deComposeStereoPara(vec_lower, lower_cur_left_calibration_paras, lower_cur_right_calibration_paras, lower_cur_pose);
				int loop = 0;
				if (k < 8 + m_kSize * 2)
				{
					
					if (k < 4 + m_kSize)
					{
						loop = 0;
						for (size_t i = 0; i < left_image_points.size(); ++i)
						{
							for (size_t j = 0; j < left_image_points[i].size(); ++j)
							{
								Matrix_Type upper_value = value(upper_cur_left_calibration_paras, i, j);
								Matrix_Type lower_value = value(lower_cur_left_calibration_paras, i, j);
								ret(loop, k) = (upper_value(0, 0) - lower_value(0, 0)) / (2.0 * eps);
								ret(loop + 1, k) = (upper_value(1, 0) - lower_value(1, 0)) / (2.0 * eps);
								loop += 2;
							}
						}
					}
					else
					{
						loop = left_image_points.size() * left_image_points[0].size() * 2;
						for (size_t i = 0; i < right_image_points.size(); ++i)
						{
							for (size_t j = 0; j < right_image_points[i].size(); ++j)
							{
								Matrix_Type upper_value = value(upper_cur_right_calibration_paras, i, j);
								Matrix_Type lower_value = value(lower_cur_right_calibration_paras, i, j);
								ret(loop, k) = (upper_value(0, 0) - lower_value(0, 0)) / (2.0 * eps);
								ret(loop + 1, k) = (upper_value(1, 0) - lower_value(1, 0)) / (2.0 * eps);
								loop += 2;
							}
						}
					}
				}
				else if(k < cols - 6)
				{
					loop = 0;
					for (size_t i = 0; i < left_image_points.size(); ++i)
					{
						int range_left = 8 + m_kSize * 2 + i * 6;
						int range_right = 8 + m_kSize * 2 + (i + 1) * 6;
						if (k >= range_left && k < range_right)
						{
							for (size_t j = 0; j < left_image_points[i].size(); ++j)
							{
								Matrix_Type upper_value = value(upper_cur_left_calibration_paras, i, j);
								Matrix_Type lower_value = value(lower_cur_left_calibration_paras, i, j);
								ret(loop, k) = (upper_value(0, 0) - lower_value(0, 0)) / (2.0 * eps);
								ret(loop + 1, k) = (upper_value(1, 0) - lower_value(1, 0)) / (2.0 * eps);
								loop += 2;
							}
						}
						else
						{
							loop += left_image_points[i].size() * 2;
						}
					}
				}
				else
				{
					loop = left_image_points.size() * left_image_points[0].size() * 2;
					for (size_t i = 0; i < right_image_points.size(); ++i)
					{
						for (size_t j = 0; j < right_image_points[i].size(); ++j)
						{
							Matrix_Type upper_value = value(upper_cur_right_calibration_paras, i, j);
							//vec_lower[k] -= eps;
							Matrix_Type lower_value = value(lower_cur_right_calibration_paras, i, j);
							ret(loop, k) = (upper_value(0, 0) - lower_value(0, 0)) / (2.0 * eps);
							ret(loop + 1, k) = (upper_value(1, 0) - lower_value(1, 0)) / (2.0 * eps);
							loop += 2;
						}
					}
				}
			}
			return ret;
		}
		else
		{
			int cols = left_image_points.size() * 6 + 6;
			int offset = 8 + m_kSize * 2;
			Matrix_Type ret = Matrix_Type::Zero(count, cols);
			std::vector<Data_Type> vec_upper;
			std::vector<Data_Type> vec_lower;
			for (int k = 0; k < cols; ++k)
			{
				vec_upper = vec;
				vec_lower = vec;
				vec_upper[k + offset] += eps;
				vec_lower[k + offset] -= eps;
				deComposeStereoPara(vec_upper, upper_cur_left_calibration_paras, upper_cur_right_calibration_paras, upper_cur_pose);
				deComposeStereoPara(vec_lower, lower_cur_left_calibration_paras, lower_cur_right_calibration_paras, lower_cur_pose);
				int loop = 0;
				if (k < cols - 6)
				{
					for (size_t i = 0; i < left_image_points.size(); ++i)
					{
						int range_left = i * 6;
						int range_right = (i + 1) * 6;
						if (k >= range_left && k < range_right)
						{
							for (size_t j = 0; j < left_image_points[i].size(); ++j)
							{
								Matrix_Type upper_value = value(upper_cur_left_calibration_paras, i, j);
								//vec_lower[k] -= eps;
								Matrix_Type lower_value = value(lower_cur_left_calibration_paras, i, j);
								ret(loop, k) = (upper_value(0, 0) - lower_value(0, 0)) / (2.0 * eps);
								ret(loop + 1, k) = (upper_value(1, 0) - lower_value(1, 0)) / (2.0 * eps);
								loop += 2;
							}
						}
						else
						{
							loop += left_image_points[i].size() * 2;
						}
					}
				}
				else
				{
					loop = left_image_points.size() * left_image_points[0].size() * 2;
					for (size_t i = 0; i < right_image_points.size(); ++i)
					{
						for (size_t j = 0; j < right_image_points[i].size(); ++j)
						{
							Matrix_Type upper_value = value(upper_cur_right_calibration_paras, i, j);
							//vec_lower[k] -= eps;
							Matrix_Type lower_value = value(lower_cur_right_calibration_paras, i, j);
							ret(loop, k) = (upper_value(0, 0) - lower_value(0, 0)) / (2.0 * eps);
							ret(loop + 1, k) = (upper_value(1, 0) - lower_value(1, 0)) / (2.0 * eps);
							loop += 2;
						}
					}
				}
			}
			return ret;
		}
		return Matrix_Type();
	};
	////////////////////////////////////////////////////
	m_kSize = left_calibration_paras.distortionVec.size();
	m_extSize = left_calibration_paras.extrinsicsMatrixs.size();
	Matrix_Type initial_pose = guessInitialPose(left_calibration_paras.extrinsicsMatrixs, right_calibration_paras.extrinsicsMatrixs);

	std::vector<Data_Type> composeVec;
	composeStereoPara(left_calibration_paras, right_calibration_paras, initial_pose, composeVec);
	/*std::cout << "Left Para:\n";
	std::cout << left_calibration_paras.intrinsicsMatrix << std::endl;
	std::cout << left_calibration_paras.extrinsicsMatrixs[0] << std::endl;
	std::cout << left_calibration_paras.extrinsicsMatrixs[1] << std::endl;
	std::cout << left_calibration_paras.extrinsicsMatrixs[2] << std::endl;
	std::cout << left_calibration_paras.extrinsicsMatrixs[3] << std::endl;
	std::cout << "Right Para:\n";
	std::cout << right_calibration_paras.intrinsicsMatrix << std::endl;
	std::cout << right_calibration_paras.extrinsicsMatrixs[0] << std::endl;
	std::cout << right_calibration_paras.extrinsicsMatrixs[1] << std::endl;
	std::cout << right_calibration_paras.extrinsicsMatrixs[2] << std::endl;
	std::cout << right_calibration_paras.extrinsicsMatrixs[3] << std::endl;
	std::cout << "initial_pose:\n";*/
	std::cout << initial_pose << std::endl;

	Matrix_Type residualError = residual(composeVec);
	std::cout << "residualError: " << residualError.norm() << std::endl;
	Matrix_Type JacobMat = jacobian(composeVec, update_intrinsics);
	//std::cout << "JacobMat:\n" << JacobMat << std::endl;
	Matrix_Type A = JacobMat.transpose() * JacobMat;
	//std::cout << "A:\n" << A << std::endl;
	Matrix_Type G = JacobMat.transpose() * residualError;
	//std::cout << "G:\n" << G << std::endl;
	Matrix_Type OE = Matrix_Type::Identity(JacobMat.cols(), JacobMat.cols());
	Data_Type threshold_step = 1.0e-5, current_step;
	Data_Type lamda = 1.0e-3, v = 2, r = 0;
	//std::cout << "lamda: " << lamda << std::endl;
	int k = 0, max_k = 100;
	int offset = 0;
	if (!update_intrinsics)
		offset = 8 + m_kSize * 2;
	while (k < max_k)
	{
		OE.diagonal() = A.diagonal();
		Matrix_Type istep = (A + lamda * OE).inverse();
		//std::cout<< "istep:\n" << istep << std::endl;
		Matrix_Type step = -1.0 * istep * G;
		//std::cout << "step:\n" << step << std::endl;
		if (step.norm() < threshold_step)
			break;
		else
		{
			std::vector<Data_Type> new_com_vec;
			composeStereoPara(left_calibration_paras, right_calibration_paras, initial_pose, new_com_vec);

			std::vector<Data_Type> deltavec;
			for (int i = 0; i < step.rows(); ++i)
				deltavec.push_back(step(i, 0));
			

			for (int i = 0; i < step.rows(); ++i)
				new_com_vec[i + offset] += deltavec[i];
			//std::cout << "deltaMat:\n" << deltaMat << std::endl;
			Matrix_Type newResidualError = residual(new_com_vec);
			std::cout << "newResidualError: " << newResidualError.norm() << std::endl;
			//r = (residualError.norm() * residualError.norm() - newResidualError.norm() * newResidualError.norm()) / (step.transpose() * (lamda * step + G));
			Matrix_Type step_t = step.transpose();
			Matrix_Type r0 = step_t * (step * lamda - G);
			r = (residualError.norm() * residualError.norm() - newResidualError.norm() * newResidualError.norm()) / r0(0, 0);
			if (r > 0)
			{
				composeVec = new_com_vec;
				deComposeStereoPara(composeVec, left_calibration_paras, right_calibration_paras, initial_pose);
				
				JacobMat = jacobian(composeVec, update_intrinsics);
				A = JacobMat.transpose() * JacobMat;
				residualError = residual(composeVec);
				G = JacobMat.transpose() * residualError;
				if (residualError.norm() < 1.0e-3)
					break;
				lamda = lamda * std::max(1.0 / 3.0, 1.0 - std::pow((2.0 * r - 1), 3));
				v = 2.0;
			}
			else
			{
				lamda = lamda * v;
				v = 2.0 * v;
			}
		}
		k++;
	}
	std::cout << "final ResidualError: " << residualError.norm() << std::endl;
	/*std::cout << "Left Para:\n";
	std::cout << left_calibration_paras.intrinsicsMatrix << std::endl;
	std::cout << left_calibration_paras.extrinsicsMatrixs[0] << std::endl;
	std::cout << left_calibration_paras.extrinsicsMatrixs[1] << std::endl;
	std::cout << left_calibration_paras.extrinsicsMatrixs[2] << std::endl;
	std::cout << left_calibration_paras.extrinsicsMatrixs[3] << std::endl;
	std::cout << "Right Para:\n";
	std::cout << right_calibration_paras.intrinsicsMatrix << std::endl;
	std::cout << right_calibration_paras.extrinsicsMatrixs[0] << std::endl;
	std::cout << right_calibration_paras.extrinsicsMatrixs[1] << std::endl;
	std::cout << right_calibration_paras.extrinsicsMatrixs[2] << std::endl;
	std::cout << right_calibration_paras.extrinsicsMatrixs[3] << std::endl;
	std::cout << "initial_pose:\n";*/
	std::cout << initial_pose << std::endl;
	rotation_mat = initial_pose.block(0, 0, 3, 3);
	translation_vector = initial_pose.block(0, 3, 3, 1);
	Matrix_Type T = Matrix_Type::Zero(3, 3);
	T(0, 1) = -translation_vector(2, 0); T(0, 2) = translation_vector(1, 0);
	T(1, 0) = translation_vector(2, 0); T(1, 2) = -translation_vector(0, 0);
	T(2, 0) = -translation_vector(1, 0); T(2, 1) = translation_vector(0, 0);
	essential_mat = T * rotation_mat;
	fundamental_mat = (right_calibration_paras.intrinsicsMatrix.inverse().transpose()) * essential_mat * (left_calibration_paras.intrinsicsMatrix.inverse());
	fundamental_mat = fundamental_mat * (1.0 / fundamental_mat(2, 2));
	return true; 
}

bool CStereoCameraCalibration::doStereoCalibrate(CameraCalibrationParas& left_calibration_paras, CameraCalibrationParas& right_calibration_paras,
	Matrix_Type& rotation_mat, Matrix_Type& translation_vector, Matrix_Type& essential_mat, Matrix_Type& fundamental_mat)
{
	bool ret = false;
	////////////////////////////////////////////////////
	Matrix_Type initial_pose = guessInitialPose(left_calibration_paras.extrinsicsMatrixs, right_calibration_paras.extrinsicsMatrixs);

	return ret;
}

void CStereoCameraCalibration::rodriguesTransform(Matrix_Type src, Matrix_Type& dst)
{
	if ((src.rows() == 1 && src.cols() == 3) || (src.cols() == 1 && src.rows() == 3))
	{
		if (src.rows() == 1)
			src = src.transpose();

		Data_Type theta = src.norm();

		if (theta < 1.0e-6)
		{
			dst = Matrix_Type::Identity(3, 3);
		}
		else
		{
			src = src / theta;

			Matrix_Type temp = Matrix_Type::Zero(3, 3);
			temp(0, 1) = -src(2, 0); temp(0, 2) = src(1, 0);
			temp(1, 0) = src(2, 0); temp(1, 2) = -src(0, 0);
			temp(2, 0) = -src(1, 0); temp(2, 1) = src(0, 0);

			dst = Matrix_Type::Identity(3, 3) + temp * sin(theta) + temp * temp * (1.0 - cos(theta));
		}
	}
	else if (src.cols() == 3 && src.rows() == 3)
	{
		Eigen::JacobiSVD<Matrix_Type> svd(src, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Matrix_Type R = svd.matrixU() * svd.matrixV().transpose();
		Matrix_Type r(1, 3);
		r << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
		Data_Type s = std::sqrt((r(0, 0) * r(0, 0) + r(0, 1) * r(0, 1) + r(0, 2) * r(0, 2)) * 0.25);
		Data_Type c = (R(0, 0) + R(1, 1) + R(2, 2) - 1) * 0.5;
		c = c > 1. ? 1. : c < -1. ? -1. : c;
		Data_Type theta = std::acos(c);
		if (s < 1.0e-5)
		{
			Data_Type t;
			if (c > 0)
			{
				r = Matrix_Type::Zero(1, 3);
			}
			else
			{
				t = (R(0, 0) + 1) * 0.5;
				r(0, 0) = std::sqrt(std::max(t, Data_Type(0.)));
				t = (R(1, 1) + 1) * 0.5;
				r(0, 1) = std::sqrt(std::max(t, Data_Type(0.))) * (R(0, 1) < 0 ? -1. : 1.);
				t = (R(2, 2) + 1) * 0.5;
				r(0, 2) = std::sqrt(std::max(t, Data_Type(0.))) * (R(0, 2) < 0 ? -1. : 1.);
				if (fabs(r(0, 0)) < fabs(r(0, 1)) && fabs(r(0, 0)) < fabs(r(0, 2)) && (R(1, 2) > 0) != (r(0, 1) * r(0, 2) > 0))
					r(0, 2) = -r(0, 2);
				theta /= r.norm();
				r *= theta;
			}
		}
		else
		{
			Data_Type vth = 1 / (2 * s);
			vth *= theta;
			r *= vth;
		}
		dst = r.transpose();
	}
}

Matrix_Type CStereoCameraCalibration::guessInitialPose(std::vector<Matrix_Type>& left_camera, std::vector<Matrix_Type>& right_camera)
{
	Matrix_Type pose;
	pose = Matrix_Type::Zero(3, 4);
	pose.block(0, 0, 3, 3) = Matrix_Type::Identity(3, 3);
	if (left_camera.empty() || right_camera.empty() || left_camera.size() != right_camera.size())
	{	
		return pose;
	}
	size_t count = left_camera.size();
	std::vector<std::vector<Data_Type>> poseVec(6, std::vector<Data_Type>());
	for (size_t i = 0; i < count; ++i)
	{
		Matrix_Type R = right_camera[i].leftCols(3) * (left_camera[i].leftCols(3).transpose());
		Matrix_Type T = right_camera[i].block<3, 1>(0, 3) - R * left_camera[i].block<3, 1>(0, 3);
		Matrix_Type vec;
		rodriguesTransform(R, vec);
		poseVec[0].push_back(vec(0, 0));
		poseVec[1].push_back(vec(1, 0));
		poseVec[2].push_back(vec(2, 0));
		poseVec[3].push_back(T(0, 0));
		poseVec[4].push_back(T(1, 0));
		poseVec[5].push_back(T(2, 0));
	}
	Matrix_Type t_P(6, 1);
	for (size_t i = 0; i < 6; ++i)
	{
		std::sort(poseVec[i].begin(), poseVec[i].end());
		t_P(i, 0) = poseVec[i][count / 2];
	}
	Matrix_Type t_R;
	rodriguesTransform(t_P.block<3, 1>(0, 0), t_R);
	pose.block<3, 3>(0, 0) = t_R;
	pose(0, 3) = t_P(3, 0);
	pose(1, 3) = t_P(4, 0);
	pose(2, 3) = t_P(5, 0);
	return pose;
}

void CStereoCameraCalibration::composeStereoPara(CameraCalibrationParas& left_calibration_paras, CameraCalibrationParas& right_calibration_paras, Matrix_Type pose, std::vector<Data_Type>& vec)
{
	vec.clear();
	Matrix_Type rvec;

	vec.push_back(left_calibration_paras.intrinsicsMatrix(0, 0));
	//vec.push_back(left_calibration_paras.intrinsicsMatrix(0, 1));
	vec.push_back(left_calibration_paras.intrinsicsMatrix(1, 1));
	vec.push_back(left_calibration_paras.intrinsicsMatrix(0, 2));
	vec.push_back(left_calibration_paras.intrinsicsMatrix(1, 2));
	for (size_t i = 0; i < left_calibration_paras.distortionVec.size(); ++i)
	{
		vec.push_back(left_calibration_paras.distortionVec[i]);
	}
	vec.push_back(right_calibration_paras.intrinsicsMatrix(0, 0));
	//vec.push_back(right_calibration_paras.intrinsicsMatrix(0, 1));
	vec.push_back(right_calibration_paras.intrinsicsMatrix(1, 1));
	vec.push_back(right_calibration_paras.intrinsicsMatrix(0, 2));
	vec.push_back(right_calibration_paras.intrinsicsMatrix(1, 2));
	for (size_t i = 0; i < right_calibration_paras.distortionVec.size(); ++i)
	{
		vec.push_back(right_calibration_paras.distortionVec[i]);
	}
	for (size_t i = 0; i < left_calibration_paras.extrinsicsMatrixs.size(); ++i)
	{
		rodriguesTransform(left_calibration_paras.extrinsicsMatrixs[i].leftCols(3), rvec);
		vec.push_back(rvec(0, 0));
		vec.push_back(rvec(1, 0));
		vec.push_back(rvec(2, 0));
		vec.push_back(left_calibration_paras.extrinsicsMatrixs[i](0, 3));
		vec.push_back(left_calibration_paras.extrinsicsMatrixs[i](1, 3));
		vec.push_back(left_calibration_paras.extrinsicsMatrixs[i](2, 3));
	}
	rodriguesTransform(pose.leftCols(3), rvec);
	vec.push_back(rvec(0, 0));
	vec.push_back(rvec(1, 0));
	vec.push_back(rvec(2, 0));
	vec.push_back(pose(0, 3));
	vec.push_back(pose(1, 3));
	vec.push_back(pose(2, 3));
}

void CStereoCameraCalibration::deComposeStereoPara(std::vector<Data_Type> vec, CameraCalibrationParas& left_calibration_paras, CameraCalibrationParas& right_calibration_paras, Matrix_Type& pose)
{
	size_t vec_size = vec.size();
	if (vec_size == 8/*10*/ + m_kSize * 2 + m_extSize * 6 + 6)
	{
		left_calibration_paras.intrinsicsMatrix = Matrix_Type::Identity(3, 3);
		left_calibration_paras.intrinsicsMatrix(0, 0) = vec[0];
		//left_calibration_paras.intrinsicsMatrix(0, 1) = 0/*vec[1]*/;
		left_calibration_paras.intrinsicsMatrix(1, 1) = vec[1];
		left_calibration_paras.intrinsicsMatrix(0, 2) = vec[2];
		left_calibration_paras.intrinsicsMatrix(1, 2) = vec[3];
		left_calibration_paras.distortionVec.clear();
		for (size_t i = 0; i < m_kSize; ++i)
		{
			left_calibration_paras.distortionVec.push_back(vec[4 + i]);
		}
		right_calibration_paras.intrinsicsMatrix = Matrix_Type::Identity(3, 3);
		right_calibration_paras.intrinsicsMatrix(0, 0) = vec[4 + m_kSize];
		//right_calibration_paras.intrinsicsMatrix(0, 1) = 0/*vec[5 + m_kSize + 1]*/;
		right_calibration_paras.intrinsicsMatrix(1, 1) = vec[4 + m_kSize + 1];
		right_calibration_paras.intrinsicsMatrix(0, 2) = vec[4 + m_kSize + 2];
		right_calibration_paras.intrinsicsMatrix(1, 2) = vec[4 + m_kSize + 3];
		right_calibration_paras.distortionVec.clear();
		for (size_t i = 0; i < m_kSize; ++i)
		{
			right_calibration_paras.distortionVec.push_back(vec[8 + m_kSize + i]);
		}
		left_calibration_paras.extrinsicsMatrixs.clear();
		for (size_t i = 0; i < m_extSize; ++i)
		{
			Matrix_Type rvec(3, 1);
			int idx = i * 6 + m_kSize * 2 + 8;
			rvec << vec[idx], vec[idx + 1], vec[idx + 2];
			Matrix_Type rot;
			rodriguesTransform(rvec, rot);
			Matrix_Type temp_extrinsic = Matrix_Type::Zero(3, 4);
			temp_extrinsic.block(0, 0, 3, 3) = rot;
			temp_extrinsic(0, 3) = vec[idx + 3];
			temp_extrinsic(1, 3) = vec[idx + 4];
			temp_extrinsic(2, 3) = vec[idx + 5];
			left_calibration_paras.extrinsicsMatrixs.push_back(temp_extrinsic);
		}
		Matrix_Type rvec(3, 1);
		int idx = m_kSize * 2 + 8 + m_extSize * 6;
		rvec << vec[idx], vec[idx + 1], vec[idx + 2];
		Matrix_Type rot;
		rodriguesTransform(rvec, rot);
		pose = Matrix_Type::Zero(3, 4);
		pose.block(0, 0, 3, 3) = rot;
		pose(0, 3) = vec[idx + 3];
		pose(1, 3) = vec[idx + 4];
		pose(2, 3) = vec[idx + 5];
		right_calibration_paras.extrinsicsMatrixs.clear();
		for (size_t i = 0; i < m_extSize; ++i)
		{
			Matrix_Type rvec = Matrix_Type::Identity(4, 4);
			rvec.block(0, 0, 3, 4) = left_calibration_paras.extrinsicsMatrixs[i];
			Matrix_Type temp_extrinsic = pose * rvec;
			right_calibration_paras.extrinsicsMatrixs.push_back(temp_extrinsic);
		}
	}
}