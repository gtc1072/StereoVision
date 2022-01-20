#include "CMonocularCameraCalibration.h"

CMonocularCameraCalibration::CMonocularCameraCalibration()
{
	m_bForceFs2zero = true;
	m_bKFirst = true;
	m_bk2 = true;
}

CMonocularCameraCalibration::~CMonocularCameraCalibration()
{

}

bool CMonocularCameraCalibration::doCalibrate(std::vector<Point3_Type> world_points, std::vector<std::vector<Point2_Type>> image_points, int imageWidth, int imageHeight)
{
	bool ret = false;
	if (imageWidth > 0 && imageHeight > 0 && image_points.size() > 2 && world_points.size() > 3 && (world_points.size() == image_points[0].size()))
	{
		m_imageWidth = imageWidth;
		m_imageHeight = imageHeight;
		m_worldPoints = world_points;
		m_imagePoints = image_points;
		m_homographyMatrixs.clear();
	}
	else
		return ret;
	int frame_size = m_imagePoints.size();

	m_bForceFs2zero = true;
	m_bKFirst = true;
	m_bk2 = false;

	if (frame_size >= 3)
	{
		for (int i = 0; i < frame_size; ++i)
		{
			Matrix_Type Hi = getHomography(i);
			refineHomography(i, Hi);
			m_homographyMatrixs.push_back(Hi);
			std::cout << "Homography Matrix " << i << ":" << std::endl << Hi << std::endl;
		}

		Matrix_Type Intrinsic;
		if (m_bForceFs2zero)
		{
			Intrinsic = getIntrinsic2();
			std::cout << "Intrinsic Matrix:\n" << Intrinsic << std::endl;
			m_intrinsicsMatrix = Intrinsic;
		}
		else
		{
			Intrinsic = getIntrinsic/*ForceZero*/();
			std::cout << "Intrinsic Matrix:\n" << Intrinsic << std::endl;
			m_intrinsicsMatrix = Intrinsic;
		}

		std::vector<Matrix_Type> Extrinsic = getExtrinsics();
		//std::cout << "Extrinsic Matrix:\n" << Extrinsic << std::endl;
		m_extrinsicsMatrixs = Extrinsic;

		std::vector<Data_Type> distortion = getDistortion();
		m_distortionVec = distortion;

		if (m_bk2)
			std::cout << "Distortion: " << m_distortionVec[0] << " " << m_distortionVec[1] << std::endl;
		else
			std::cout << "Distortion: " << m_distortionVec[0] << " " << m_distortionVec[1] << " " << m_distortionVec[2] << " " << m_distortionVec[3] << " " << m_distortionVec[4] << std::endl;

		for (int i = 0; i < m_extrinsicsMatrixs.size(); ++i)
		{
			std::cout << "Extrinsic Matrix " << i << ":\n";
			std::cout << m_extrinsicsMatrixs[i] << std::endl;
		}

		refineAllPara();
		std::cout << "After refinement:\n";
		std::cout << "Intrinsic Matrix:\n" << m_intrinsicsMatrix << std::endl;
		if (m_bk2)
			std::cout << "Distortion: " << m_distortionVec[0] << " " << m_distortionVec[1] << std::endl;
		else
			std::cout << "Distortion: " << m_distortionVec[0] << " " << m_distortionVec[1] << " " << m_distortionVec[2] << " " << m_distortionVec[3] << " " << m_distortionVec[4] << std::endl;


		for (int i = 0; i < m_extrinsicsMatrixs.size(); ++i)
		{
			std::cout << "Extrinsic Matrix " << i << ":\n";
			std::cout << m_extrinsicsMatrixs[i] << std::endl;
		}
	}
	return ret;
}

Matrix_Type CMonocularCameraCalibration::getHomography(int idx)
{
	if (idx >= 0 && idx < m_imagePoints.size())
	{
		std::pair<Matrix_Type, Matrix_Type> ret = normalizing_frame(idx);
		//std::cout << "Normal Left:\n" << ret.first << std::endl;
		//std::cout << "Normal Right:\n" << ret.second << std::endl;
		int frame_size = m_imagePoints[idx].size();
		Eigen::Matrix<Data_Type, 3, 1> p_r, p_p;
		Matrix_Type M(frame_size * 2, 9);
		for (int i = 0; i < frame_size; ++i)
		{
			p_r(0, 0) = m_worldPoints[i].x;
			p_r(1, 0) = m_worldPoints[i].y; p_r(2, 0) = 1;
			p_p(0, 0) = m_imagePoints[idx][i].x;
			p_p(1, 0) = m_imagePoints[idx][i].y; p_p(2, 0) = 1;

			p_r = ret.first * p_r;
			p_p = ret.second * p_p;
			M(i * 2, 0) = -p_r(0, 0); M(i * 2, 1) = -p_r(1, 0); M(i * 2, 2) = -1;
			M(i * 2, 3) = 0; M(i * 2, 4) = 0; M(i * 2, 5) = 0;
			M(i * 2, 6) = p_p(0, 0) * p_r(0, 0); M(i * 2, 7) = p_p(0, 0) * p_r(1, 0); M(i * 2, 8) = p_p(0, 0);

			M(i * 2 + 1, 0) = 0; M(i * 2 + 1, 1) = 0; M(i * 2 + 1, 2) = 0;
			M(i * 2 + 1, 3) = -p_r(0, 0); M(i * 2 + 1, 4) = -p_r(1, 0); M(i * 2 + 1, 5) = -1;
			M(i * 2 + 1, 6) = p_p(1, 0) * p_r(0, 0); M(i * 2 + 1, 7) = p_p(1, 0) * p_r(1, 0); M(i * 2 + 1, 8) = p_p(1, 0);
		}
		Eigen::JacobiSVD<Matrix_Type> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Matrix_Type sv = svd.matrixV();

		Matrix_Type ret2(3, 3);

		ret2(0, 0) = sv(0, 8); ret2(0, 1) = sv(1, 8); ret2(0, 2) = sv(2, 8);
		ret2(1, 0) = sv(3, 8); ret2(1, 1) = sv(4, 8); ret2(1, 2) = sv(5, 8);
		ret2(2, 0) = sv(6, 8); ret2(2, 1) = sv(7, 8); ret2(2, 2) = sv(8, 8);

		//std::cout << "Origin Matrix:\n" << ret2 << std::endl;

		ret2 = ret.second.inverse() * ret2 * ret.first;

		ret2(0, 0) = ret2(0, 0) / ret2(2, 2); ret2(0, 1) = ret2(0, 1) / ret2(2, 2); ret2(0, 2) = ret2(0, 2) / ret2(2, 2);
		ret2(1, 0) = ret2(1, 0) / ret2(2, 2); ret2(1, 1) = ret2(1, 1) / ret2(2, 2); ret2(1, 2) = ret2(1, 2) / ret2(2, 2);
		ret2(2, 0) = ret2(2, 0) / ret2(2, 2); ret2(2, 1) = ret2(2, 1) / ret2(2, 2); ret2(2, 2) = 1.0;

		return ret2;
	}
	return Matrix_Type(3, 3);
}

std::pair<Matrix_Type, Matrix_Type> CMonocularCameraCalibration::normalizing_frame(int idx)
{
	if (idx >= 0 && idx < m_imagePoints.size())
	{
		int size = m_imagePoints[idx].size();
		Matrix_Type src(3, size);
		Matrix_Type dst(3, size);
		for (int j = 0; j < size; ++j)
		{
			src(0, j) = m_worldPoints[j].x;
			src(1, j) = m_worldPoints[j].y;
			src(2, j) = 1.0;
			dst(0, j) = m_imagePoints[idx][j].x;
			dst(1, j) = m_imagePoints[idx][j].y;
			dst(2, j) = 1.0;
		}
		Matrix_Type src_mean(2, 1);
		Matrix_Type dst_mean(2, 1);
		src_mean << src.row(0).mean(), src.row(1).mean();
		dst_mean << dst.row(0).mean(), dst.row(1).mean();
		Matrix_Type src2(3, size);
		Matrix_Type dst2(3, size);
		Matrix_Type src3(3, size);
		Matrix_Type dst3(3, size);
		for (int i = 0; i < size; ++i)
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

		Data_Type ss0, ss1, sd0, sd1;
		ss0 = sqrt(2.0) / sqrt(src3.row(0).mean());
		ss1 = sqrt(2.0) / sqrt(src3.row(1).mean());
		sd0 = sqrt(2.0) / sqrt(dst3.row(0).mean());
		sd1 = sqrt(2.0) / sqrt(dst3.row(1).mean());

		Matrix_Type r_norm(3, 3), p_norm(3, 3);
		//Eigen::MatrixXd ts(3, 3);
		r_norm << ss0, 0, -ss0 * src_mean(0, 0), 0, ss1, -ss1 * src_mean(1, 0), 0, 0, 1;

		//Eigen::MatrixXd td(3, 3);
		p_norm << sd0, 0, -sd0 * dst_mean(0, 0), 0, sd1, -sd1 * dst_mean(1, 0), 0, 0, 1;

		return std::make_pair(r_norm, p_norm);
	}
	return std::make_pair(Matrix_Type::Identity(3, 3), Matrix_Type::Identity(3, 3));
}

void CMonocularCameraCalibration::refineHomography(int idx, Matrix_Type& homoMat)
{
	std::function<Matrix_Type(int idx, Matrix_Type& Mat/*, std::vector<int> lp*/)> residual = [this](int idx, Matrix_Type& Mat/*, std::vector<int> lp*/)->Matrix_Type {
		Eigen::Matrix<Data_Type, 3, 1> p_r, p_p, p_d;
		int frame_size = m_worldPoints.size();
		Matrix_Type ret = Matrix_Type::Zero(2 * frame_size, 1);
		for (int i = 0; i < frame_size; ++i)
		{
			//int lpid = lp[i];
			p_r(0, 0) = m_worldPoints[i].x;
			p_r(1, 0) = m_worldPoints[i].y; p_r(2, 0) = 1;
			p_p(0, 0) = m_imagePoints[idx][i].x;
			p_p(1, 0) = m_imagePoints[idx][i].y; p_p(2, 0) = 1;
			p_d = Mat * p_r;
			ret(2 * i, 0) = p_p(0, 0) - p_d(0, 0) / p_d(2, 0);
			ret(2 * i + 1, 0) = p_p(1, 0) - p_d(1, 0) / p_d(2, 0);
		}
		return ret;
	};
	std::function<Matrix_Type(int idx, Matrix_Type& Mat/*, std::vector<int> lp*/)> Jacobian4 = [this](int idx, Matrix_Type& Mat/*, std::vector<int> lp*/)->Matrix_Type {
		Eigen::Matrix<Data_Type, 9, 2> p_j = Eigen::Matrix<Data_Type, 9, 2>::Zero();
		Eigen::Matrix<Data_Type, 3, 1> p_r/*, p_p, p_d*/;
		int frame_size = m_worldPoints.size();
		Matrix_Type ret = Matrix_Type::Zero(8, frame_size * 2);
		for (int i = 0; i < frame_size; ++i)
		{
			//int lpid = lp[i];
			p_r(0, 0) = m_worldPoints[i].x;
			p_r(1, 0) = m_worldPoints[i].y; p_r(2, 0) = 1;

			Data_Type sx = Mat(0, 0) * p_r(0, 0) + Mat(0, 1) * p_r(1, 0) + Mat(0, 2);
			Data_Type sy = Mat(1, 0) * p_r(0, 0) + Mat(1, 1) * p_r(1, 0) + Mat(1, 2);
			Data_Type w = Mat(2, 0) * p_r(0, 0) + Mat(2, 1) * p_r(1, 0) + Mat(2, 2);
			Data_Type w2 = w * w;

			ret(0, i * 2) = -p_r(0, 0) / w;				ret(1, i * 2) = -p_r(1, 0) / w;				ret(2, i * 2) = -1.0 / w;
			ret(3, i * 2) = 0;							ret(4, i * 2) = 0;							ret(5, i * 2) = 0;
			ret(6, i * 2) = sx * p_r(0, 0) / w2;		ret(7, i * 2) = sx * p_r(1, 0) / w2;		//ret(8, i * 2) = sx / w2;

			ret(0, i * 2 + 1) = 0;						ret(1, i * 2 + 1) = 0;						ret(2, i * 2 + 1) = 0;
			ret(3, i * 2 + 1) = -p_r(0, 0) / w;			ret(4, i * 2 + 1) = -p_r(1, 0) / w;			ret(5, i * 2 + 1) = -1.0 / w;
			ret(6, i * 2 + 1) = sy * p_r(0, 0) / w2;	ret(7, i * 2 + 1) = sy * p_r(1, 0) / w2;	//ret(8, i * 2 + 1) = sy / w2;
		}
		return ret;
	};
	std::default_random_engine generator;
	std::uniform_int_distribution<int> distribution(0, m_imagePoints[idx].size() - 1);
	std::vector<int> batchIdxs;
	for (int i = 0; i < 48; ++i)
	{
		batchIdxs.push_back(i);
	}
	Matrix_Type residualError = residual(idx, homoMat/*, batchIdxs*/);
	std::cout << "residualError: " << residualError.norm() << std::endl;
	Matrix_Type JacobMat = Jacobian4(idx, homoMat/*, batchIdxs*/);
	Matrix_Type A = JacobMat * (JacobMat.transpose());
	//std::cout << "A:\n" << A << std::endl;
	Matrix_Type G = JacobMat * residualError;
	//std::cout << "G:\n" << G << std::endl;
	Matrix_Type OE = Matrix_Type::Identity(8, 8);
	Data_Type threshold_step = 1.0e-5/*, current_step*/;
	Data_Type lamda =  /*A.diagonal().maxCoeff() * */1.0e-3, v = 2.0, r = 0.0;
	//std::cout << "lamda: " << lamda << std::endl;
	int k = 0, max_k = 100;
	while (k < max_k)
	{
		for (int i = 0; i < 8; ++i)
		{
			OE(i, i) = A(i, i);
		}
		Matrix_Type istep = (A + lamda * OE).inverse();
		//std::cout<< "istep:\n" << istep << std::endl;
		Matrix_Type step = -1.0 * istep * G;
		//std::cout << "step:\n" << step << std::endl;
		if (step.norm() < threshold_step)
			break;
		else
		{
			Matrix_Type deltaMat(3, 3);
			deltaMat(0, 0) = step(0, 0); deltaMat(0, 1) = step(1, 0); deltaMat(0, 2) = step(2, 0);
			deltaMat(1, 0) = step(3, 0); deltaMat(1, 1) = step(4, 0); deltaMat(1, 2) = step(5, 0);
			deltaMat(2, 0) = step(6, 0); deltaMat(2, 1) = step(7, 0); deltaMat(2, 2) = 0.0/*step(8, 0)*/;
			//std::cout << "deltaMat:\n" << deltaMat << std::endl;
			deltaMat = homoMat + deltaMat;
			//std::cout << "deltaMat:\n" << deltaMat << std::endl;
			Matrix_Type newResidualError = residual(idx, deltaMat/*, batchIdxs*/);
			std::cout << "newResidualError: " << newResidualError.norm() << std::endl;
			//r = (residualError.norm() * residualError.norm() - newResidualError.norm() * newResidualError.norm()) / (step.transpose() * (lamda * step + G));
			Matrix_Type step_t = step.transpose();
			Matrix_Type r0 = step_t * (step * lamda - G);
			r = (residualError.norm() * residualError.norm() - newResidualError.norm() * newResidualError.norm()) / r0(0, 0);
			if (r > 0)
			{
				homoMat = deltaMat;
				batchIdxs.clear();
				for (int i = 0; i < 48; ++i)
				{
					batchIdxs.push_back(i);
				}
				JacobMat = Jacobian4(idx, homoMat/*, batchIdxs*/);
				A = JacobMat * (JacobMat.transpose());
				residualError = residual(idx, homoMat/*, batchIdxs*/);
				G = JacobMat * residualError;
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
}

Matrix_Type CMonocularCameraCalibration::getIntrinsic()
{
	int size = m_homographyMatrixs.size();
	if (size >= 3)
	{
		Matrix_Type V(size * 2, 6);
		for (int i = 0; i < size; ++i)
		{
			V(i * 2, 0) = m_homographyMatrixs[i](0, 0) * m_homographyMatrixs[i](0, 1);
			V(i * 2, 1) = m_homographyMatrixs[i](1, 0) * m_homographyMatrixs[i](0, 1) + m_homographyMatrixs[i](0, 0) * m_homographyMatrixs[i](1, 1);
			V(i * 2, 2) = m_homographyMatrixs[i](1, 0) * m_homographyMatrixs[i](1, 1);
			V(i * 2, 3) = m_homographyMatrixs[i](2, 0) * m_homographyMatrixs[i](0, 1) + m_homographyMatrixs[i](0, 0) * m_homographyMatrixs[i](2, 1);
			V(i * 2, 4) = m_homographyMatrixs[i](2, 0) * m_homographyMatrixs[i](1, 1) + m_homographyMatrixs[i](1, 0) * m_homographyMatrixs[i](2, 1);
			V(i * 2, 5) = m_homographyMatrixs[i](2, 0) * m_homographyMatrixs[i](2, 1);

			V(i * 2 + 1, 0) = m_homographyMatrixs[i](0, 0) * m_homographyMatrixs[i](0, 0) - m_homographyMatrixs[i](0, 1) * m_homographyMatrixs[i](0, 1);
			V(i * 2 + 1, 1) = (m_homographyMatrixs[i](1, 0) * m_homographyMatrixs[i](0, 0) + m_homographyMatrixs[i](0, 0) * m_homographyMatrixs[i](1, 0)) - (m_homographyMatrixs[i](1, 1) * m_homographyMatrixs[i](0, 1) + m_homographyMatrixs[i](0, 1) * m_homographyMatrixs[i](1, 1));
			V(i * 2 + 1, 2) = m_homographyMatrixs[i](1, 0) * m_homographyMatrixs[i](1, 0) - m_homographyMatrixs[i](1, 1) * m_homographyMatrixs[i](1, 1);
			V(i * 2 + 1, 3) = (m_homographyMatrixs[i](2, 0) * m_homographyMatrixs[i](0, 0) + m_homographyMatrixs[i](0, 0) * m_homographyMatrixs[i](2, 0)) - (m_homographyMatrixs[i](2, 1) * m_homographyMatrixs[i](0, 1) + m_homographyMatrixs[i](0, 1) * m_homographyMatrixs[i](2, 1));
			V(i * 2 + 1, 4) = (m_homographyMatrixs[i](2, 0) * m_homographyMatrixs[i](1, 0) + m_homographyMatrixs[i](1, 0) * m_homographyMatrixs[i](2, 0)) - (m_homographyMatrixs[i](2, 1) * m_homographyMatrixs[i](1, 1) + m_homographyMatrixs[i](1, 1) * m_homographyMatrixs[i](2, 1));
			V(i * 2 + 1, 5) = m_homographyMatrixs[i](2, 0) * m_homographyMatrixs[i](2, 0) - m_homographyMatrixs[i](2, 1) * m_homographyMatrixs[i](2, 1);
		}
		//std::cout << "V:\n" << V << std::endl;

		Eigen::JacobiSVD<Matrix_Type> svd(V.transpose() * V, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Matrix_Type sv = svd.matrixV();
		//std::cout << "sv\n" << sv << std::endl;

		Eigen::EigenSolver<Matrix_Type> eigen_solve(V.transpose() * V);

		Data_Type B0 = sv(0, 5); Data_Type B1 = sv(1, 5); Data_Type B2 = sv(2, 5);
		Data_Type B3 = sv(3, 5); Data_Type B4 = sv(4, 5); Data_Type B5 = sv(5, 5);

		std::cout << B0 << " " << B1 << " " << B2 << " " << B3 << " " << B4 << " " << B5 << std::endl;

		Matrix_Type test(3, 3);
		test << B0, B1, B3, B1, B2, B4, B3, B4, B5;
		//std::cout << "test:\n" << test << std::endl;

		if (B0 < 0 || B2 < 0 || B5 < 0)
		{
			B0 = -B0; B1 = -B1; B2 = -B2;
			B3 = -B3; B4 = -B4; B5 = -B5;
		}

		Matrix_Type ml = test.inverse().llt().matrixL();

		//std::cout << "ml:\n" << ml << std::endl;

		Data_Type d = B0 * B2 - B1 * B1;
		//std::cout << "d: " << d << std::endl;
		Data_Type w = B0 * B2 * B5 - B1 * B1 * B5 - B0 * B4 * B4 + 2.0 * B1 * B3 * B4 - B2 * B3 * B3;
		//std::cout << "w: " << w << std::endl;
		Data_Type v0 = (B1 * B3 - B0 * B4) / d;
		//std::cout << "V0 " << v0 << std::endl;
		Data_Type mu0 = (B1 * B4 - B2 * B3) / d;
		//std::cout << "mu0 " << mu0 << std::endl;
		Data_Type alpha = sqrt(w / (B0 * d));
		//std::cout << "alpha " << alpha << std::endl;
		Data_Type belta = sqrt(w / (d * d) * B0);
		//std::cout << "belta " << belta << std::endl;
		Data_Type gamma = sqrt(w / (d * d * B0)) * B1;
		//std::cout << "gamma " << gamma << std::endl;


		Matrix_Type ret(3 ,3);

		ret(0, 0) = alpha; ret(0, 1) = gamma; ret(0, 2) = mu0;
		ret(1, 0) = 0; ret(1, 1) = belta; ret(1, 2) = v0;
		ret(2, 0) = 0; ret(2, 1) = 0; ret(2, 2) = 1;

		return ret;
	}
	return Matrix_Type::Identity(3, 3);
}

Matrix_Type CMonocularCameraCalibration::getIntrinsic2()
{
	Matrix_Type ret = Matrix_Type::Zero(3, 3);
	ret(0, 2) = (m_imageWidth - 1) * 0.5;
	ret(1, 2) = (m_imageHeight - 1) * 0.5;

	int size = m_homographyMatrixs.size();
	if (size >= 3)
	{
		Matrix_Type Aa(size * 2, 2);
		Matrix_Type Bb(size * 2, 1);
		Matrix_Type temp_inv_m = Matrix_Type::Identity(3, 3);
		temp_inv_m(0, 2) = -ret(0, 2);
		temp_inv_m(1, 2) = -ret(1, 2);

		for (int i = 0; i < size; ++i)
		{
			Eigen::Matrix<Data_Type, 3, 3> tempHomography = temp_inv_m * m_homographyMatrixs[i];

			Data_Type h[3], v[3], d1[3], d2[3];
			Data_Type n[4] = { 0,0,0,0 };

			for (int j = 0; j < 3; j++)
			{
				Data_Type t0 = tempHomography(j, 0), t1 = tempHomography(j, 1);
				h[j] = t0; v[j] = t1;
				d1[j] = (t0 + t1) * 0.5;
				d2[j] = (t0 - t1) * 0.5;
				n[0] += t0 * t0; n[1] += t1 * t1;
				n[2] += d1[j] * d1[j]; n[3] += d2[j] * d2[j];
			}

			for (int j = 0; j < 4; j++)
				n[j] = 1. / sqrt(n[j]);

			for (int j = 0; j < 3; j++)
			{
				h[j] *= n[0]; v[j] *= n[1];
				d1[j] *= n[2]; d2[j] *= n[3];
			}

			Aa(i * 2, 0) = d1[0] * d2[0]; Aa(i * 2, 1) = d1[1] * d2[1];
			Aa(i * 2 + 1, 0) = h[0] * v[0]; Aa(i * 2 + 1, 1) = h[1] * v[1];
			Bb(i * 2, 0) = -d1[2] * d2[2]; Bb(i * 2 + 1, 0) = -h[2] * v[2];
		}

		Eigen::JacobiSVD<Matrix_Type> svd(Aa, Eigen::ComputeFullU | Eigen::ComputeFullV);

		Matrix_Type final1 = (Aa.transpose() * Aa).inverse() * Aa.transpose() * Bb;

		std::cout << "final:\n" << final1 << std::endl << sqrt(1.0 / fabs(final1(0, 0))) << "," << sqrt(1.0 / fabs(final1(1, 0))) << std::endl;

		ret(0, 0) = sqrt(1.0 / fabs(final1(0, 0)));
		ret(1, 1) = sqrt(1.0 / fabs(final1(1, 0)));
		ret(2, 2) = 1.0;
	}
	return ret;
}

std::vector<Matrix_Type> CMonocularCameraCalibration::getExtrinsics()
{
	std::vector<Matrix_Type> extrinsicsPara;
	Matrix_Type inv_instrinsic = m_intrinsicsMatrix.inverse();
	//std::cout << "inv_instrinsic:\n" << inv_instrinsic << std::endl;
	for (int i = 0; i < m_homographyMatrixs.size(); ++i)
	{
		Matrix_Type ret_i = Matrix_Type::Zero(3, 4);
		Data_Type scale_factor1 = (inv_instrinsic * m_homographyMatrixs[i].col(0)).norm();
		Data_Type scale_factor2 = (inv_instrinsic * m_homographyMatrixs[i].col(1)).norm();

		scale_factor1 = scale_factor1 < 1.0e-5 ? 1.0e-5 : scale_factor1;
		scale_factor2 = scale_factor2 < 1.0e-5 ? 1.0e-5 : scale_factor2;

		scale_factor1 = 1.0 / scale_factor1;
		scale_factor2 = 1.0 / scale_factor2;

		ret_i.col(0) = scale_factor1 * (inv_instrinsic * m_homographyMatrixs[i].col(0));
		ret_i.col(1) = scale_factor2 * (inv_instrinsic * m_homographyMatrixs[i].col(1));
		Vetor3_Type c1 = ret_i.col(0);
		Vetor3_Type c2 = ret_i.col(1);
		ret_i.col(2) = c1.cross(c2);
		ret_i.col(3) = (scale_factor1 + scale_factor2) * 0.5 * (inv_instrinsic * m_homographyMatrixs[i].col(2));

		Eigen::JacobiSVD<Matrix_Type> svd(ret_i.leftCols(3), Eigen::ComputeThinU | Eigen::ComputeThinV);

		ret_i.leftCols(3) = svd.matrixU() * (svd.matrixV().transpose());

		extrinsicsPara.push_back(ret_i);
	}
	return extrinsicsPara;
}

std::vector<Data_Type> CMonocularCameraCalibration::getDistortion()
{
	std::vector<Data_Type> ret; //k1 k2 ¾¶Ïò»û±ä
	if (m_bk2)
	{
		int count = 0;
		for (int j = 0; j < m_imagePoints.size(); ++j)
		{
			for (int i = 0; i < m_imagePoints[j].size(); ++i)
			{
				count += 2;
			}
		}
		Matrix_Type D(count, 2);
		Matrix_Type d(count, 1);
		int index = 0;
		if (m_bKFirst)
		{
			for (int j = 0; j < m_imagePoints.size(); ++j)
			{
				for (int i = 0; i < m_imagePoints[j].size(); ++i)
				{
					Matrix_Type single_coor(4, 1);
					single_coor(0, 0) = m_worldPoints[i].x;
					single_coor(1, 0) = m_worldPoints[i].y;
					single_coor(2, 0) = 0;
					single_coor(3, 0) = 1;
					Matrix_Type single_img_coor(3, 1);
					single_img_coor(0, 0) = m_imagePoints[j][i].x;
					single_img_coor(1, 0) = m_imagePoints[j][i].y;
					single_img_coor(2, 0) = 1;

					Matrix_Type u = m_intrinsicsMatrix.inverse() * single_img_coor;
					Data_Type ux = u(0, 0);
					Data_Type uy = u(1, 0);

					Matrix_Type n = m_extrinsicsMatrixs[j] * single_coor;
					Data_Type nx = n(0, 0) / n(2, 0);
					Data_Type ny = n(1, 0) / n(2, 0);
					Data_Type r = nx * nx + ny * ny;

					D(index, 0) = nx * r; D(index, 1) = nx * r * r;
					D(index + 1, 0) = ny * r; D(index + 1, 1) = ny * r * r;

					d(index, 0) = ux - nx;
					d(index + 1, 0) = uy - ny;

					index += 2;
				}
			}
		}
		else
		{
			for (int j = 0; j < m_imagePoints.size(); ++j)
			{
				for (int i = 0; i < m_imagePoints[j].size(); ++i)
				{
					Matrix_Type single_coor(4, 1);
					single_coor(0, 0) = m_worldPoints[i].x;
					single_coor(1, 0) = m_worldPoints[i].y;
					single_coor(2, 0) = 0;
					single_coor(3, 0) = 1;

					Matrix_Type u = m_intrinsicsMatrix * m_extrinsicsMatrixs[j] * single_coor;
					Data_Type ux = u(0, 0) / u(2, 0);
					Data_Type uy = u(1, 0) / u(2, 0);

					Matrix_Type n = m_extrinsicsMatrixs[j] * single_coor;
					Data_Type nx = n(0, 0) / n(2, 0);
					Data_Type ny = n(1, 0) / n(2, 0);
					Data_Type r = nx * nx + ny * ny;

					D(index, 0) = (ux - m_intrinsicsMatrix(0, 2)) * r; D(index, 1) = (ux - m_intrinsicsMatrix(0, 2)) * r * r;
					D(index + 1, 0) = (uy - m_intrinsicsMatrix(1, 2)) * r; D(index + 1, 1) = (uy - m_intrinsicsMatrix(1, 2)) * r * r;

					d(index, 0) = m_imagePoints[j][i].x - ux;
					d(index + 1, 0) = m_imagePoints[j][i].y - uy;

					index += 2;
				}
			}
		}
		Matrix_Type K = (D.transpose() * D).inverse() * (D.transpose()) * d;

		ret.push_back(K(0, 0));
		ret.push_back(K(1, 0));
	}
	else //k1 k2 p1 p2 k3
	{
		ret.push_back(0);
		ret.push_back(0);
		ret.push_back(0);
		ret.push_back(0);
		ret.push_back(0);
	}

	return ret;
}

void CMonocularCameraCalibration::refineAllPara()
{
	std::function<Matrix_Type(std::vector<Data_Type> para, int frameIdx, int pointIdx)> value = [this](std::vector<Data_Type> para, int frameIdx, int pointIdx)->Matrix_Type {
		Matrix_Type p_r(4, 1);
		//p_r(0, 0) = m_frames[frameIdx][pointIdx].first.getAt(0);
		//p_r(1, 0) = m_frames[frameIdx][pointIdx].first.getAt(1);
		p_r(0, 0) = m_worldPoints[pointIdx].x;
		p_r(1, 0) = m_worldPoints[pointIdx].y;
		p_r(2, 0) = 0;
		p_r(3, 0) = 1;

		Matrix_Type upper_intrinsic;
		std::vector<Matrix_Type> upper_extrinsics;
		std::vector<Data_Type> upper_distortion;

		decomposeCameraPara(para, upper_intrinsic, upper_distortion, upper_extrinsics);

		Matrix_Type ret = Matrix_Type::Zero(2, 1);

		if (m_bk2)
		{
			if (m_bKFirst)
			{
				Matrix_Type n = upper_extrinsics[frameIdx] * p_r;
				Data_Type nx = n(0, 0) / n(2, 0);
				Data_Type ny = n(1, 0) / n(2, 0);
				Data_Type r = nx * nx + ny * ny;
				//r = r * r;

				Data_Type xe = nx * (1.0 + upper_distortion[0] * r + upper_distortion[1] * r * r);
				Data_Type ye = ny * (1.0 + upper_distortion[0] * r + upper_distortion[1] * r * r);

				ret(0, 0) = upper_intrinsic(0, 0) * xe + upper_intrinsic(0, 1) * ye + upper_intrinsic(0, 2);
				ret(1, 0) = upper_intrinsic(1, 1) * ye + upper_intrinsic(1, 2);
			}
			else
			{
				Matrix_Type u = upper_intrinsic * upper_extrinsics[frameIdx] * p_r;
				Data_Type ux = u(0, 0) / u(2, 0);
				Data_Type uy = u(1, 0) / u(2, 0);

				Matrix_Type n = upper_extrinsics[frameIdx] * p_r;
				Data_Type nx = n(0, 0) / n(2, 0);
				Data_Type ny = n(1, 0) / n(2, 0);
				Data_Type r = nx * nx + ny * ny;
				//r = r * r;

				Data_Type xe = ux + (ux - upper_intrinsic(0, 2)) * (upper_distortion[0] * r + upper_distortion[1] * r * r);
				Data_Type ye = uy + (uy - upper_intrinsic(1, 2)) * (upper_distortion[0] * r + upper_distortion[1] * r * r);

				ret(0, 0) = xe;
				ret(1, 0) = ye;
				//ret << xe, ye;
			}
		}
		else
		{
			Matrix_Type n = upper_extrinsics[frameIdx] * p_r;
			Data_Type nx = n(0, 0) / n(2, 0);
			Data_Type ny = n(1, 0) / n(2, 0);
			Data_Type r = nx * nx + ny * ny;
			//r = r * r;

			Data_Type xe = nx * (1.0 + upper_distortion[0] * r + upper_distortion[1] * r * r + upper_distortion[4] * r * r * r) + 2.0 * upper_distortion[2] * nx * ny + upper_distortion[3] * (r * r + 2.0 * nx * nx);
			Data_Type ye = ny * (1.0 + upper_distortion[0] * r + upper_distortion[1] * r * r + upper_distortion[4] * r * r * r) + upper_distortion[2] * (r * r + 2.0 * ny * ny) + 2.0 * upper_distortion[3] * nx * ny;

			ret(0, 0) = upper_intrinsic(0, 0) * xe + upper_intrinsic(0, 1) * ye + upper_intrinsic(0, 2);
			ret(1, 0) = upper_intrinsic(1, 1) * ye + upper_intrinsic(1, 2);
		}

		return ret;
	};
	std::function<Matrix_Type(std::vector<Data_Type> vec)> residual = [&](std::vector<Data_Type> vec)->Matrix_Type {
		Eigen::Matrix<Data_Type, 2, 1> p_r, p_p;
		int matrixRows = 0, rowLoop = 0;
		for (int i = 0; i < m_imagePoints.size(); ++i)
		{
			matrixRows += m_imagePoints[i].size() * 2;
		}
		Matrix_Type ret = Matrix_Type::Zero(matrixRows, 1);
		for (int i = 0; i < m_imagePoints.size(); ++i)
		{
			for (int j = 0; j < m_imagePoints[i].size(); ++j)
			{
				p_r = value(vec, i, j);

				p_p(0, 0) = m_imagePoints[i][j].x;
				p_p(1, 0) = m_imagePoints[i][j].y; //p_p(2, 0) = 1;
				ret(rowLoop, 0) = p_p(0, 0) - p_r(0, 0);
				ret(rowLoop + 1, 0) = p_p(1, 0) - p_r(1, 0);

				rowLoop += 2;
			}
		}
		return ret;
	};
	std::function<Matrix_Type()> jacobian = [&]()->Matrix_Type {
		Data_Type eps;
		if (typeid(Data_Type) == typeid(double))
			eps = 1.0e-7;
		else
			eps = 0.00005;
		int matrixRows = 0, rowLoop = 0;
		for (int i = 0; i < m_imagePoints.size(); ++i)
		{
			matrixRows += m_imagePoints[i].size() * 2;
		}
		int midk = getIntrinsicsKOffset();
		int matrixCols = midk + m_imagePoints.size() * 6;
		Matrix_Type ret = Matrix_Type::Zero(matrixRows, matrixCols);

		std::vector<Data_Type> com_vec;
		composeCameraPara(m_intrinsicsMatrix, m_distortionVec, m_extrinsicsMatrixs, com_vec);

		int loop = 0;
		for (int i = 0; i < m_imagePoints.size(); ++i)
		{
			int leftk = midk + i * 6;
			int rightk = midk + (i + 1) * 6;
			for (int j = 0; j < m_imagePoints[i].size(); ++j)
			{
				std::vector<Data_Type> vec_upper;
				std::vector<Data_Type> vec_lower;
				for (int k = 0; k < midk; ++k)
				{
					//if (k == 1) continue;
					vec_upper = com_vec;
					vec_lower = com_vec;
					vec_upper[k] += eps;
					vec_lower[k] -= eps;
					Matrix_Type upper_value = value(vec_upper, i, j);
					Matrix_Type lower_value = value(vec_lower, i, j);
					ret(loop, k) = -(upper_value(0, 0) - lower_value(0, 0)) / (2.0 * eps);
					ret(loop + 1, k) = -(upper_value(1, 0) - lower_value(1, 0)) / (2.0 * eps);
				}
				for (int k = leftk; k < rightk; ++k)
				{
					vec_upper = com_vec;
					vec_lower = com_vec;
					vec_upper[k] += eps;
					vec_lower[k] -= eps;
					Matrix_Type upper_value = value(vec_upper, i, j);
					Matrix_Type lower_value = value(vec_lower, i, j);
					ret(loop, k) = -(upper_value(0, 0) - lower_value(0, 0)) / (2.0 * eps);
					ret(loop + 1, k) = -(upper_value(1, 0) - lower_value(1, 0)) / (2.0 * eps);
				}
				loop += 2;
			}
		}
		return ret;
	};
	std::function<Matrix_Type(Matrix_Type&, std::vector<Matrix_Type>&, std::vector<Data_Type>&, int frameIdx, int pointIdx)> value2 = [this](Matrix_Type &upper_intrinsic, std::vector<Matrix_Type> &upper_extrinsics, std::vector<Data_Type> &upper_distortion, int frameIdx, int pointIdx)->Matrix_Type {
		Matrix_Type p_r(4, 1);
		p_r(0, 0) = m_worldPoints[pointIdx].x;
		p_r(1, 0) = m_worldPoints[pointIdx].y;
		p_r(2, 0) = 0;
		p_r(3, 0) = 1;

		Matrix_Type ret = Matrix_Type::Zero(2, 1);

		if (m_bk2)
		{
			if (m_bKFirst)
			{
				Matrix_Type n = upper_extrinsics[frameIdx] * p_r;
				Data_Type nx = n(0, 0) / n(2, 0);
				Data_Type ny = n(1, 0) / n(2, 0);
				Data_Type r = nx * nx + ny * ny;
				//r = r * r;

				Data_Type xe = nx * (1.0 + upper_distortion[0] * r + upper_distortion[1] * r * r);
				Data_Type ye = ny * (1.0 + upper_distortion[0] * r + upper_distortion[1] * r * r);

				ret(0, 0) = upper_intrinsic(0, 0) * xe + upper_intrinsic(0, 1) * ye + upper_intrinsic(0, 2);
				ret(1, 0) = upper_intrinsic(1, 1) * ye + upper_intrinsic(1, 2);
			}
			else
			{
				Matrix_Type u = upper_intrinsic * upper_extrinsics[frameIdx] * p_r;
				Data_Type ux = u(0, 0) / u(2, 0);
				Data_Type uy = u(1, 0) / u(2, 0);

				Matrix_Type n = upper_extrinsics[frameIdx] * p_r;
				Data_Type nx = n(0, 0) / n(2, 0);
				Data_Type ny = n(1, 0) / n(2, 0);
				Data_Type r = nx * nx + ny * ny;
				//r = r * r;

				Data_Type xe = ux + (ux - upper_intrinsic(0, 2)) * (upper_distortion[0] * r + upper_distortion[1] * r * r);
				Data_Type ye = uy + (uy - upper_intrinsic(1, 2)) * (upper_distortion[0] * r + upper_distortion[1] * r * r);

				ret(0, 0) = xe;
				ret(1, 0) = ye;
				//ret << xe, ye;
			}
		}
		else
		{
			Matrix_Type n = upper_extrinsics[frameIdx] * p_r;
			Data_Type nx = n(0, 0) / n(2, 0);
			Data_Type ny = n(1, 0) / n(2, 0);
			Data_Type r = nx * nx + ny * ny;
			//r = r * r;

			Data_Type xe = nx * (1.0 + upper_distortion[0] * r + upper_distortion[1] * r * r + upper_distortion[4] * r * r * r) + 2.0 * upper_distortion[2] * nx * ny + upper_distortion[3] * (r * r + 2.0 * nx * nx);
			Data_Type ye = ny * (1.0 + upper_distortion[0] * r + upper_distortion[1] * r * r + upper_distortion[4] * r * r * r) + upper_distortion[2] * (r * r + 2.0 * ny * ny) + 2.0 * upper_distortion[3] * nx * ny;

			ret(0, 0) = upper_intrinsic(0, 0) * xe + upper_intrinsic(0, 1) * ye + upper_intrinsic(0, 2);
			ret(1, 0) = upper_intrinsic(1, 1) * ye + upper_intrinsic(1, 2);
		}

		return ret;
	};
	std::function<Matrix_Type(std::vector<Data_Type> vec)> residual2 = [&](std::vector<Data_Type> vec)->Matrix_Type {
		Eigen::Matrix<Data_Type, 2, 1> p_r, p_p;
		int matrixRows = 0, rowLoop = 0;
		for (int i = 0; i < m_imagePoints.size(); ++i)
		{
			matrixRows += m_imagePoints[i].size() * 2;
		}
		Matrix_Type ret = Matrix_Type::Zero(matrixRows, 1);

		Matrix_Type upper_intrinsic;
		std::vector<Matrix_Type> upper_extrinsics;
		std::vector<Data_Type> upper_distortion;

		decomposeCameraPara(vec, upper_intrinsic, upper_distortion, upper_extrinsics);
		for (int i = 0; i < m_imagePoints.size(); ++i)
		{
			for (int j = 0; j < m_imagePoints[i].size(); ++j)
			{
				p_r = value2(upper_intrinsic, upper_extrinsics, upper_distortion, i, j);

				p_p(0, 0) = m_imagePoints[i][j].x;
				p_p(1, 0) = m_imagePoints[i][j].y; //p_p(2, 0) = 1;
				ret(rowLoop, 0) = p_p(0, 0) - p_r(0, 0);
				ret(rowLoop + 1, 0) = p_p(1, 0) - p_r(1, 0);

				rowLoop += 2;
			}
		}
		return ret;
	};
	std::function<Matrix_Type()> jacobian2 = [&]()->Matrix_Type {
		Data_Type eps;
		if (typeid(Data_Type) == typeid(double))
			eps = 1.0e-7;
		else
			eps = 0.00005;
		int matrixRows = 0, rowLoop = 0;
		for (int i = 0; i < m_imagePoints.size(); ++i)
		{
			matrixRows += m_imagePoints[i].size() * 2;
		}
		int midk = getIntrinsicsKOffset();
		int matrixCols = midk + m_imagePoints.size() * 6;
		Matrix_Type ret = Matrix_Type::Zero(matrixRows, matrixCols);

		std::vector<Data_Type> com_vec;
		composeCameraPara(m_intrinsicsMatrix, m_distortionVec, m_extrinsicsMatrixs, com_vec);

		std::vector<Data_Type> vec_upper;
		std::vector<Data_Type> vec_lower;
		for (int k = 0; k < matrixCols; ++k)
		{
			vec_upper = com_vec;
			vec_lower = com_vec;
			vec_upper[k] += eps;
			vec_lower[k] -= eps;
			Matrix_Type upper_intrinsic;
			std::vector<Matrix_Type> upper_extrinsics;
			std::vector<Data_Type> upper_distortion;
			decomposeCameraPara(vec_upper, upper_intrinsic, upper_distortion, upper_extrinsics);

			Matrix_Type lower_intrinsic;
			std::vector<Matrix_Type> lower_extrinsics;
			std::vector<Data_Type> lower_distortion;
			decomposeCameraPara(vec_lower, lower_intrinsic, lower_distortion, lower_extrinsics);

			if (k < midk)
			{
				int loop = 0;
				for (int i = 0; i < m_imagePoints.size(); ++i)
				{
					for (int j = 0; j < m_imagePoints[i].size(); ++j)
					{
						Matrix_Type upper_value = value2(upper_intrinsic, upper_extrinsics, upper_distortion, i, j);
						Matrix_Type lower_value = value2(lower_intrinsic, lower_extrinsics, lower_distortion, i, j);
						ret(loop, k) = -(upper_value(0, 0) - lower_value(0, 0)) / (2.0 * eps);
						ret(loop + 1, k) = -(upper_value(1, 0) - lower_value(1, 0)) / (2.0 * eps);
						loop += 2;
					}
				}
			}
			else
			{
				int loop = 0;
				for (int i = 0; i < m_imagePoints.size(); ++i)
				{
					int leftk = midk + i * 6;
					int rightk = midk + (i + 1) * 6;
					if (k >= leftk && k < rightk)
					{
						for (int j = 0; j < m_imagePoints[i].size(); ++j)
						{
							Matrix_Type upper_value = value2(upper_intrinsic, upper_extrinsics, upper_distortion, i, j);
							Matrix_Type lower_value = value2(lower_intrinsic, lower_extrinsics, lower_distortion, i, j);
							ret(loop, k) = -(upper_value(0, 0) - lower_value(0, 0)) / (2.0 * eps);
							ret(loop + 1, k) = -(upper_value(1, 0) - lower_value(1, 0)) / (2.0 * eps);
							loop += 2;
						}
					}
					else
					{
						loop += m_imagePoints[i].size() * 2;
					}
				}
			}
		}
		return ret;
	};

	std::vector<Data_Type> com_vec;
	composeCameraPara(m_intrinsicsMatrix, m_distortionVec, m_extrinsicsMatrixs, com_vec);
	Matrix_Type residualError = residual2(com_vec);
	std::cout << "residualError: " << residualError.norm() << std::endl;
	Matrix_Type JacobMat = jacobian2();
	Matrix_Type A = JacobMat.transpose() * JacobMat;
	//std::cout << "A:\n" << A << std::endl;
	Matrix_Type G = JacobMat.transpose() * residualError;
	//std::cout << "G:\n" << G << std::endl;
	Matrix_Type OE = Matrix_Type::Identity(JacobMat.cols(), JacobMat.cols());
	Data_Type threshold_step = 1.0e-5, current_step;
	Data_Type lamda = 1.0e-3, v = 2, r = 0;
	//std::cout << "lamda: " << lamda << std::endl;
	int k = 0, max_k = 100;
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
			composeCameraPara(m_intrinsicsMatrix, m_distortionVec, m_extrinsicsMatrixs, new_com_vec);

			std::vector<Data_Type> deltavec;
			for (int i = 0; i < step.rows(); ++i)
				deltavec.push_back(step(i, 0));

			for (int i = 0; i < step.rows(); ++i)
				new_com_vec[i] += deltavec[i];
			//std::cout << "deltaMat:\n" << deltaMat << std::endl;
			Matrix_Type newResidualError = residual2(new_com_vec);
			std::cout << "newResidualError: " << newResidualError.norm() << std::endl;
			//r = (residualError.norm() * residualError.norm() - newResidualError.norm() * newResidualError.norm()) / (step.transpose() * (lamda * step + G));
			Matrix_Type step_t = step.transpose();
			Matrix_Type r0 = step_t * (step * lamda - G);
			r = (residualError.norm() * residualError.norm() - newResidualError.norm() * newResidualError.norm()) / r0(0, 0);
			if (r > 0)
			{
				com_vec = new_com_vec;
				decomposeCameraPara(com_vec, m_intrinsicsMatrix, m_distortionVec, m_extrinsicsMatrixs);
				JacobMat = jacobian2();
				A = JacobMat.transpose() * JacobMat;
				residualError = residual(com_vec);
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
	//std::cout << "final ResidualError: " << residualError << std::endl;
}

void CMonocularCameraCalibration::rodriguesTransform(Matrix_Type src, Matrix_Type& dst)
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

			//dst = Eigen::MatrixXd::Identity(3, 3) * cos(theta) + temp * sin(theta) + src * src * (1.0 - cos(theta));
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
	else
	{
		std::cout << "rodriguesTransform -> Unsupported Matrix Size." << std::endl;
	}
}

void CMonocularCameraCalibration::composeCameraPara(Matrix_Type intrinsic, std::vector<Data_Type> distortion, std::vector<Matrix_Type> extrinsic, std::vector<Data_Type>& out)
{
	out.clear();
	//out.swap(std::vector<T>());

	out.push_back(intrinsic(0, 0)); //fx
	if (!m_bForceFs2zero)
		out.push_back(intrinsic(0, 1)); //fs
	out.push_back(intrinsic(1, 1)); //fy
	out.push_back(intrinsic(0, 2)); //ux
	out.push_back(intrinsic(1, 2)); //uy

	if (m_bk2)
	{
		out.push_back(distortion[0]); //k1
		out.push_back(distortion[1]); //k2
	}
	else
	{
		out.push_back(distortion[0]); //k1
		out.push_back(distortion[1]); //k2
		out.push_back(distortion[2]); //p1
		out.push_back(distortion[3]); //p2
		out.push_back(distortion[4]); //k3
	}

	for (int i = 0; i < extrinsic.size(); ++i)
	{
		Matrix_Type vec;
		rodriguesTransform(extrinsic[i].leftCols(3), vec);
		out.push_back(vec(0, 0));
		out.push_back(vec(1, 0));
		out.push_back(vec(2, 0));
		out.push_back(extrinsic[i](0, 3));
		out.push_back(extrinsic[i](1, 3));
		out.push_back(extrinsic[i](2, 3));
	}
}

void CMonocularCameraCalibration::decomposeCameraPara(std::vector<Data_Type>& para, Matrix_Type& intrinsic, std::vector<Data_Type>& distortion, std::vector<Matrix_Type>& extrinsic)
{
	int para_size = para.size();
	int midk = getIntrinsicsKOffset();
	if (para_size < midk) return;
	intrinsic = Matrix_Type::Identity(3, 3);

	int count = 0;

	if (m_bForceFs2zero)
	{
		intrinsic(0, 0) = para[0];
		intrinsic(1, 1) = para[1];
		intrinsic(0, 2) = para[2];
		intrinsic(1, 2) = para[3];

		distortion.clear();

		if (m_bk2)
		{
			distortion.push_back(para[4]);
			distortion.push_back(para[5]);
		}
		else
		{
			distortion.push_back(para[4]);
			distortion.push_back(para[5]);
			distortion.push_back(para[6]);
			distortion.push_back(para[7]);
			distortion.push_back(para[8]);
		}
		count = int((para_size - midk) / 6);
	}
	else
	{
		intrinsic(0, 0) = para[0];
		intrinsic(0, 1) = para[1];
		intrinsic(1, 1) = para[2];
		intrinsic(0, 2) = para[3];
		intrinsic(1, 2) = para[4];

		distortion.clear();
		if (m_bk2)
		{
			distortion.push_back(para[5]);
			distortion.push_back(para[6]);
		}
		else
		{
			distortion.push_back(para[5]);
			distortion.push_back(para[6]);
			distortion.push_back(para[7]);
			distortion.push_back(para[8]);
			distortion.push_back(para[9]);
		}
		count = int((para_size - midk) / 6);
	}

	extrinsic.clear();

	for (int i = 0; i < count; ++i)
	{
		Matrix_Type vec(3, 1);
		int idx = i * 6 + midk;
		vec << para[idx], para[idx + 1], para[idx + 2];
		Matrix_Type rot;
		rodriguesTransform(vec, rot);
		Matrix_Type temp_extrinsic = Matrix_Type::Zero(3, 4);
		temp_extrinsic.block(0, 0, 3, 3) = rot;
		temp_extrinsic(0, 3) = para[idx + 3];
		temp_extrinsic(1, 3) = para[idx + 4];
		temp_extrinsic(2, 3) = para[idx + 5];

		extrinsic.push_back(temp_extrinsic);
	}
}

int CMonocularCameraCalibration::getIntrinsicsKOffset()
{
	int midk = 0;
	if (m_bForceFs2zero)
	{
		if (m_bk2)
		{
			midk = 6;
		}
		else
		{
			midk = 9;
		}
	}
	else
	{
		if (m_bk2)
		{
			midk = 7;
		}
		else
		{
			midk = 10;
		}
	}
	return midk;
}

CameraCalibrationParas CMonocularCameraCalibration::getCameraCalibrateParas()
{
	CameraCalibrationParas paras;
	paras.intrinsicsMatrix = m_intrinsicsMatrix;
	paras.extrinsicsMatrixs = m_extrinsicsMatrixs;
	paras.distortionVec = m_distortionVec;
	return paras;
}