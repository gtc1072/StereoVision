#include "CStereoCameraRectification.h"

CStereoCameraRectification::CStereoCameraRectification()
{

}

CStereoCameraRectification::~CStereoCameraRectification()
{

}

bool CStereoCameraRectification::doStereoRectify(Matrix_Type left_intrinsic, std::vector<Data_Type> left_distort,
	Matrix_Type right_intrinsic, std::vector<Data_Type> right_distort,
	int image_width, int image_height,
	Matrix_Type rotation_mat, Matrix_Type translation_vector,
	Matrix_Type& left_proj, Matrix_Type& right_proj, Matrix_Type& Q)
{
	bool ret = false;
	Matrix_Type eul, halfRot, halfRotPeer, rect;
	rotation_mat = rotation_mat.inverse();
	rodriguesTransform(rotation_mat, eul);
	eul = eul * (-0.5);
	rodriguesTransform(eul, halfRot);
	halfRotPeer = halfRot * rotation_mat;
	translation_vector = -halfRotPeer * translation_vector;
	Vetor3_Type e1, e2, e3;
	e1 << translation_vector(0, 0), translation_vector(1, 0), translation_vector(2, 0);
	e1 = e1 / e1.norm();
	e2 << -translation_vector(1, 0), translation_vector(0, 0), 0;
	e2 = e2 / sqrt(translation_vector(0, 0) * translation_vector(0, 0) + translation_vector(1, 0) * translation_vector(1, 0));
	e3 = e1.cross(e2);
	rect = Matrix_Type::Zero(3, 3);
	rect(0, 0) = e1(0); rect(0, 1) = e1(1); rect(0, 2) = e1(2);
	rect(1, 0) = e2(0); rect(1, 1) = e2(1); rect(1, 2) = e2(2);
	rect(2, 0) = e3(0); rect(2, 1) = e3(1); rect(2, 2) = e3(2);
	left_proj = rect * halfRot;
	right_proj = rect * halfRotPeer;
	Data_Type Tx = (rect * translation_vector)(0, 0);
	Matrix_Type newK(3, 3);
	newK(0, 0) = (left_intrinsic(0, 0) + left_intrinsic(1, 1) + right_intrinsic(0, 0) + right_intrinsic(1, 1)) * 0.25;
	newK(0, 1) = 0;
	newK(0, 2) = (left_intrinsic(0, 2) + right_intrinsic(0, 2)) * 0.5;
	newK(1, 0) = 0;
	newK(1, 1) = newK(0, 0);
	newK(1, 2) = (left_intrinsic(1, 2) + right_intrinsic(1, 2)) * 0.5;
	newK(2, 0) = 0; newK(2, 1) = 0; newK(2, 2) = 1;
	left_proj = newK * left_proj;
	right_proj = newK * right_proj;
	Q = Matrix_Type::Identity(4, 4);
	Q(0, 3) = -newK(0, 2);
	Q(1, 3) = -newK(1, 2);
	Q(2, 2) = 0;
	Q(2, 3) = newK(0, 0);
	Q(3, 2) = -1.0 / Tx;
	Q(3, 3) = 0;
	return true;
}

bool CStereoCameraRectification::doImageRectify(int imageWidth, int imageHeight,
	Data_Type** pXmap_left, Data_Type** pYmap_left,
	Data_Type** pXmap_right, Data_Type** pYmap_right,
	Matrix_Type left_intrinsic, std::vector<Data_Type> left_distort,
	Matrix_Type right_intrinsic, std::vector<Data_Type> right_distort,
	Matrix_Type left_proj, Matrix_Type right_proj)
{
	bool ret = false;
	if (imageWidth <= 0 || imageHeight <= 0 || *pXmap_left || *pYmap_left || *pXmap_right || *pYmap_right) return ret;
	if (left_distort.size() != 2 || left_distort.size() != 5 || left_distort.size() != right_distort.size()) return ret;
	*pXmap_left = new Data_Type[imageWidth * imageHeight];
	*pYmap_left = new Data_Type[imageWidth * imageHeight];
	*pXmap_right = new Data_Type[imageWidth * imageHeight];
	*pYmap_right = new Data_Type[imageWidth * imageHeight];
	Matrix_Type point = Matrix_Type::Ones(3, 1);
	Matrix_Type temp;
	Data_Type nx, ny, r, xe, ye, tx, ty;
	for (int j = 0; j < imageHeight; ++j)
	{
		for (int i = 0; i < imageWidth; ++i)
		{
			point(0, 0) = i; point(1, 0) = j;
			temp = left_proj.inverse() * point;
			nx = temp(0, 0) / temp(2, 0);
			ny = temp(1, 0) / temp(2, 0);
			r = nx * nx + ny * ny;
			if (left_distort.size() == 2)
			{
				xe = nx * (1.0 + left_distort[0] * r + left_distort[1] * r * r);
				ye = ny * (1.0 + left_distort[0] * r + left_distort[1] * r * r);
			}
			else
			{
				xe = nx * (1.0 + left_distort[0] * r + left_distort[1] * r * r + left_distort[4] * r * r * r) 
					+ 2.0 * left_distort[2] * nx * ny + left_distort[3] * (r * r + 2.0 * nx * nx);
				ye = ny * (1.0 + left_distort[0] * r + left_distort[1] * r * r + left_distort[4] * r * r * r)
					+ 2.0 * left_distort[3] * nx * ny + left_distort[2] * (r * r + 2.0 * ny * ny);
			}
			tx = left_intrinsic(0, 0) * xe + left_intrinsic(0, 1) * ye + left_intrinsic(0, 2);
			ty = left_intrinsic(1, 1) * ye + left_intrinsic(1, 2);
			if (tx >= 0 && ty >= 0 && tx <= imageWidth - 1 && ty <= imageHeight - 1)
			{
				*pXmap_left[j * imageWidth + i] = tx;
				*pYmap_left[j * imageWidth + i] = ty;
			}
			else
			{
				tx = ty = INVALID_MAP;
			}
			temp = right_proj.inverse() * point;
			nx = temp(0, 0) / temp(2, 0);
			ny = temp(1, 0) / temp(2, 0);
			r = nx * nx + ny * ny;
			if (right_distort.size() == 2)
			{
				xe = nx * (1.0 + right_distort[0] * r + right_distort[1] * r * r);
				ye = ny * (1.0 + right_distort[0] * r + right_distort[1] * r * r);
			}
			else
			{
				xe = nx * (1.0 + right_distort[0] * r + right_distort[1] * r * r + right_distort[4] * r * r * r)
					+ 2.0 * right_distort[2] * nx * ny + right_distort[3] * (r * r + 2.0 * nx * nx);
				ye = ny * (1.0 + right_distort[0] * r + right_distort[1] * r * r + right_distort[4] * r * r * r)
					+ 2.0 * right_distort[3] * nx * ny + right_distort[2] * (r * r + 2.0 * ny * ny);
			}
			tx = right_intrinsic(0, 0) * xe + right_intrinsic(0, 1) * ye + right_intrinsic(0, 2);
			ty = right_intrinsic(1, 1) * ye + right_intrinsic(1, 2);
			if (tx >= 0 && ty >= 0 && tx <= imageWidth - 1 && ty <= imageHeight - 1)
			{
				*pXmap_right[j * imageWidth + i] = tx;
				*pYmap_right[j * imageWidth + i] = ty;
			}
			else
			{
				tx = ty = INVALID_MAP;
			}
		}
	}
	return ret;
}

void CStereoCameraRectification::rodriguesTransform(Matrix_Type src, Matrix_Type& dst)
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