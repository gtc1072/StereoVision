#pragma once
#include "Utils.hpp"

class CCommonTransform
{
public:
	static bool rodriguesTransform(Matrix_Type src, Matrix_Type& dst)
	{
		bool ret = true;

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
		else
		{
			ret = false;
			std::cout << "rodriguesTransform -> Unsupported Matrix Size." << std::endl;
		}

		return ret;
	}

	static bool skewTransform(Matrix_Type src, Matrix_Type& dst)
	{
		bool ret = false;

		if ((src.rows() == 1 && src.cols() == 3) || (src.cols() == 1 && src.rows() == 3))
		{
			dst = Matrix_Type::Zero(3, 3);

			if (src.rows() == 1)
			{
				dst(0, 1) = -src(0, 2);
				dst(0, 2) = src(0, 1);

				dst(1, 0) = src(0, 2);
				dst(1, 2) = -src(0, 0);

				dst(2, 0) = -src(0, 1);
				dst(2, 1) = src(0, 0);
			}
			else
			{
				dst(0, 1) = -src(2, 0);
				dst(0, 2) = src(1, 0);

				dst(1, 0) = src(2, 0);
				dst(1, 2) = -src(0, 0);

				dst(2, 0) = -src(1, 0);
				dst(2, 1) = src(0, 0);
			}

			ret = true;
		}
		return ret;
	}
};

class CQuatTransform
{
public:
	static bool rotation2quaternionTransform(Matrix_Type rot, Quat_Type& quat)
	{
		bool ret = false;
		if (rot.cols() == 3 && rot.rows() == 3)
		{
			Data_Type m00 = rot(0, 0); Data_Type m01 = rot(0, 1); Data_Type m02 = rot(0, 2);
			Data_Type m10 = rot(1, 0); Data_Type m11 = rot(1, 1); Data_Type m12 = rot(1, 2);
			Data_Type m20 = rot(2, 0); Data_Type m21 = rot(2, 1); Data_Type m22 = rot(2, 2);
			Data_Type trace = m00 + m11 + m22;
			if (trace > 0)
			{
				Data_Type s = sqrt(trace + 1.0) * 2;
				quat(0) = 0.25 * s;
				quat(1) = (m21 - m12) / s;
				quat(2) = (m02 - m20) / s;
				quat(3) = (m10 - m01) / s;
			}
			else if ((m00 > m11) && (m00 > m22))
			{
				Data_Type s = sqrt(m00 - m11 - m22 + 1.0) * 2;
				quat(0) = (m21 - m12) / s;
				quat(1) = 0.25 * s;
				quat(2) = (m01 + m10) / s;
				quat(3) = (m02 + m20) / s;
			}
			else if (m11 > m22)
			{
				Data_Type s = sqrt(m11 - m00 - m22 + 1.0) * 2;
				quat(0) = (m02 - m20) / s;
				quat(1) = (m01 + m10) / s;
				quat(2) = 0.25 * s;
				quat(3) = (m21 + m12) / s;
			}
			else
			{
				Data_Type s = sqrt(m22 - m00 - m11 + 1.0) * 2;
				quat(0) = (m10 - m01) / s;
				quat(1) = (m02 + m20) / s;
				quat(2) = (m12 + m21) / s;
				quat(3) = 0.25 * s;
			}
			ret = true;
		}
		return ret;
	}

	static bool quaternion2rotationTransform(Quat_Type quat, Matrix_Type& rot)
	{
		Data_Type qw = quat(0); Data_Type qx = quat(1); Data_Type qy = quat(2); Data_Type qz = quat(3);
		rot = Matrix_Type::Zero(3, 3);

		rot(0, 0) = 1.0 - 2.0 * qy * qy - 2.0 * qz * qz;
		rot(0, 1) = 2.0 * qx * qy - 2.0 * qw * qz;
		rot(0, 2) = 2.0 * qx * qz + 2.0 * qy * qw;

		rot(1, 0) = 2.0 * qx * qy + 2.0 * qw * qz;
		rot(1, 1) = 1.0 - 2.0 * qx * qx - 2.0 * qz * qz;
		rot(1, 2) = 2.0 * qy * qz - 2.0 * qx * qw;

		rot(2, 0) = 2.0 * qx * qz - 2.0 * qw * qy;
		rot(2, 1) = 2.0 * qy * qz + 2.0 * qx * qw;
		rot(2, 2) = 1.0 - 2.0 * qx * qx - 2.0 * qy * qy;

		return true;
	}

	static bool rigid2dualquternionTransform(Matrix_Type rigid, Dualqual_Type& dualquat)
	{
		bool ret = false;
		if (rigid.rows() == 4 && rigid.cols() == 4)
		{
			ret = rotation2quaternionTransform(rigid.block(0, 0, 3, 3), dualquat.qr);
			if (ret)
			{
				Quat_Type tp;
				tp(0) = 0; tp(1) = rigid(0, 3); tp(2) = rigid(1, 3); tp(3) = rigid(2, 3);
				dualquat.qe = 0.5 * quaternionmultiply(tp, dualquat.qr);
				ret = true;
			}
		}
		return ret;
	}

	static bool dualquternion2rigidTransform(Dualqual_Type dualquat, Matrix_Type& rigid)
	{
		bool ret = false;
		rigid = Matrix_Type::Zero(4, 4);
		Matrix_Type rot;
		ret = quaternion2rotationTransform(dualquat.qr, rot);
		if (ret)
		{
			dualquat.qr(1) = -dualquat.qr(1);
			dualquat.qr(2) = -dualquat.qr(2);
			dualquat.qr(3) = -dualquat.qr(3);
			Quat_Type t = quaternionmultiply(dualquat.qe, dualquat.qr) * 2.0;
			rigid.block(0, 0, 3, 3) = rot;
			rigid(0, 3) = t(1); rigid(1, 3) = t(2); rigid(2, 3) = t(3);
		}
		return ret;
	}

	static Quat_Type quaternionmultiply(Quat_Type quat1, Quat_Type quat2)
	{
		Quat_Type quat;
		quat(0) = quat1(0) * quat2(0) - quat1(1) * quat2(1) - quat1(2) * quat2(2) - quat1(3) * quat2(3);
		quat(1) = quat1(0) * quat2(1) + quat1(1) * quat2(0) + quat1(2) * quat2(3) - quat1(3) * quat2(2);
		quat(2) = quat1(0) * quat2(2) - quat1(1) * quat2(3) + quat1(2) * quat2(0) + quat1(3) * quat2(1);
		quat(3) = quat1(0) * quat2(3) + quat1(1) * quat2(2) - quat1(2) * quat2(1) + quat1(3) * quat2(0);
		return quat;
	}

	static Dualqual_Type dualquaternionmultiply(Dualqual_Type dquat1, Dualqual_Type dquat2)
	{
		Dualqual_Type dquat;
		dquat.qr = quaternionmultiply(dquat1.qr, dquat2.qr);
		dquat.qe = quaternionmultiply(dquat1.qr, dquat2.qe) + quaternionmultiply(dquat1.qe, dquat2.qr);
		return dquat;
	}
};