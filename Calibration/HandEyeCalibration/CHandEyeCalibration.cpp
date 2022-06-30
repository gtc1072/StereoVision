#include "CHandEyeCalibration.h"

CHandEyeCalibration::CHandEyeCalibration()
{

}

CHandEyeCalibration::~CHandEyeCalibration()
{

}

bool CHandEyeCalibration::calibrate_eye_in_hand(const std::vector<Matrix_Type>& T_obj2cam, const std::vector<Matrix_Type>& T_grip2base, Matrix_Type& T_cam2grip, HandeyeCalibrateMethod method)
{
	size_t count = T_obj2cam.size();
	if (count > 2 && count == T_grip2base.size())
	{
		std::vector<Matrix_Type> As, Bs;
		for (int i = 0; i < count; ++i)
		{
			for (int j = i + 1; j < count; ++j)
			{
				As.push_back(T_obj2cam[j].inverse() * T_obj2cam[i]);
				Bs.push_back(T_grip2base[j] * T_grip2base[i].inverse());
			}
		}
		bool ret = false;
		switch (method)
		{
		case TSAI_LENZ:
			ret = calibrateTsaiLenz(As, Bs, T_cam2grip);
			break;
		case DANIILIDIS:
			ret = calibrateDaniilidis(As, Bs, T_cam2grip);
			break;
		default:
			ret = calibrateTsaiLenz(As, Bs, T_cam2grip);
			break;
		}
		return ret;
	}
	return false;
}

bool CHandEyeCalibration::calibrate_eye_out_hand(const std::vector<Matrix_Type>& T_obj2cam, const std::vector<Matrix_Type>& T_base2grip, Matrix_Type& T_cam2base, HandeyeCalibrateMethod method)
{
	size_t count = T_obj2cam.size();
	if (count > 2 && count == T_obj2cam.size())
	{
		std::vector<Matrix_Type> As, Bs;
		for (int i = 0; i < count; ++i)
		{
			for (int j = i + 1; j < count; ++j)
			{
				As.push_back(T_obj2cam[j].inverse() * T_obj2cam[i]);
				Bs.push_back(T_base2grip[j] * T_base2grip[i].inverse());
			}
		}
		bool ret = false;
		switch (method)
		{
		case TSAI_LENZ:
			ret = calibrateTsaiLenz(As, Bs, T_cam2base);
			break;
		case DANIILIDIS:
			ret = calibrateDaniilidis(As, Bs, T_cam2base);
			break;
		default:
			ret = calibrateTsaiLenz(As, Bs, T_cam2base);
			break;
		}
		return ret;
	}
	return false;
}

bool CHandEyeCalibration::calibrateDaniilidis(const std::vector<Matrix_Type>& As, const std::vector<Matrix_Type>& Bs, Matrix_Type& X)
{
	size_t count = As.size();
	if (count == Bs.size() && count > 0)
	{
		X = Matrix_Type::Identity(4, 4);
		Matrix_Type T = Matrix_Type::Zero(count * 6, 8);
		Matrix_Type a = Matrix_Type::Zero(3, 1);
		Matrix_Type b = Matrix_Type::Zero(3, 1);
		Matrix_Type ap = Matrix_Type::Zero(3, 1);
		Matrix_Type bp = Matrix_Type::Zero(3, 1);
		Dualqual_Type dqa, dqb;
		Matrix_Type cl, cr;
		for (size_t i = 0; i < count; ++i)
		{
			CQuatTransform::rigid2dualquternionTransform(As[i], dqa);
			CQuatTransform::rigid2dualquternionTransform(Bs[i], dqb);
			a << dqa.qr(1) - dqb.qr(1), dqa.qr(2) - dqb.qr(2), dqa.qr(3) - dqb.qr(3);
			b << dqa.qr(1) + dqb.qr(1), dqa.qr(2) + dqb.qr(2), dqa.qr(3) + dqb.qr(3);
			ap << dqa.qe(1) - dqb.qe(1), dqa.qe(2) - dqb.qe(2), dqa.qe(3) - dqb.qe(3);
			bp << dqa.qe(1) + dqb.qe(1), dqa.qe(2) + dqb.qe(2), dqa.qe(3) + dqb.qe(3);
			CCommonTransform::skewTransform(b, cl);
			CCommonTransform::skewTransform(bp, cr);
			T.block(i * 6, 0, 3, 1) = a;
			T.block(i * 6, 1, 3, 3) = cl;
			T.block(i * 6 + 3, 0, 3, 1) = ap;
			T.block(i * 6 + 3, 1, 3, 3) = cr;
			T.block(i * 6 + 3, 4, 3, 1) = a;
			T.block(i * 6 + 3, 5, 3, 3) = cl;
		}
		Eigen::JacobiSVD<Matrix_Type> svd(T, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::VectorXd sgv = svd.singularValues();
		if (std::fabs(sgv(sgv.size() - 1)) < 1.0e-2 && std::fabs(sgv(sgv.size() - 2)) < 1.0e-2)
		{
			Matrix_Type v = svd.matrixV();
			Quat_Type u1, v1, u2, v2;
			u1 << v(0, 6), v(1, 6), v(2, 6), v(3, 6);
			v1 << v(4, 6), v(5, 6), v(6, 6), v(7, 6);
			u2 << v(0, 7), v(1, 7), v(2, 7), v(3, 7);
			v2 << v(4, 7), v(5, 7), v(6, 7), v(7, 7);
			double va = u1.dot(v1);
			double vb = u1.dot(v2) + u2.dot(v1);
			double vc = u2.dot(v2);
			double vd = u1.dot(u1);
			double ve = u1.dot(u2);
			double vf = u2.dot(u2);
			double s1 = (-vb + sqrt(vb * vb - 4.0 * va * vc)) / (2.0 * va);
			double s2 = (-vb - sqrt(vb * vb - 4.0 * va * vc)) / (2.0 * va);
			double phi1 = s1 * s1 * vd + 2.0 * s1 * ve + vf;
			double phi2 = s2 * s2 * vd + 2.0 * s2 * ve + vf;
			double lamda1, lamda2;
			if (phi1 > phi2)
			{
				lamda2 = 1.0 / sqrt(phi1);
				lamda1 = s1 * lamda2;
			}
			else
			{
				lamda2 = 1.0 / sqrt(phi2);
				lamda1 = s2 * lamda2;
			}
			Dualqual_Type dqrst;
			dqrst.qr = lamda1 * u1 + lamda2 * u2;
			dqrst.qe = lamda2 * v1 + lamda2 * v2;
			CQuatTransform::dualquternion2rigidTransform(dqrst, X);
			return true;
		}
	}
	return false;
}

bool CHandEyeCalibration::calibrateTsaiLenz(const std::vector<Matrix_Type>& As, const std::vector<Matrix_Type>& Bs, Matrix_Type& X)
{
	size_t count = As.size();
	if (count == Bs.size() && count > 0)
	{
		X = Matrix_Type::Identity(4, 4);
		Matrix_Type A = Matrix_Type::Zero(count * 3, 3);
		Matrix_Type B = Matrix_Type::Zero(count * 3, 1);
		Matrix_Type Pa, Pb, S;
		for (size_t i = 0; i < count; ++i)
		{
			CCommonTransform::rodriguesTransform(As[i], Pa);
			CCommonTransform::rodriguesTransform(Bs[i], Pb);
			double theta_a = Pa.norm();
			double theta_b = Pb.norm();
			Pa = Pa / theta_a;
			Pb = Pb / theta_b;
			Pa = 2.0 * sin(theta_a / 2.0) * Pa;
			Pb = 2.0 * sin(theta_b / 2.0) * Pb;
			CCommonTransform::skewTransform(Pa + Pb, S);
			A.block(i * 3, 0, 3, 3) = S;
			B.block(i * 3, 0, 3, 1) = Pb - Pa;
		}
		Matrix_Type Pp = A.colPivHouseholderQr().solve(B);
		Matrix_Type Pp_norm = Pp.transpose() * Pp;
		Pp = 2.0 * Pp / sqrt(1.0 + Pp_norm(0, 0));
		CCommonTransform::skewTransform(Pp, S);
		Matrix_Type R = (1.0 - Pp_norm(0, 0) / 2) * Matrix_Type::Identity(3, 3) + 0.5 * (Pp * Pp.transpose() + sqrt(4.0 - Pp_norm(0, 0)) * S);
		A = Matrix_Type::Zero(count * 3, 3);
		B = Matrix_Type::Zero(count * 3, 1);
		for (size_t i = 0; i < count; ++i)
		{
			A.block(i * 3, 0, 3, 3) = As[i].block(0,0,3,3)- Matrix_Type::Identity(3, 3);
			B.block(i * 3, 0, 3, 1) = R * Bs[i].block(0, 3, 3, 1) - As[i].block(0, 3, 3, 1);
		}
		Matrix_Type T = A.colPivHouseholderQr().solve(B);
		X.block(0, 0, 3, 3) = R;
		X.block(0, 3, 3, 1) = T;
		return true;
	}
	return false;
}