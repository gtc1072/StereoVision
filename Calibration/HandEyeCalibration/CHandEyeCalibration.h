#pragma once
#include "Misc.hpp"

class CHandEyeCalibration
{
public:
	CHandEyeCalibration();
	~CHandEyeCalibration();
	bool calibrate_eye_in_hand(const std::vector<Matrix_Type>& T_obj2cam, const std::vector<Matrix_Type>& T_grip2base, Matrix_Type& T_cam2grip, HandeyeCalibrateMethod method = TSAI_LENZ);
	bool calibrate_eye_out_hand(const std::vector<Matrix_Type>& T_obj2cam, const std::vector<Matrix_Type>& T_base2grip, Matrix_Type& T_cam2base, HandeyeCalibrateMethod method = TSAI_LENZ);
private:
	bool calibrateDaniilidis(const std::vector<Matrix_Type>& As, const std::vector<Matrix_Type>& Bs, Matrix_Type& X);
	bool calibrateTsaiLenz(const std::vector<Matrix_Type>& As, const std::vector<Matrix_Type>& Bs, Matrix_Type& X);
};