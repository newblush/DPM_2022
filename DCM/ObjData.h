#pragma once

#include <vector>
#include <opencv2/core.hpp>

using fv = std::vector< float >;
using uiv = std::vector< unsigned int >;
using DCMObject = std::tuple< fv, fv, fv, fv, uiv, uiv, uiv, cv::Mat, cv::Mat >;
enum DcmMDATAIndex
{
	DPOINT = 0, DCM_NORMAL, DRAW_POINT, DRAW_NORMAL, INDEX, NUM, DRAW_INDEX, CAMERA_MATRIX, CAMERA_COEFFS
};