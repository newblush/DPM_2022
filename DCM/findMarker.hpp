#pragma once

#include <opencv2/opencv.hpp>
#include <limits>
#include <thread>
#include <string>
#include "compier_attribute.hpp"

#define FIND_MARKER_DEBUG_SHOW 0

constexpr float E_MAX = 0.98f;

namespace
{
	bool key_point_from_contour(
		float &x,
		float &y,
		float &sx,
		float &sy,
		std::vector< cv::Point > const &contour,
		float const area_threshold_min = 0.0f,
		float const area_threshold_max = std::numeric_limits< float >::max(),
		bool const dbg_show = false
	)
	{
		// ここ遅そう
		// https://en.wikipedia.org/wiki/Image_moment#Central_moments
		auto const m = cv::moments( contour, true );
		auto const area = static_cast< float >( m.m00 );
		if( area < area_threshold_min || area_threshold_max < area )
		// if( area <= 0 )
		{
#if FIND_MARKER_DEBUG_SHOW
			if( dbg_show )
			{
				std::clog << "area drop: " << area << std::endl;
			}
#endif
			return false;
		};
		auto const x_bar = m.m10 / m.m00, y_bar = m.m01 / m.m00;
		auto const mu00 = m.m00;
		auto const mu20 = m.m20 - x_bar * m.m10;
		auto const mu02 = m.m02 - y_bar * m.m01;
		auto const mu11_dash = m.m11 / m.m00 - x_bar * y_bar;
		auto const mu20_dash = mu20 / mu00;
		auto const mu02_dash = mu02 / mu00;
		auto const mu20_dash_minus_mu02_dash = mu20_dash - mu02_dash;
		auto const lam_tmp1 = mu20_dash + mu02_dash;
		auto const lam_tmp2 = std::sqrt( 4 * mu11_dash * mu11_dash + mu20_dash_minus_mu02_dash * mu20_dash_minus_mu02_dash );
		auto const lam1_times_two = lam_tmp1 + lam_tmp2, lam2_times_two = lam_tmp1 - lam_tmp2;
		auto const squared_ecc = 1 - lam2_times_two / lam1_times_two;
		if( squared_ecc > E_MAX )
		{
#if FIND_MARKER_DEBUG_SHOW
		if( dbg_show )
		{
			std::clog << "ecc drop: " << squared_ecc << std::endl;
		}
#endif
			return false;
		}
		x = static_cast< float >( x_bar ), y = static_cast< float >( y_bar );
		// sx = std::sqrt( static_cast< float >( mu20 ) );
		// sy = std::sqrt( static_cast< float >( mu02 ) );
		sx = std::sqrt( static_cast< float >( mu20_dash ) );
		sy = std::sqrt( static_cast< float >( mu02_dash ) );
		return true;
	}
	bool findMarker(
		cv::Mat &img,
		float const center_x,
		float const center_y,
		float const roi_size_x,
		float const roi_size_y,
		unsigned int const expect_keypoint_num,
		float &new_center_x,
		float &new_center_y,
		float &new_sx,
		float &new_sy,
		unsigned int &new_keypoint_num,
		char const * const label,	// for debug
		bool const dbg_show = false
	)
	{
		auto const roi_size_x_half = roi_size_x / 2.0f, roi_size_y_half = roi_size_y / 2.0f;
		cv::Point2f roipt1( center_x - roi_size_x_half, center_y - roi_size_y_half );
		cv::Point2f roipt2( center_x + roi_size_x_half + 1.0f, center_y + roi_size_y_half + 1.0f );

		if( !(0 <= roipt1.x && 0 <= roipt1.y && roipt2.x <= img.cols && roipt2.y <= img.rows) ) return false;

		cv::Mat imgroi = img( cv::Rect( roipt1, roipt2 ) );

		cv::Mat imgroibin, fuck;
		cv::adaptiveThreshold( imgroi, imgroibin, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 9, 8 );
		cv::erode( imgroibin, fuck, cv::noArray() );
		// cv::dilate( fuck, fuck, cv::noArray() );

#if FIND_MARKER_DEBUG_SHOW
		if( dbg_show )
		{
			auto const &wname = std::string( "findMarker(" ) + label + ")";
			cv::Mat tmp( fuck.size() + cv::Size( 200, 200 ), CV_8UC3, cv::Scalar( 255, 0, 0 ) );
			cv::Mat fuck2;
			cv::cvtColor( fuck, fuck2, cv::COLOR_GRAY2BGR );
			cv::Mat mat = (cv::Mat_< double >( 2, 3 ) << 1.0, 0.0, 100, 0.0, 1.0, 100);
			cv::warpAffine( fuck2, tmp, mat, tmp.size(), cv::INTER_NEAREST, cv::BORDER_TRANSPARENT );
			cv::imshow( wname, tmp );
			cv::waitKey( 1 );
		}
#endif

		std::vector< std::vector< cv::Point2i > > contour_arr;
		cv::findContours( fuck, contour_arr, cv::noArray(), cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE );

#if FIND_MARKER_DEBUG_SHOW
		if( dbg_show )
		{
			auto const &wname = std::string( "findMarker_contour(" ) + label + ")";
			cv::Mat tmp( fuck.size() + cv::Size( 200, 200 ), CV_8UC3, cv::Scalar( 255, 0, 0 ) );
			cv::Mat fuck2;
			cv::cvtColor( fuck, fuck2, cv::COLOR_GRAY2BGR );
			cv::drawContours( fuck2, contour_arr, -1, cv::Scalar( 255, 255, 0 ) );
			cv::Mat mat = (cv::Mat_< double >( 2, 3 ) << 1.0, 0.0, 100, 0.0, 1.0, 100);
			cv::warpAffine( fuck2, tmp, mat, tmp.size(), cv::INTER_NEAREST, cv::BORDER_TRANSPARENT );
			cv::imshow( wname, tmp );
			cv::waitKey( 1 );
		}
#endif

		auto const contour_arr_size = std::size( contour_arr );
		auto kpn = 0u;
		auto nx = 0.0f, ny = 0.0f;
		auto nsx = 0.0f, nsy = 0.0f;
		for( auto i = 0u; i < contour_arr_size; ++i )
		{
			float x, y, sx, sy;
			auto const f = key_point_from_contour( x, y, sx, sy, contour_arr[ i ], 1.5f, std::numeric_limits< float >::max(), dbg_show );
			if( !f ) continue;
			nx += x, ny += y, nsx += sx, nsy += sy;
			++kpn;
		}
		if( !kpn ) return false;
		new_keypoint_num = kpn;
		new_center_x = roipt1.x + nx / kpn;
		new_center_y = roipt1.y + ny / kpn;
		new_sx = nsx / kpn;
		new_sy = nsy / kpn;
		return true;
	}
	// erodeにより標準偏差がずれるので，それを補正する．あと，量子化の際に1ぐらいずれるので，それも補正しようと思ったけどやめた
	FORCE_INLINE
	float correction_standard_deviation( float const s )
	{
		// return s + 0.5f + 1.0f;
		return s + 0.5f/* + 1.0f*/;
	}
} // namespace