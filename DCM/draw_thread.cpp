#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <iomanip>
#include "draw_thread.hpp"
#include "interval_timer.hpp"

#define DRAW_THREAD_OUTPUT_PNP_TIME 0
#if DRAW_THREAD_OUTPUT_PNP_TIME
#include <fstream>
constexpr auto PNP_TIME_OUTPUT_FRAME_COUNT = 100u;
#endif

bool draw_thread::wait_update( void )
{
	while( last_tracking_index == tdata.tracking_frame.load() )
	{
		if( exit_flag.load() ) return false;
		// std::this_thread::yield();
	}
	return true;
}

void draw_thread::thread_func( void )
{
	if( !wait_update() ) return;

	interval_timer calc_fps( "draw_thread" );

	auto const POINT_NUM = std::accumulate( tdata.point_size.begin(), tdata.point_size.end(), 0u );

	std::vector< cv::Point3f > objpoint;
	std::vector< cv::Point2f > imagepoint;
	std::vector< unsigned int > objid;
	objpoint.reserve( POINT_NUM );
	imagepoint.reserve( POINT_NUM );

#if DRAW_THREAD_OUTPUT_PNP_TIME
	std::vector< double > pnp_time_vector( PNP_TIME_OUTPUT_FRAME_COUNT );
	std::vector< unsigned int > pnp_time_point_vector( PNP_TIME_OUTPUT_FRAME_COUNT );
	auto pnp_time_vector_index = 0u;
#endif

	std::vector< cv::Point3f > allPoint;
	for( auto i = 0u; i < POINT_NUM; ++i )
	{
		auto const * const pp = &point_vector[ 0 ][ i * 3 ];
		allPoint.emplace_back( pp[ 0 ], pp[ 1 ], pp[ 2 ] );
	}

	unsigned int last_used_point_num = 0u;

	auto const cluster_data_size = tdata.point_size.size();
	assert( cluster_data_size == 1u );

	cv::Mat last_image;
	std::size_t draw_frame_num = 0u;
	while( !exit_flag.load() )
	{
		objpoint.clear();
		imagepoint.clear();
		objid.clear();

		if( !wait_update() ) break;
#if DRAW_THREAD_OUTPUT_PNP_TIME
		auto const st = std::chrono::high_resolution_clock::now();
#endif
		last_tracking_index = tdata.tracking_frame.load();

		tracking_data_point_get_tracking_points( tdata.point[ 0 ].get(), tdata.point_size[ 0 ], objid, imagepoint );
		for( auto const id : objid )
		{
			auto const * const pp = &point_vector[ 0 ][ id * 3 ];
			objpoint.emplace_back( pp[ 0 ], pp[ 1 ], pp[ 2 ] );
		}
		/*

		cv::Mat imgroibin, fuck;
		cv::adaptiveThreshold( crrimg, imgroibin, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 9, 8 );
		cv::erode( imgroibin, fuck, cv::noArray() );
		cv::imshow( "fuck", fuck );
		*/

		bool const useGuess = last_used_point_num >= 6;
		auto const objpoint_size = objpoint.size();
		last_used_point_num = static_cast< unsigned int >( objpoint_size );
		// if( objpoint_size ) std::cout << objpoint_size << std::endl;
		if( objpoint_size >= 3 ) 
		{
			cv::Mat rvec, tvec;
			std::vector< cv::Point2f > projectedPoint;
			bool const solvePnP_ret = cv::solvePnP( objpoint, imagepoint, cameraMatrix, cameraCoeffs, rvec, tvec, false );
#if DRAW_THREAD_OUTPUT_PNP_TIME
			auto const et = std::chrono::high_resolution_clock::now();
			auto const du = std::chrono::duration_cast< std::chrono::duration< double, std::milli > >( et - st ).count();
			pnp_time_vector[ pnp_time_vector_index % pnp_time_vector.size() ] = du;
			pnp_time_point_vector[ pnp_time_vector_index % pnp_time_vector.size() ] = static_cast< unsigned int >( objpoint_size );
			++pnp_time_vector_index;
#endif
			cv::projectPoints( allPoint, rvec, tvec, cameraMatrix, cameraCoeffs, projectedPoint );
			auto crrimg = image_buffer.get_with_index( last_tracking_index );
			cv::cvtColor( crrimg, crrimg, cv::COLOR_GRAY2BGR );
			for( auto const &v : projectedPoint )
			{
				cv::circle( crrimg, v, 5, cv::Scalar( 255, 255, 0 ), -1 );
			}
			for( auto i = 0u; i < imagepoint.size(); ++i )
			{
				auto const &v = imagepoint[ i ];
				cv::circle( crrimg, v, 5, cv::Scalar( 0, 255, 255 ), -1 );
				cv::putText( crrimg, std::to_string( objid[ i ] ), v, cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar( 255, 0, 0 ) );
			}
			if( !solvePnP_ret )
			{
				std::cout << tvec << std::endl;
			}
			last_image = crrimg;
			cv::imshow( "draw", crrimg );
		}
		else
		{
			auto crrimg = image_buffer.get_with_index( last_tracking_index );
			last_image = crrimg.clone();
			cv::imshow( "draw", crrimg );
		}
		/*
		if( draw_frame_num % 10 == 0 )
		{
			std::ostringstream oss;
			oss << "!draw_tmp_" << std::setw( 4 ) << std::setfill( '0' ) << (draw_frame_num / 10) % 10 << ".png";
			cv::imwrite( oss.str(), last_image );
		}
		*/
		cv::waitKey( 33 );

		draw_frame_num++;
		// calc_fps.interval();
	}
	cv::destroyWindow( "draw" );
	cv::imwrite( "!draw.png", last_image );

#if DRAW_THREAD_OUTPUT_PNP_TIME
	std::ofstream ofs( "pnp_time.csv" );
	for( auto i = 0; i < pnp_time_vector.size(); ++i )
	{
		ofs
			<< pnp_time_vector[ (i + pnp_time_vector_index) % pnp_time_point_vector.size() ] << ","
			<< pnp_time_point_vector[ (i + pnp_time_vector_index) % pnp_time_point_vector.size() ] << std::endl;
	}
#endif
}
