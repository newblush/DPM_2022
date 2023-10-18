#pragma once

#include <atomic>
#include <thread>
#include <opencv2/core.hpp>
#include "tracking_data.hpp"
#include "ring_buffer.hpp"
#include "HighSpeedProjector.h"
#include "ProjectorUtility.h"

class draw_opengl_shade
{
private:
	HighSpeedProjector &proj_v3;
	std::vector< tracking_data > &tdata_vector;
	ring_buffer< cv::Mat > const &image_buffer;
	std::thread thread;
	std::atomic< bool > exit_flag;
	std::size_t last_tracking_index;
	std::vector< std::vector< std::vector< float > > > const point_vector_vector;
	std::vector< std::vector< std::vector< float > > > const draw_point_vector_vector;
	std::vector< std::vector< std::vector< unsigned int > > > const draw_index_vector_vector;
	std::vector< std::vector< std::vector< float > > > const draw_normal_vector_vector;
	cv::Mat const cameraMatrix, cameraCoeffs;

	std::atomic< std::size_t > &objdata_index;


public:
	draw_opengl_shade(
		HighSpeedProjector &_proj,
		std::vector< tracking_data > &_tdata_vector,
		ring_buffer< cv::Mat > const &_image_buffer,
		std::vector< std::vector< std::vector< float > > > _point_vector_vector,
		std::vector< std::vector< std::vector< float > > > _draw_point_vector_vector,
		std::vector< std::vector< std::vector< unsigned int > > > _draw_index_vector_vector,
		std::vector< std::vector< std::vector< float > > > _draw_normal_vector_vector,
		cv::Mat _cameraMatrix,
		cv::Mat _cameraCoeffs,
		std::atomic< std::size_t > &_objdata_index
	)
		: proj_v3(_proj)
		, tdata_vector( _tdata_vector )
		, image_buffer( _image_buffer )
		, exit_flag( true )
		, last_tracking_index( 0u )
		, point_vector_vector( std::move( _point_vector_vector ) )
		, draw_point_vector_vector( std::move( _draw_point_vector_vector ) )
		, draw_index_vector_vector( std::move( _draw_index_vector_vector ) )
		, draw_normal_vector_vector( std::move( _draw_normal_vector_vector ) )
		, cameraMatrix( std::move( _cameraMatrix ) )
		, cameraCoeffs( std::move( _cameraCoeffs ) )
		, objdata_index( _objdata_index )
	{
		// assert( tdata.point.size() == tdata.point_size.size() );
		// assert( tdata.point.size() == point_vector.size() );
		// assert( tdata.point.size() == draw_point_vector.size() );
		// assert( tdata.point.size() == draw_index_vector.size() );
		// assert( tdata.point.size() == draw_normal_vector.size() );
	}

	void run( void )
	{
		exit_flag = false;
		thread = std::thread( &draw_opengl_shade::thread_func, this );
	}
	void stop( void )
	{
		exit_flag = true;
		thread.join();
	}
	bool is_running( void ) const
	{
		return !exit_flag;
	}

private:
	bool wait_update();
	void thread_func( void );
};
