#pragma once

#include <atomic>
#include <thread>
#include <opencv2/core.hpp>
#include "tracking_data.hpp"
#include "ring_buffer.hpp"

class draw_thread
{
private:
	tracking_data &tdata;
	ring_buffer< cv::Mat > const &image_buffer;
	std::thread thread;
	std::atomic< bool > exit_flag;
	std::size_t last_tracking_index;
	std::vector< std::vector< float > > const point_vector;
	cv::Mat const cameraMatrix, cameraCoeffs;

public:
	draw_thread(
		tracking_data &_tdata,
		ring_buffer< cv::Mat > const &_image_buffer,
		std::vector< std::vector< float > > _point_vector,
		cv::Mat _cameraMatrix,
		cv::Mat _cameraCoeffs
	)
		: tdata( _tdata )
		, image_buffer( _image_buffer )
		, exit_flag( true )
		, last_tracking_index( 0u )
		, point_vector( std::move( _point_vector) )
		, cameraMatrix( std::move( _cameraMatrix ) )
		, cameraCoeffs( std::move( _cameraCoeffs ) )
	{
	    // do nothing
	}

	void run( void )
	{
		exit_flag = false;
		thread = std::thread( &draw_thread::thread_func, this );
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
	bool wait_update( void );
	void thread_func( void );
};
