#pragma once

#include <thread>
#include <opencv2/opencv.hpp>
#include "tracking_data.hpp"
#include "ring_buffer.hpp"

class tracking_thread
{
private:
	tracking_data &tdata;
	ring_buffer< cv::Mat > const &image_buffer;
	std::size_t last_image_index;
	std::thread thread;
	std::atomic< bool > exit_flag;
	std::vector< std::vector< unsigned int > > const num_vector;

public:
	tracking_thread(
		tracking_data &_tdata,
		ring_buffer< cv::Mat > const &_image_buffer,
		std::vector< std::vector< unsigned int > > _num_vector
	)
		: tdata( _tdata )
		, image_buffer( _image_buffer )
		, last_image_index( _image_buffer.current_index() )
		, exit_flag( true )
		, num_vector( std::move( _num_vector ) )
	{
		// do nothing
	}
	void run( void )
	{
		exit_flag = false;
		thread = std::thread( &tracking_thread::thread_func, this );
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
	std::size_t wait_next_image( void );
	void thread_func( void );
};
