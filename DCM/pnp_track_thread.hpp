#pragma once

#include <atomic>
#include <vector>
#include <tuple>
#include <thread>
#include <opencv2/core.hpp>
#include "tracking_data.hpp"
#include "ring_buffer.hpp"

class pnp_track_thread
{
private:
	tracking_data &tdata;
	ring_buffer< cv::Mat > const &image_buffer;
	std::size_t last_image_index;
	std::thread thread;
	std::atomic< bool > exit_flag;
	std::vector< std::vector< unsigned int > > const num_vector;
	std::vector< std::vector< float > > const point_vector;
	std::vector< std::vector< float > > const dcm_normal_vector;
	cv::Mat const cameraMatrix, cameraCoeffs;

public:
	pnp_track_thread(
		tracking_data &_tdata,
		ring_buffer< cv::Mat > const &_image_buffer,
		std::vector< std::vector< unsigned int > > _num_vector,
		std::vector< std::vector< float > > _point_vector,
		std::vector< std::vector< float > > _dcm_normal_vector,
		cv::Mat _cameraMatrix,
		cv::Mat _cameraCoeffs
	)
		: tdata( _tdata )
		, image_buffer( _image_buffer )
		, last_image_index( 0u )
		, exit_flag( true )
		, num_vector( std::move( _num_vector ) )
		, point_vector( std::move( _point_vector ) )
		, dcm_normal_vector( std::move( _dcm_normal_vector ) )
		, cameraMatrix( std::move( _cameraMatrix ) )
		, cameraCoeffs( std::move( _cameraCoeffs ) )
	{
		assert( num_vector.size() == point_vector.size() );
		assert( point_vector.size() == dcm_normal_vector.size() );
	}

	void run( void )
	{
		exit_flag = false;
		thread = std::thread( &pnp_track_thread::thread_func, this );
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
	std::atomic<bool> flag = false;
	cv::Mat solve_pnp_point_img;
private:
	std::size_t wait_tracking_update( void );
	void thread_func( void );
};