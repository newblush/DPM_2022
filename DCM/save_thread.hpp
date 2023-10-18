#pragma once

#include <atomic>
#include <vector>
#include <tuple>
#include <thread>
#include <opencv2/core.hpp>
#include "tracking_data.hpp"
#include "ring_buffer.hpp"

class save_thread
{
private:
	tracking_data &tdata;
	ring_buffer< cv::Mat > const &image_buffer;
	std::size_t last_image_index;
	std::thread thread;
	std::atomic< bool > exit_flag;
	// std::get< 0 >( cluster_data ) は index の配列, std::get< 1 >( cluster_data ) は num の配列
	std::vector< std::tuple< std::vector< unsigned int >, std::vector< unsigned int > > > const cluster_data;
	// 上にまとめるべきなんだろうけど，他のクラスとの兼ね合い上取り敢えずこんな感じ
	std::vector< std::vector< float > > const cluster_point;
	cv::Mat cameraMatrix, cameraCoeffs;

public:
	save_thread( tracking_data &_tdata, ring_buffer< cv::Mat > const &_image_buffer, std::vector< std::tuple< std::vector< unsigned int >, std::vector< unsigned int > > > _cluster_data, std::vector< std::vector< float > > _cluster_point, cv::Mat _cameraMatrix, cv::Mat _cameraCoeffs )
		: tdata( _tdata )
		, image_buffer( _image_buffer )
		, last_image_index( 0u )
		, exit_flag( true )
		, cluster_data( std::move( _cluster_data ) )
		, cluster_point( std::move( _cluster_point ) )
		, cameraMatrix( std::move( _cameraMatrix ) )
		, cameraCoeffs( std::move( _cameraCoeffs ) )
	{
		assert( cluster_data.size() == cluster_point.size() );
	}

	void run( void )
	{
		exit_flag = false;
		thread = std::thread( &save_thread::thread_func, this );
	}
	void stop( void )
	{
		exit_flag = true;
		if( thread.joinable() )
		{
			thread.join();
		}
	}
	bool is_running( void ) const
	{
		return thread.joinable();
	}

private:
	std::size_t wait_tracking_update( void );
	void thread_func( void );
};