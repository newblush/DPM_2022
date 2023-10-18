#pragma once

#include <thread>
#include <opencv2/opencv.hpp>
#include <limits>
#include <chrono>
#include "tracking_data.hpp"
#include "ring_buffer.hpp"
#include "recognition_thread.hpp"
#include "recognition.hpp"
#include "tracking_data.hpp"
#include "interval_timer.hpp"
#include "meta_programming.hpp"

// 設定：ガチで動かす時は全て0にすべき
#define RCOGNITION_THREAD_DEBUG_SHOW 0
#define RCOGNITION_THREAD_DEBUG_SHOW_SAVE 0
// 十分点があってもRCOGNITION_THREADを動かし続ける
#define RCOGNITION_THREAD_NON_STOP_MODE 0
#define RCOGNITION_THREAD_SHOW_PROCESS_TIME 0

#if RCOGNITION_THREAD_SHOW_PROCESS_TIME
#include <fstream>
#endif

using namespace recognition;

constexpr float AREA_THRESHOLD_MIN = 1.5f;
constexpr float AREA_THRESHOLD_MAX = std::numeric_limits< float >::max();
constexpr std::size_t KEYPOINT_GRID_NUM_THRESHOLD = 15;
constexpr float KEYPOINT_DISTANCE_THRESHOLD = 15.0f;
constexpr unsigned int MAX_NUM_OF_KEY_POINT_IN_CLUSTER = 6; // 本来は定数で与えられる値ではない．
constexpr unsigned int COUNT_THRESHOLD = 4u;

constexpr std::size_t ABORT_WAIT_INDEX = std::numeric_limits< std::size_t >::max();

template< typename... Recognizer >
class recognition_thread
{
private:
	tracking_data &tdata;
	ring_buffer< cv::Mat > const &image_buffer;
	std::size_t last_image_index;
	std::thread thread;
	std::atomic< bool > exit_flag;

	std::tuple< Recognizer... > recognizer_tuple;
	meta::unique_tuple_t< typename Recognizer::prefilter... > prefilter_tuple;

public:
	recognition_thread( tracking_data &_tdata, ring_buffer< cv::Mat > const &_image_buffer, Recognizer... args )
		: tdata( _tdata )
		, image_buffer( _image_buffer )
		, last_image_index( _image_buffer.current_index() )
		, exit_flag( true )
		, recognizer_tuple( std::make_tuple( std::move( args )... ) )
		, prefilter_tuple{}
	{
		// for( auto const &t : cluster_data ) for( auto const &n : std::get< 1 >( t ) ) if( max_cluster_num < n ) max_cluster_num = n;
	}
	void run( void )
	{
		exit_flag = false;
		thread = std::thread( &recognition_thread::thread_func, this );
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
	std::size_t wait_next_image( void )
	{
		while( !exit_flag.load() )
		{
			std::size_t const ici = image_buffer.current_index();
			if( ici != last_image_index )
			{
				return ici;
			}
			// std::this_thread::yield();
		}
		return ABORT_WAIT_INDEX;
	}
	bool wait_for_need( void )
	{
#if !RCOGNITION_THREAD_NON_STOP_MODE
		while( !exit_flag.load() )
		{
			auto const point_size_size = std::size( tdata.point_size );
			for( auto i = 0u; i < point_size_size; ++i )
			{
				auto count = 0u;
				auto const size = tdata.point_size[ i ];
				for( auto j = 0u; j < size; ++j )
				{
					if( tdata.point[ i ][ j ].state == tracking_state::TRACKING )
					{
						if( ++count >= COUNT_THRESHOLD ) break;
					}
				}
				if( count < COUNT_THRESHOLD ) return true;
			}
		}
		return false;
#else
		return true;
#endif
	}
	void thread_func( void )
	{
		std::vector< key_point > kps;
		std::vector< cluster > clus;
		std::vector< GEOM_FADE2D::Triangle2 * > triangle;
		std::unique_ptr< GEOM_FADE2D::Fade_2D > fade2d;
		std::vector< election > elect;
		std::vector< Cluster_Index > cluster_index;
		
		// TODO: vectorのreserve

		interval_timer calc_fps( "recognition_thread" );

		// get_num_to_cluster_id( cluster_data, num_to_index );

#if RCOGNITION_THREAD_SHOW_PROCESS_TIME
		std::size_t processtime_index = 0u;
		std::vector< long long int > processtime_vector( 100 );
#endif
		while( !exit_flag.load() )
		{
			elect.clear();

			if( !wait_for_need() ) break;
			auto const image_index = wait_next_image();
			if( image_index == ABORT_WAIT_INDEX ) break;
#if RCOGNITION_THREAD_SHOW_PROCESS_TIME
			auto const start_time = std::chrono::high_resolution_clock::now();
#endif
#if SINGLE_RECOGNITION_DATA_CALC_TRACKING_THREAD_TIME
			auto const single_recognition_start_time = std::chrono::high_resolution_clock::now();
#endif

			last_image_index = image_index;
			auto img_gray = image_buffer.get_with_index( image_index );
			// std::cout << "recognition_thread: image_index = " << image_index << std::endl;

#if RCOGNITION_THREAD_DEBUG_SHOW
			// デバッグ用画像
			cv::Mat key_point_img, cluster_img, triangle_img, index_img;
			cv::cvtColor( img_gray, key_point_img, cv::COLOR_GRAY2BGR );
			cluster_img = key_point_img.clone();
			triangle_img = key_point_img.clone();
			index_img = key_point_img.clone();
#endif

#if RCOGNITION_THREAD_DEBUG_SHOW
			cv::Mat bin_frame;
			cv::adaptiveThreshold( img_gray, bin_frame, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 9, 8 );
			cv::erode( bin_frame, bin_frame, cv::noArray() );
			cv::cvtColor( bin_frame, bin_frame, cv::COLOR_GRAY2BGR );
			cv::imshow( "bin_frame", bin_frame );
#endif

			get_key_point( kps, img_gray, AREA_THRESHOLD_MIN, AREA_THRESHOLD_MAX );

#if RCOGNITION_THREAD_DEBUG_SHOW
			draw_key_point( key_point_img, kps );
			cv::imshow( "key_point", key_point_img );
#endif

			get_cluster( clus, kps, KEYPOINT_DISTANCE_THRESHOLD, KEYPOINT_GRID_NUM_THRESHOLD, MAX_NUM_OF_KEY_POINT_IN_CLUSTER, img_gray.cols, img_gray.rows );
			elect.resize( std::size( clus ) );

#if RCOGNITION_THREAD_DEBUG_SHOW
			draw_cluster( cluster_img, clus );
			cv::imshow( "cluster", cluster_img );
#endif
			get_triangulation( triangle, fade2d, clus );
			filter_triangle( triangle );

#if RCOGNITION_THREAD_DEBUG_SHOW
			draw_triangle( triangle_img, triangle );
			cv::imshow( "triangle", triangle_img );
#endif

			meta::for_each(
				prefilter_tuple,
				[ & ]( auto &f )
				{
					f.filter( meta::as_const( triangle ), meta::as_const( clus ) );
				}
			);
			meta::for_each(
				recognizer_tuple,
				[ & ]( auto &c )
				{
					c.recognize(
						elect, clus, triangle,
						std::get< typename std::decay_t< decltype( c ) >::prefilter >( prefilter_tuple )
					);
				}
			);
			get_cluster_index( cluster_index, elect );

#if RCOGNITION_THREAD_DEBUG_SHOW
			draw_cluster_index( index_img, cluster_index, clus );
			cv::imshow( "cluster_index", index_img );
#endif

#if RCOGNITION_THREAD_DEBUG_SHOW && RCOGNITION_THREAD_DEBUG_SHOW_SAVE
			cv::imwrite( "recognition_gray_frame.png", img_gray );
			cv::imwrite( "recognition_bin_frame.png", bin_frame );
			cv::imwrite( "recognition_key_point.png", key_point_img );
			cv::imwrite( "recognition_cluster.png", cluster_img );
			cv::imwrite( "recognition_triangle.png", triangle_img );
			cv::imwrite( "recognition_cluster_index.png", index_img );
#endif

			update_tracking_data( tdata, cluster_index, clus );
			tdata.recognition_frame.store( last_image_index, std::memory_order::memory_order_seq_cst );

#if SINGLE_RECOGNITION_DATA_CALC_TRACKING_THREAD_TIME
			auto const single_recognition_end_time = std::chrono::high_resolution_clock::now();
			auto const single_recognition_duration = single_recognition_end_time - single_recognition_start_time;
			tdata.single_recognition_duration.store( std::chrono::duration_cast< std::chrono::nanoseconds >( single_recognition_duration ).count(), std::memory_order::memory_order_seq_cst );
#endif
#if RCOGNITION_THREAD_SHOW_PROCESS_TIME
			auto const end_time = std::chrono::high_resolution_clock::now();
			auto const duration = end_time - start_time;
			auto const duration_count = std::chrono::duration_cast< std::chrono::nanoseconds >( duration ).count();
			processtime_vector[ processtime_index++ % processtime_vector.size() ] = duration_count;
			std::cout << duration_count << std::endl;
#endif

#if RCOGNITION_THREAD_DEBUG_SHOW
			/*
			for( int i = 0; i < 100; ++i )
			{
				if( exit_flag.load() ) break;
				cv::waitKey( 10 );
			}
			*/
			cv::waitKey( 1 );
#endif
			calc_fps.interval();
		}
	
#if RCOGNITION_THREAD_SHOW_PROCESS_TIME
		std::ofstream processtime_ofs( "!process_time.csv" );
		for( auto i = 0u; i < processtime_vector.size(); ++i  )
		{
			processtime_ofs << processtime_vector[ (processtime_index + i) % processtime_vector.size() ] << std::endl;
		}
#endif
	}
};
