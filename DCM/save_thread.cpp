#include <numeric>
#include <chrono>
#include <fstream>
#include <sstream>
#include <iomanip>
#include "save_thread.hpp"
#include "findMarker.hpp"
#include "make_point.hpp"
#include "interval_timer.hpp"
#include "tracking_data.hpp"

const unsigned int SAVE_FRAME_NUM = 1000;

using cclock = std::chrono::high_resolution_clock;

constexpr std::size_t INVALID_INDEX = std::numeric_limits< std::size_t >::max();

std::size_t save_thread::wait_tracking_update( void )
{
	while( !exit_flag.load() )
	{
		// std::size_t const index = tdata.tracking_frame.load();
		std::size_t const index = tdata.tracking_start_frame.load();
		if( index != last_image_index )
		{
			return index;
		}
		// std::this_thread::yield();
	}
	return INVALID_INDEX;
}
void save_thread::thread_func( void )
{
	auto const cluster_data_size = tdata.point_size.size();
	assert( cluster_data_size == 1u );

	auto const POINT_NUM = tdata.point_size[ 0 ];

	std::vector< cclock::time_point > time_vector( SAVE_FRAME_NUM );
	std::vector< cv::Mat > img_vector( SAVE_FRAME_NUM );
	std::vector< std::size_t > framenum_vector( SAVE_FRAME_NUM );
	std::vector < std::size_t > singleframenum_vector( SAVE_FRAME_NUM );
	std::vector< std::size_t > pnpframenum_vector( SAVE_FRAME_NUM );
#if SINGLE_RECOGNITION_DATA_CALC_TRACKING_THREAD_TIME
	std::vector< long long int > single_recognition_time_vector( SAVE_FRAME_NUM );
#endif
#if PNP_RECOGNITION_DATA_CALC_TRACKING_THREAD_TIME
	std::vector< long long int > pnp_track_time_vector( SAVE_FRAME_NUM );
#endif
#if TRACKING_DATA_CALC_TRACKING_THREAD_TIME
	std::vector< long long int > tracking_time_vector( SAVE_FRAME_NUM );
#endif
	std::vector< std::vector< unsigned int > > objid_vector( SAVE_FRAME_NUM, std::vector< unsigned int >( POINT_NUM ) );
	std::vector< std::vector< cv::Point2f > > objpoint_vector( SAVE_FRAME_NUM, std::vector< cv::Point2f >( POINT_NUM ) );
	std::vector< std::vector< tracking_state > > objstate_vector( SAVE_FRAME_NUM, std::vector< tracking_state >( POINT_NUM ) );
	for( unsigned int i = 0; i < SAVE_FRAME_NUM; ++i )
	{
		auto const tracking_index = wait_tracking_update();
		if( tracking_index == INVALID_INDEX ) break;
		last_image_index = tracking_index;
		time_vector[ i ] = cclock::now();
		pnpframenum_vector[ i ] = tdata.pnp_frame.load();
		singleframenum_vector[ i ] = tdata.recognition_frame.load();
		img_vector[ i ] = image_buffer.get_with_index( last_image_index ).clone();
		framenum_vector[ i ] = last_image_index;
#if SINGLE_RECOGNITION_DATA_CALC_TRACKING_THREAD_TIME
		single_recognition_time_vector[ i ] = tdata.single_recognition_duration.load();
#endif
#if PNP_RECOGNITION_DATA_CALC_TRACKING_THREAD_TIME
		pnp_track_time_vector[ i ] = tdata.pnp_recognition_duration.load();
#endif
#if TRACKING_DATA_CALC_TRACKING_THREAD_TIME
		tracking_time_vector[ i ] = tdata.tracking_duration.load();
#endif

		auto const &paper = tdata.point[ 0 ];
		auto const paper_cluster_size = tdata.point_size[ 0 ];
		unsigned int count = 0u;
		auto &objid = objid_vector[ i ];
		auto &objpoint = objpoint_vector[ i ];
		auto &objstate = objstate_vector[ i ];
		for( auto j = 0u; j < POINT_NUM; ++j )
		{
			auto const &point = paper[ j ];
			auto const state = point.state.load( std::memory_order::memory_order_acquire );
			// if( state != tracking_state::TRACKING ) continue;
			auto const x = point.x.load( std::memory_order::memory_order_relaxed );
			auto const y = point.y.load( std::memory_order::memory_order_relaxed );
			std::atomic_thread_fence( std::memory_order::memory_order_acq_rel );
			// auto const state2 = point.state.load( std::memory_order::memory_order_relaxed );
			// if( state2 != tracking_state::TRACKING ) continue;
			objid[ count ] = j;
			objpoint[ count ] = cv::Point2f( x, y );
			objstate[ count ] = state;
			++count;
		}
		objid.resize( count );
		objpoint.resize( count );
	}
	
	std::ofstream ofs( "!save.dat" );
	ofs << SAVE_FRAME_NUM << " " <<
#if SINGLE_RECOGNITION_DATA_CALC_TRACKING_THREAD_TIME
	"e" <<	// single image recognition の時間計測
#endif
#if TRACKING_DATA_CALC_TRACKING_THREAD_TIME
	"t" <<
#endif
#if PNP_RECOGNITION_DATA_CALC_TRACKING_THREAD_TIME
	"r" <<	// pnp recogonition の時間計測
#endif
	"s" <<	// tracking_stateの出力追加
	"c" <<	// single image recognitionのフレーム番号出力
	"p" <<	// pnp_track_threadのフレーム番号出力
	"\n";
	for( auto i = 0u; i < SAVE_FRAME_NUM; ++i )
	{
		auto const &objid = objid_vector[ i ];
		auto const &objpoint = objpoint_vector[ i ];
		auto const &objstate = objstate_vector[ i ];
		assert( objid.size() == objpoint.size() );
		std::clog << i << " frame" << std::endl;
		std::ostringstream oss;
		oss << "!save" << std::setw( 4 ) << std::setfill( '0' ) << i << ".png";
		cv::imwrite( oss.str(), img_vector[ i ] );
		ofs <<
			std::chrono::duration_cast< std::chrono::nanoseconds >( time_vector[ i ].time_since_epoch() ).count() << " " <<
			framenum_vector[ i ] << " " <<
#if SINGLE_RECOGNITION_DATA_CALC_TRACKING_THREAD_TIME
			single_recognition_time_vector[ i ] << " " <<
#endif
#if PNP_RECOGNITION_DATA_CALC_TRACKING_THREAD_TIME
			pnp_track_time_vector[ i ] << " " <<
#endif
#if TRACKING_DATA_CALC_TRACKING_THREAD_TIME
			tracking_time_vector[ i ] << " " <<
#endif 
			singleframenum_vector[ i ] << " " <<
			pnpframenum_vector[ i ] << " " <<
			objid.size() << "\n";
		for( auto j = 0u; j < objid.size(); ++j )
		{
			ofs << (j == 0u ? "" : " ") << objid[ j ] << " " << objpoint[ j ].x << " " << objpoint[ j ].y << " " << (int)(std::underlying_type_t< tracking_state >)objstate[ j ];
		}
		ofs << "\n";
	}
	ofs << std::flush;
	std::clog << "save_thread: save done!" << std::endl;
}
