#include <chrono>
#include "pnp_track_thread.hpp"
#include "findMarker.hpp"
#include "make_point.hpp"
#include "interval_timer.hpp"

#define PNP_TRACK_THREAD_DEBUG_SHOW 1
#define PNP_TRACK_THREAD_DEBUG_SHOW_SAVE 0

constexpr std::size_t INVALID_INDEX = std::numeric_limits< std::size_t >::max();

std::size_t pnp_track_thread::wait_tracking_update( void )
{
	while( !exit_flag.load() )
	{
		std::size_t const index = tdata.tracking_frame.load();
		if( index != last_image_index)
		{
			return index;
		}
		// std::this_thread::yield();
	}
	return INVALID_INDEX;
}
void pnp_track_thread::thread_func( void )
{
	interval_timer calc_fps( "pnp_track_thread" );

	std::vector< cv::Point3f > objpoint;
	std::vector< cv::Point2f > imagepoint;
	cv::Mat rvec, tvec;
	cv::Mat rmat;

	auto const cluster_data_size = std::size( tdata.point_size );
	assert( cluster_data_size == 1u );
	if( !(cluster_data_size == 1u) ) std::clog << "ï≥é¿ëï" << std::endl;
	auto const point_num = tdata.point_size[ 0 ];
	std::vector< cv::Point3f > allPoint;
	std::vector< cv::Point2f > projectedPoint;
#if PNP_TRACK_THREAD_DEBUG_SHOW
	std::vector< unsigned int > pointId;
#endif
	for( auto i = 0u; i < point_num; ++i )
	{
		auto const * const pp = &point_vector[ 0 ][ i * 3 ];
		allPoint.emplace_back( pp[ 0 ], pp[ 1 ], pp[ 2 ] );
	}

	while( !exit_flag.load() )
	{
		objpoint.clear();
		imagepoint.clear();
#if PNP_TRACK_THREAD_DEBUG_SHOW
		pointId.clear();
#endif
		auto const tracking_index = wait_tracking_update();
		if( tracking_index == INVALID_INDEX ) break;
#if PNP_RECOGNITION_DATA_CALC_TRACKING_THREAD_TIME
		auto const pnp_track_time_start = std::chrono::high_resolution_clock::now();
#endif
		last_image_index = tracking_index;
		auto crrimg = image_buffer.get_with_index( last_image_index );
		
		auto max_sx = -std::numeric_limits< float >::max(), max_sy = max_sx;
		for( auto j = 0u; j < point_num; ++j )
		{
			auto const &point = tdata.point[ 0 ][ j ];
			auto const state = point.state.load( std::memory_order::memory_order_acquire );
			if( state != tracking_state::TRACKING ) continue;
			// if( std::get< 1 >( cluster_data[ 0 ] )[ j ] == 1u ) continue;	// key_pointÇ™1ÇÃÇ‡ÇÃÇÕÉmÉCÉYÇ™èÊÇËÇ‚Ç∑Ç¢ÇÃÇ≈íeÇ≠
			auto const x = point.x.load( std::memory_order::memory_order_relaxed );
			auto const y = point.y.load( std::memory_order::memory_order_relaxed );
			max_sx = std::max( max_sx, point.sx.load( std::memory_order::memory_order_relaxed ) );
			max_sy = std::max( max_sy, point.sy.load( std::memory_order::memory_order_relaxed ) );
			std::atomic_thread_fence( std::memory_order::memory_order_acq_rel );
			auto const state2 = point.state.load( std::memory_order::memory_order_relaxed );
			if( state2 != tracking_state::TRACKING ) continue;
			imagepoint.emplace_back( x, y );
			auto pp = &point_vector[ 0 ][ j * 3 ];
			objpoint.emplace_back( pp[ 0 ], pp[ 1 ], pp[ 2 ] );
#if PNP_TRACK_THREAD_DEBUG_SHOW
			pointId.emplace_back( j );
#endif
		}

#if PNP_TRACK_THREAD_DEBUG_SHOW
		cv::Mat solve_pnp_point_img, 
		reproject_point_img, visible_point_img;
		flag =false;
		//solve_pnp_point_img= cv::Mat(540,720,CV_8UC3,cv::Scalar::all(100));
		cv::cvtColor( crrimg, solve_pnp_point_img, cv::COLOR_GRAY2BGR );
		reproject_point_img = solve_pnp_point_img.clone();
		visible_point_img = solve_pnp_point_img.clone();

		for( auto i = 0u; i < std::size( imagepoint ); ++i )
		{
			auto const &p = imagepoint[ i ];
			cv::circle( solve_pnp_point_img, p, 5, cv::Scalar( 255, 255, 0 ), -1 );
		}
		flag = true;
#endif
		auto const objpoint_size = std::size( objpoint );
		if( objpoint_size >= 3 )
		{
			bool const solvePnP_ret = cv::solvePnP( objpoint, imagepoint, cameraMatrix, cameraCoeffs, rvec, tvec );
			cv::projectPoints( allPoint, rvec, tvec, cameraMatrix, cameraCoeffs, projectedPoint );
			
			auto const max_sxy = std::max( max_sx, max_sy );

			cv::Rodrigues( rvec, rmat );
			std::cout << "RMAT-" << rmat << std::endl;
			std::cout << "TVEC-" << tvec << std::endl;
			for( auto i = 0u; i < point_num; ++i )
			{
				auto &p = tdata.point[ 0 ][ i ];
				auto const &v = projectedPoint[ i ];
				auto const nv = &dcm_normal_vector[ 0 ][ i * 3 ];
				// auto const nx = static_cast< float >( rmat.at< double >( 0, 0 ) * nv[ 0 ] + rmat.at< double >( 0, 1 ) * nv[ 1 ] + rmat.at< double >( 0, 2 ) * nv[ 2 ] );
				// auto const ny = static_cast< float >( rmat.at< double >( 1, 0 ) * nv[ 0 ] + rmat.at< double >( 1, 1 ) * nv[ 1 ] + rmat.at< double >( 1, 2 ) * nv[ 2 ] );
				auto const nz = static_cast< float >( rmat.at< double >( 2, 0 ) * nv[ 0 ] + rmat.at< double >( 2, 1 ) * nv[ 1 ] + rmat.at< double >( 2, 2 ) * nv[ 2 ] );
 				// std::cout << nx << ", " << ny << ", " << nz << std::endl;
#if PNP_TRACK_THREAD_DEBUG_SHOW
				cv::circle( reproject_point_img, v, 5, cv::Scalar( 255, 255, 0 ), -1 );
#endif
				if( nz > -0.3f )
				{
					if( nz < 0.0f || p.state.load( std::memory_order::memory_order_acquire ) == tracking_state::NO_TRACKING ) continue;
					// çƒìäâeÇµÇΩåãâ ÅCämé¿Ç…å©Ç¶ÇƒÇ¢Ç»Ç¢Ç‡ÇÃÇÕóéÇ∆Ç∑
					p.state.store( tracking_state::NO_TRACKING, std::memory_order::memory_order_seq_cst );
					p.x.store( 0.0f, std::memory_order::memory_order_relaxed );
					p.y.store( 0.0f, std::memory_order::memory_order_relaxed );
					p.sx.store( 0.0f, std::memory_order::memory_order_relaxed );
					p.sy.store( 0.0f, std::memory_order::memory_order_relaxed );
					p.age.store( 0u, std::memory_order::memory_order_relaxed );
					continue;
				}
#if PNP_TRACK_THREAD_DEBUG_SHOW
				cv::circle( visible_point_img, v, 5, cv::Scalar( 255, 255, 0 ), -1 );
#endif
				float const correction_max_sxy = correction_standard_deviation( max_sxy );
				float x, y, sx, sy;
				unsigned kpn;
				// if( !findMarker( crrimg, v.x, v.y, correction_max_sxy * 12.0f, correction_max_sxy * 12.0f, x, y, sx, sy, kpn, "pnp", true ) ) continue;
				if( !findMarker( crrimg, v.x, v.y, correction_max_sxy * 20.0f, correction_max_sxy * 20.0f, num_vector[ 0 ][ i ], x, y, sx, sy, kpn, "pnp" ) ) continue;
				// if( !findMarker( crrimg, v.x, v.y, 60.0f, 60.0f, x, y, sx, sy, kpn, "pnp" ) ) continue;
				if( kpn != num_vector[ 0 ][ i ] ) continue;
				if( kpn == 1 && nz > -0.7 ) continue;
				auto const state = p.state.load( std::memory_order::memory_order_acquire );
				if( state == tracking_state::NO_TRACKING )
				{
					auto const dx = v.x - x, dy = v.y - y;
					if( dx * dx + dy * dy <= (correction_max_sxy * correction_max_sxy * (2 * 2)) )
					{
						p.x.store( x, std::memory_order::memory_order_relaxed );
						p.y.store( y, std::memory_order::memory_order_relaxed );
						p.sx.store( sx, std::memory_order::memory_order_relaxed );
						p.sy.store( sy, std::memory_order::memory_order_relaxed );
						p.age.store( 0u, std::memory_order::memory_order_relaxed );
						p.state.store( tracking_state::BEFORE_TRACKING, std::memory_order_release );
					}
				}
				else
				{
					auto const cx = p.x.load( std::memory_order::memory_order_relaxed );
					auto const cy = p.y.load( std::memory_order::memory_order_relaxed );
					auto const dx = cx - x, dy = cy - y;
					if( dx * dx + dy * dy >= 20 * 20 )
					{
						p.state.store( tracking_state::NO_TRACKING, std::memory_order::memory_order_seq_cst );
						p.x.store( 0.0f, std::memory_order::memory_order_relaxed );
						p.y.store( 0.0f, std::memory_order::memory_order_relaxed );
						p.sx.store( 0.0f, std::memory_order::memory_order_relaxed );
						p.sy.store( 0.0f, std::memory_order::memory_order_relaxed );
						p.age.store( 0u, std::memory_order::memory_order_relaxed );
					}
				}
			}
		}
		tdata.pnp_frame.store( last_image_index, std::memory_order::memory_order_seq_cst );
#if PNP_RECOGNITION_DATA_CALC_TRACKING_THREAD_TIME
		auto const pnp_track_time_end = std::chrono::high_resolution_clock::now();
		auto const pnp_track_time_duration = pnp_track_time_end - pnp_track_time_start;
		tdata.pnp_recognition_duration.store( std::chrono::duration_cast< std::chrono::nanoseconds >( pnp_track_time_duration ).count(), std::memory_order::memory_order_seq_cst );
#endif
#if PNP_TRACK_THREAD_DEBUG_SHOW && PNP_TRACK_THREAD_DEBUG_SHOW_SAVE
		cv::imwrite( "pnp_used_points.png", solve_pnp_point_img );
		cv::imwrite( "pnp_reprojected_points.png", reproject_point_img );
		cv::imwrite( "pnp_visible_points.png", visible_point_img );
#endif
#if PNP_TRACK_THREAD_DEBUG_SHOW
		cv::imshow( "used_points", solve_pnp_point_img );
		cv::imshow( "reprojected_points", reproject_point_img );
		cv::imshow( "visible_points", visible_point_img );
		cv::waitKey( 1 );
#endif
		calc_fps.interval();
	}
}