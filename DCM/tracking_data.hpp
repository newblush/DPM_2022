#pragma once

#include <vector>
#include <atomic>
#include <cstdint>
#include <memory>

#define TRACKING_DATA_CALC_TRACKING_THREAD_TIME 1
#define PNP_RECOGNITION_DATA_CALC_TRACKING_THREAD_TIME 1
#define SINGLE_RECOGNITION_DATA_CALC_TRACKING_THREAD_TIME 1

enum class tracking_state : std::uint8_t
{
	NO_TRACKING,
	BEFORE_TRACKING,
	TRACKING,
};

struct point_tracking_data
{
	std::atomic< tracking_state > state{ tracking_state::NO_TRACKING };
	std::atomic< float > x{ 0.0f }, y{ 0.0f };
	std::atomic< float > sx{ 0.0f }, sy{ 0.0f };
	std::atomic< unsigned int > age{ 0u };
};

struct tracking_data
{
	std::vector< std::unique_ptr< point_tracking_data[] > > point;
	std::vector< unsigned int > point_size;
	std::atomic< std::size_t > tracking_start_frame{ 0u }; // ‚ ‚Æ‚©‚ç‚¯‚µ‚½‚¢
	std::atomic< std::size_t > recognition_frame{ 0u };
	std::atomic< std::size_t > pnp_frame{ 0u };
	std::atomic< std::size_t > tracking_frame{ 0u };
#if SINGLE_RECOGNITION_DATA_CALC_TRACKING_THREAD_TIME
	std::atomic< long long int > single_recognition_duration;
#endif
#if PNP_RECOGNITION_DATA_CALC_TRACKING_THREAD_TIME
	std::atomic< long long int > pnp_recognition_duration;
#endif
#if TRACKING_DATA_CALC_TRACKING_THREAD_TIME
	std::atomic< long long int > tracking_duration;
#endif
};

namespace
{
	template< typename Point2D >
	void tracking_data_point_get_tracking_points( point_tracking_data const * const p, unsigned int const psize, std::vector< unsigned int > &objid, std::vector< Point2D > &position )
	{
		objid.clear();
		position.clear();
		for( auto i = 0u; i < psize; ++i )
		{
			auto const &point = p[ i ];
			auto const state = point.state.load( std::memory_order::memory_order_acquire );
			if( state != tracking_state::TRACKING ) continue;
			auto const x = point.x.load( std::memory_order::memory_order_relaxed );
			auto const y = point.y.load( std::memory_order::memory_order_relaxed );
			std::atomic_thread_fence( std::memory_order::memory_order_acq_rel );
			auto const state2 = point.state.load( std::memory_order::memory_order_relaxed );
			if( state2 != tracking_state::TRACKING ) continue;
			position.emplace_back( x, y );
			objid.emplace_back( i );
		}
	}
}
