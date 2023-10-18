#include <vector>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <limits>
#include <unordered_map>
#include <array>
#include "recognition.hpp"
#include "findMarker.hpp"
#include "make_point.hpp"
#include "tracking_data.hpp"

namespace std
{
	template<>
	struct hash< std::tuple< unsigned int, unsigned int > >
	{
		typedef std::tuple< unsigned int, unsigned int > argument_type;
		typedef std::size_t result_type;
		result_type operator()(argument_type const &s) const
		{
			result_type const h1 = std::hash< unsigned int >{}( std::get< 0 >( s ) );
			result_type const h2 = std::hash< unsigned int >{}( std::get< 1 >( s ) );
			return h1 ^ (h2 << 1); // or use boost::hash_combine
		}
	};
}

namespace recognition
{

Cluster_Index election::elected( void )
{
	if( elect_num < 2 ) return INVALID_CLUSTER_INDEX;
	std::sort( elect.begin(), elect.begin() + elect_num );
	Cluster_Index const *obj_max = nullptr, *obj_crr = &elect[ 0 ];
	auto count_max = 0u, count_crr = 1u;
	auto update = [ & ]( void )
	{
		if( count_max == count_crr )
		{
			obj_max = &INVALID_CLUSTER_INDEX;
		}
		else if( count_max < count_crr )
		{
			obj_max = obj_crr;
			count_max = count_crr;
		}
	};
	for( auto i = 1u; i < elect_num; ++i )
	{
		if( *obj_crr == elect[ i ] ) ++count_crr;
		else
		{
			update();
			obj_crr = &elect[ i ];
			count_crr = 1u;
		}
	}
	update();
	return *obj_max;
}

// この関数，全体的にオーバースペック
void get_key_point( std::vector< key_point > &ret, cv::Mat &frame_gray, float const area_threshold_min, float const area_threshold_max )
{
	// 二値化
	cv::Mat frame_bin;
	cv::adaptiveThreshold( frame_gray, frame_bin, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 9, 8 );

	cv::Mat fuck;
	cv::erode( frame_bin, fuck, cv::noArray() );
	// cv::dilate( fuck, fuck, cv::noArray() );
	frame_bin = fuck;
	// cv::imshow( "frame_bin", frame_bin );

	// 輪郭抽出（ここ遅そう）
	std::vector< std::vector< cv::Point > > contour_arr;
	// ここが1ms近い
	// cv::findContours( frame_bin, contour_arr, cv::noArray(), cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE );
	cv::findContours( frame_bin, contour_arr, cv::noArray(), cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE );

	auto const contour_arr_size = std::size( contour_arr );
	ret.clear();
	ret.reserve( contour_arr_size );
	for( auto i = 0u; i < contour_arr_size; ++i)
	{
		key_point kp;
		auto const f = key_point_from_contour( kp.x, kp.y, kp.sx, kp.sy, contour_arr[ i ], area_threshold_min, area_threshold_max );
		if( !f ) continue;
		ret.emplace_back( std::move( kp ) );
	}
}

namespace
{	
	struct grid
	{
		std::vector< std::vector< std::size_t > > index;
		unsigned int col, row;
		float width, height;
	};
	
	void devide_keypoint_to_grid( grid &ret, std::vector< key_point > const &kp, float const distance_threshold, std::size_t const key_point_num_threshold, unsigned int const width, unsigned int height )
	{
		auto &index = ret.index;
		for( auto &&v : index ) v.clear();
		unsigned int const grid_col = static_cast< unsigned int >( width / distance_threshold );
		unsigned int const grid_row = static_cast< unsigned int >( height / distance_threshold );
		float const grid_width  = static_cast< float >( width ) / grid_col, grid_height = static_cast< float >( height ) / grid_row;
		ret.col = grid_col, ret.row = grid_row;
		ret.width = grid_width, ret.height = grid_height;
		std::size_t const index_size = grid_col * grid_row, kp_size = kp.size();
		index.resize( index_size );
		for( std::size_t i = 0u; i < kp_size; ++i )
		{
			unsigned int xi = static_cast< unsigned int >( kp[ i ].x / grid_width );
			unsigned int yi = static_cast< unsigned int >( kp[ i ].y / grid_height ); 
			xi = std::min( xi, grid_col - 1 ), yi = std::min( yi, grid_row - 1 );
			index[ yi * grid_col + xi ].emplace_back( i );
		}
		for( auto &&v : index )
		{
			if( v.size() > key_point_num_threshold ) v.clear();
		}
	}
}

// Union Find Tree の実装のちょい簡略版（多少効率が悪いかもしれない）
struct group
{
	static constexpr std::size_t INVALID_INDEX = std::numeric_limits< std::size_t >::max();
	std::vector< std::size_t > data;
	group( std::size_t size )
		: data( size, INVALID_INDEX )
	{}
	std::size_t get( std::size_t i )
	{
		auto const gi = data[ i ];
		return gi == INVALID_INDEX ? i : data[ i ] = get( gi );
	}
	void same( std::size_t const crr, std::size_t const add )
	{
		auto const ai = get( add );
		auto ci = crr;
		while( true )
		{
			auto const ni = data[ ci ];
			if( ni == ai ) break;
			if( ni == INVALID_INDEX )
			{
				if( ai < ci ) data[ ci ] = ai;
				else data[ ai ] = ci;
				break;
			}
			ci = ni;
		}
	}
};

void get_cluster( std::vector< cluster > &ret, std::vector< key_point > const &kp, float const keypoint_distance_threshold, std::size_t const key_point_grid_num_threshold, unsigned int const max_num_of_key_point_in_cluster, unsigned int const width, unsigned int height )
{
	ret.clear();

	auto const kp_size = kp.size();
	group gi( kp_size );
	for( std::size_t i = 0u; i < kp_size; ++i )
	{
		auto const &kpi = kp[ i ];
		auto const csx = correction_standard_deviation( kpi.sx ), csy = correction_standard_deviation( kpi.sy );
		auto const csm = std::max( csx, csy ) * 8.0f;
		auto const squared_csm = csm * csm;
		for( std::size_t j = i + 1; j < kp_size; ++j )
		{
			auto const &kpj = kp[ j ];
			float const dx = kpi.x - kpj.x, dy = kpi.y - kpj.y;
			if( dx * dx + dy * dy < squared_csm )
			{
				gi.same( j, i );
			}
		}
	}

	std::vector< std::size_t > index2index( kp_size );
	for( std::size_t i = 0u; i < kp_size; ++i )
	{
		auto const &k = kp[ i ];
		if( gi.data[ i ] == group::INVALID_INDEX )
		{
			cluster c;
			c.x = k.x, c.y = k.y;
			c.sx = k.sx, c.sy = k.sy;
			c.num = 1u;
			index2index[ i ] = ret.size();
			ret.emplace_back( std::move( c ) );
		}
		else
		{
			auto &c = ret[ index2index[ gi.get( i ) ] ];
			c.x += k.x, c.y += k.y;
			c.sx += k.sx, c.sy += k.sy;
			++c.num;
		}
	}
	ret.erase( std::remove_if( ret.begin(), ret.end(), [ & ]( auto const &c ){ return c.num > max_num_of_key_point_in_cluster; } ), ret.end() );
	for( auto &c : ret ) c.x /= c.num, c.y /= c.num, c.sx /= c.num, c.sy /= c.num;
}

void get_triangulation( std::vector< GEOM_FADE2D::Triangle2 * > &triangle, std::unique_ptr< GEOM_FADE2D::Fade_2D > &fade2d, std::vector< cluster > const &clu )
{
	auto const clusize = clu.size();
	if( clusize < 3u )
	{
		triangle.clear();
		return;
	}
	fade2d = std::make_unique< GEOM_FADE2D::Fade_2D >( static_cast< unsigned int >( clusize ) );
	std::vector< GEOM_FADE2D::Point2 > vertex_arr;
	vertex_arr.reserve( clusize );
	for( auto i = 0u; i < clusize; ++i )
	{
		// FADE2Dと座標系の取り方が鏡写しのため，yは反転させる
		vertex_arr.emplace_back( clu[ i ].x, -clu[ i ].y );
		vertex_arr.back().setCustomIndex( static_cast< int >( i ) );
	}
	fade2d->insert( vertex_arr );
	fade2d->getTrianglePointers( triangle );
}

void filter_triangle( std::vector< GEOM_FADE2D::Triangle2 * > &triangle )
{
	/*
	auto rit = std::remove_if(
		triangle.begin(), triangle.end(),
		[]( auto const &tri )
		{
			for( auto i = 0; i < 3; ++i )
			{
				auto sqlen = tri->getSquaredEdgeLength( i );
				// ここの値はもっと動的に決めるべきであろう
				if( sqlen >= 5000 ) return true;
			}
			return false;
		}
	);
	triangle.erase( rit, triangle.end() );
	*/
}

void get_cluster_index( std::vector< Cluster_Index > &cluster_index, std::vector< election > &elect )
{
	auto const elect_size = std::size( elect );
	cluster_index.resize( elect_size );
	for( auto i = 0u; i < elect_size; ++i )
	{
		cluster_index[ i ] = elect[ i ].elected();
	}
}

void update_tracking_data( tracking_data &tdata, std::vector< Cluster_Index > const &cluster_index, std::vector< cluster > const &clus )
{
	for( auto i = 0u; i < std::size( cluster_index ); ++i )
	{
		auto const &pno = std::get< 0 >( cluster_index[ i ] );
		auto const &ind = std::get< 1 >( cluster_index[ i ] );
		if( pno == recognition::INVALID_INDEX ) continue;
		auto const &clusi = clus[ i ];
		auto &p = tdata.point[ pno ][ ind ];
		auto const state = p.state.load( std::memory_order::memory_order_acquire );
		if( state == tracking_state::NO_TRACKING )
		{
			p.x.store( clusi.x, std::memory_order::memory_order_relaxed );
			p.y.store( clusi.y, std::memory_order::memory_order_relaxed );
			p.sx.store( clusi.sx, std::memory_order::memory_order_relaxed );
			p.sy.store( clusi.sy, std::memory_order::memory_order_relaxed );
			p.age.store( 0u, std::memory_order::memory_order_relaxed );
			p.state.store( tracking_state::BEFORE_TRACKING, std::memory_order::memory_order_release );
		}
		else
		{
			auto const x = p.x.load( std::memory_order::memory_order_relaxed );
			auto const y = p.y.load( std::memory_order::memory_order_relaxed );
			auto const dx = x - clusi.x, dy = y - clusi.y;
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

} // namescape recognition
