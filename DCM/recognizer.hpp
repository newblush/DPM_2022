#pragma once

#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <vector>
#include <queue>
#include "make_point.hpp"
#include "meta_programming.hpp"
#include "compier_attribute.hpp"
#include "recognition.hpp"
#include "custom_hash.hpp"
#include "vector.hpp"

namespace recognition
{

struct none_prefilter{
	void filter( std::vector< GEOM_FADE2D::Triangle2 * > const &triangle, std::vector< cluster > const &clu )
	{
		// do nothing
	}
};

class larger_triangle_recognizer
{
private:
	constexpr static auto CHECK_NUM = 6u;
	using Cluster_Index_Tuple = meta::multi_tuple_t< CHECK_NUM, Cluster_Index >;
	using Cluster_Num_Tuple = meta::multi_tuple_t< CHECK_NUM, unsigned int >;

public:
	using prefilter = none_prefilter;

private:
	cunordered_map< Cluster_Num_Tuple, Cluster_Index_Tuple > map;

public:
	// tuple—v‘f = < •¨‘Ì‚Ì”Ô†, index, num >
	larger_triangle_recognizer(
		std::vector< std::tuple< unsigned int, std::vector< unsigned int >, std::vector< unsigned int > > > const &cluster_data
	)
	{
		for( auto i = 0u; i < std::size( cluster_data ); ++i )
		{
			auto const &t = cluster_data[ i ];
			auto const &objnum = std::get< 0 >( t );
			auto const &index = std::get< 1 >( t );
			auto const &num = std::get< 2 >( t );
			
			auto const &vmap = make_vmap( index );
			// auto const &vmap_index = make_map_pindex_to_index( index );
	
			auto add_map = [ & ]( auto... arg )
			{
				auto r = map.emplace( std::forward_as_tuple( num[ arg ]... ), std::forward_as_tuple( std::make_tuple( objnum, arg )... ) );
				if( !r.second ) throw std::logic_error( "larger_triangle_recognizer: same cluster pattern!" );
			};
	
			auto const index_size = std::size( index );
			for( auto j = 0u; j + 2 < index_size; j += 3 )
			{
				auto const a = index[ j + 0 ], b = index[ j + 1 ], c = index[ j + 2 ];
				auto const u_it = vmap.find( std::make_tuple( b, a ) );
				auto const v_it = vmap.find( std::make_tuple( c, b ) );
				auto const w_it = vmap.find( std::make_tuple( a, c ) );
				if( u_it == vmap.end() || v_it == vmap.end() || w_it == vmap.end() ) continue;
				auto const u = u_it->second, v = v_it->second, w = w_it->second;
	
				add_map( a, b, c, u, v, w );
				add_map( b, c, a, v, w, u );
				add_map( c, a, b, w, u, v );
			}
		}
	}

	void recognize(
		std::vector< election > &elect,
		std::vector< cluster > const &clu,
		std::vector< GEOM_FADE2D::Triangle2 * > const &triangle,
		prefilter const &
	) const
	{
		auto const clu_size = std::size( clu );

		constexpr unsigned int NUM_TOHYO = 12u;
		using Tohyo = std::tuple< std::array< Cluster_Index, NUM_TOHYO + 1u >, unsigned int >;
		std::vector< Tohyo > tohyo( clu_size );
		for( auto const &tri : triangle )
		{
			auto const ot0 = tri->getOppositeTriangle( 0 ), ot1 = tri->getOppositeTriangle( 1 ), ot2 = tri->getOppositeTriangle( 2 );
			if( !ot0 || !ot1 || !ot2 ) continue;
			auto const pt0 = ot0->getIntraTriangleIndex( tri ), pt1 = ot1->getIntraTriangleIndex( tri ), pt2 = ot2->getIntraTriangleIndex( tri );
			unsigned int const a = tri->getCorner( 0 )->getCustomIndex(), b = tri->getCorner( 1 )->getCustomIndex(), c = tri->getCorner( 2 )->getCustomIndex();
			unsigned int const u = ot2->getCorner( pt2 )->getCustomIndex(), v = ot0->getCorner( pt0 )->getCustomIndex(), w = ot1->getCorner( pt1 )->getCustomIndex();
			auto it = [ & ]( auto &&... c ){ return map.find( std::make_tuple( clu[ c ].num... ) ); }( a, b, c, u, v, w );
			if( it == map.end() ) continue;
			meta::for_each( std::forward_as_tuple( a, b, c, u, v, w ), it->second, [ & ]( unsigned int const i, Cluster_Index const &n )
			{
				elect[ i ].vote( n );
			} );
		}
	}
}; // class larger_triangle_recognizer

struct square_prefilter
{
private:
	static constexpr float L_SLESHOLD = 0.95f, THETA_PALLA = 0.05f, THETA_VERTI = 0.05f;

	cunordered_map< std::tuple< GEOM_FADE2D::Triangle2 *, GEOM_FADE2D::Triangle2 * >, std::tuple< int, int, float > > triangle_pair;
	std::vector< decltype( triangle_pair )::iterator > square;

private:
	using Vector = kato::vector2f;

	static
	Vector VFP( GEOM_FADE2D::Point2 *p )
	{
		return Vector( static_cast< float >( p->x() ), static_cast< float >( p->y() ) );
	}
	static
	std::tuple< GEOM_FADE2D::Triangle2 *, GEOM_FADE2D::Triangle2 * > sort_tptr( GEOM_FADE2D::Triangle2 *p1, GEOM_FADE2D::Triangle2 *p2 ) noexcept
	{
		return reinterpret_cast< std::uintptr_t >( p1 ) < reinterpret_cast< std::uintptr_t >( p2 ) ? std::make_tuple( p1, p2 ) : std::make_tuple( p2, p1 );
	}

public:
	void filter( std::vector< GEOM_FADE2D::Triangle2 * > const &triangle, std::vector< cluster > const &clu )
	{
		triangle_pair.clear();
		square.clear();
		auto const triangle_size = std::size( triangle );
		for( auto i = 0u; i < triangle_size; ++i )
		{
			auto const t = triangle[ i ];
			for( auto id = 0; id < 3; ++id )
			{
				auto const ot = t->getOppositeTriangle( id );
				if( ot == nullptr ) continue;
				auto const tpn = reinterpret_cast< std::uintptr_t >( t ), otpn = reinterpret_cast< std::uintptr_t >( ot );
				if( tpn >= otpn ) continue;
				auto const oid = ot->getIntraTriangleIndex( t );
				auto const pa = t->getCorner( id ), pb = t->getCorner( (id + 1) % 3 ), pc = ot->getCorner( oid ), pd = t->getCorner( (id + 2) % 3 );
				auto const va = VFP( pa ), vb = VFP( pb ), vc = VFP( pc ), vd = VFP( pd );
				auto const nvab = (vb - va).normarize(), nvad = (vd - va).normarize(), nvcb = (vb - vc).normarize(), nvcd = (vd - vc).normarize(), nvac = (vc - va).normarize(), nvdb = (vb - vd).normarize();
				auto const ipabad = nvab.inner_product( nvad ), ipcbcd = nvcb.inner_product( nvcd ), ipacdb = nvac.inner_product( nvdb );
				auto const l = 1.0f - ipabad * ipabad / 3.0f - ipcbcd * ipcbcd / 3.0f - ipacdb * ipacdb / 3.0f;
				if( l < L_SLESHOLD ) continue;
				auto v = triangle_pair.emplace( std::piecewise_construct, std::make_tuple( t, ot ), std::make_tuple( id, oid, l ) );
				square.emplace_back( v.first );
			}
		}
		std::sort( square.begin(), square.end(), []( auto const &a, auto const &b ){ return std::get< float >( a->second ) < std::get< float >( b->second ); });
		cunordered_set< GEOM_FADE2D::Triangle2 * > traveled_triangle;
		std::queue< decltype( triangle_pair )::iterator > queue;
		for( auto const &cs : square )
		{
			queue.push( cs );
			while( !queue.empty() )
			{
				auto const s = queue.back(); queue.pop();
				auto const t1 = std::get< 0 >( s->first ), t2 = std::get< 1 >( s->first );
				if( traveled_triangle.find( t1 ) != traveled_triangle.end() || traveled_triangle.find( t2 ) != traveled_triangle.end() ) continue;
				traveled_triangle.emplace( t1 );
				traveled_triangle.emplace( t2 );
				auto const id1 = std::get< 0 >( s->second ), id2 = std::get< 1 >( s->second );
				auto const id1p1 = (id1 + 1) % 3, id1p2 = (id1 + 2) % 3, id2p1 = (id2 + 1) % 3, id2p2 = (id2 + 2) % 3;
				for( auto &v : { std::make_tuple( t1, t1->getOppositeTriangle( id1p1 ), id1p1 ), std::make_tuple( t1, t1->getOppositeTriangle( id1p2 ), id1p2 ), std::make_tuple( t2, t2->getOppositeTriangle( id2p1 ), id2p1 ), std::make_tuple( t2, t2->getOppositeTriangle( id2p2 ), id2p2 ) } )
				{
					auto const t = std::get< 0 >( v ), ot = std::get< 1 >( v );
					auto const id = std::get< 2 >( v );
					if( ot == nullptr ) continue;
					auto oid = ot->getIntraTriangleIndex( t );
					auto const pa = t->getCorner( id ), pb = t->getCorner( (id + 1) % 3 ), pc = ot->getCorner( oid ), pd = t->getCorner( (id + 2) % 3 );
					auto const va = VFP( pa ), vb = VFP( pb ), vc = VFP( pc ), vd = VFP( pd );
					auto const nvab = (vb - va).normarize(), nvad = (vd - va).normarize(), nvcb = (vb - vc).normarize(), nvcd = (vd - vc).normarize();
					// condition (i)
					auto const ipabcd = nvab.inner_product( nvcd ), ipadcb = nvad.inner_product( nvcb );
					auto const con1 = (1 - ipabcd * ipabcd < THETA_PALLA) && (1 - ipadcb * ipadcb < THETA_PALLA);
					// condition (ii)
					auto const ipadcd = nvad.inner_product( nvcd ), ipabcb = nvab.inner_product( nvcb );
					auto const con2 = (ipadcd * ipadcd < THETA_VERTI) && (1 - ipabcb * ipabcb < THETA_PALLA);
					if( (!con1 && !con2) || (con1 && con2) ) continue;
					auto const oot = ot->getOppositeTriangle( (oid + (con1 ? 1 : 2)) % 3 );
					auto nit = triangle_pair.find( sort_tptr( ot, oot ) );
					if( nit == triangle_pair.end() ) continue;
					queue.push( nit );
				}
			}
		}
	}
};

template< unsigned int WIDTH, unsigned int HEIGHT >
struct square_recognizer
{
	static_assert( WIDTH > 0 && HEIGHT > 0, "width and height must be larger than 0" );
	static_assert( WIDTH <= HEIGHT, "square_recognizer: needs width <= height" );

public:
	using prefilter = square_prefilter;

private:
	using Num_Tuple = meta::multi_tuple_t< WIDTH * HEIGHT, unsigned int >;

public:
	square_recognizer( std::vector< std::tuple< unsigned int, unsigned int, std::vector< unsigned int > > > const &data )
	{
		for( auto const &t : data )
		{
			auto const &objnum = std::get< 0 >( t );
			auto const &width = std::get< 1 >( t );
			auto const &num = std::get< 2 >( t );
			for()
		}
	}

	void recognize(
		std::vector< election > &elect,
		std::vector< cluster > const &clu,
		std::vector< GEOM_FADE2D::Triangle2 * > const &,
		prefilter const &pf
	) const
	{
	}
};

} // namespace recognition
