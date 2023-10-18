// recognition.cpp �̃f�o�b�O�p�⏕�֐�
//  recognition �̎������̂ɂ͕s�v�����C�f�o�b�O�ɂ���ƕ֗��Ȋ֐�

#include <opencv2/opencv.hpp>
#include "recognition.hpp"

namespace recognition
{

void draw_key_point( cv::Mat &frame_color, std::vector< key_point > const &kp )
{
	for( auto const &k : kp )
	{
		cv::circle( frame_color, cv::Point( static_cast< int >( k.x ), static_cast< int >( k.y ) ), 2, cv::Scalar( 0, 0, 255 ), -1 );
	}
}

void draw_cluster( cv::Mat &frame_color, std::vector< cluster > const &clu )
{
	for( auto const &c : clu )
	{
		int const x = static_cast< int >( c.x ), y = static_cast< int >( c.y );
		auto e = [ & ]( auto sc ){ cv::circle( frame_color, cv::Point( x, y ), 10, sc, 1 ); };
		switch( c.num )
		{
		case 0: break;
		case 1: e( cv::Scalar( 0, 0, 255 ) ); break;
		case 2: e( cv::Scalar( 0, 255, 0 ) ); break;
		case 3: e( cv::Scalar( 255, 0, 0 ) ); break;
		case 4: e( cv::Scalar( 0, 255, 255 ) ); break;
		case 5: e( cv::Scalar( 255, 0, 255 ) ); break;
		case 6: e( cv::Scalar( 255, 255, 0 ) ); break;
		case 7: e( cv::Scalar( 0, 0, 128 ) ); break;
		default: e( cv::Scalar( 0, 0, 0 ) ); break;
		}
	}
}

void draw_triangle( cv::Mat &frame_color, std::vector< GEOM_FADE2D::Triangle2 * > const &triangle )
{
	for( auto const p : triangle )
	{
		auto const c0 = p->getCorner( 0 ), c1 = p->getCorner( 1 ), c2 = p->getCorner( 2 );
		cv::Point const p0( static_cast< int >( c0->x() ), static_cast< int >( -c0->y() ) );
		cv::Point const p1( static_cast< int >( c1->x() ), static_cast< int >( -c1->y() ) );
		cv::Point const p2( static_cast< int >( c2->x() ), static_cast< int >( -c2->y() ) );
		cv::line( frame_color, p0, p1, cv::Scalar( 64, 255, 128 ) );
		cv::line( frame_color, p1, p2, cv::Scalar( 64, 255, 128 ) );
		cv::line( frame_color, p2, p0, cv::Scalar( 64, 255, 128 ) );
	}
}

void draw_cluster_index( cv::Mat &frame_color, std::vector< Cluster_Index > const &index, std::vector< cluster > const &clu )
{
	for( auto i = 0u; i < std::size( index ); ++i )
	{
		auto const &ci = index[ i ];
		auto const &cd = clu[ i ];
		auto const &pno = std::get< 0 >( ci );
		auto const &ind = std::get< 1 >( ci );
		if( pno == INVALID_INDEX ) continue;
		cv::Point const cvp( static_cast< int >( cd.x ), static_cast< int >( cd.y ) );
		cv::circle( frame_color, cvp, 10, cv::Scalar( 0, 0, 255 ), -1 );
		cv::putText( frame_color, std::to_string( pno ) + "," + std::to_string( ind ), cvp, cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar( 0, 255, 0 ) );
	}
}

} // namespace recognition