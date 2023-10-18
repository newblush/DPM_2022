#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <fstream>

static
cv::Mat load_camera_matrix( std::string const &filepath )
{
	cv::Mat ret( 3, 3, CV_32FC1 );
	cv::Mat_< float > t = ret;
	std::ifstream ifs( filepath );
	if( !ifs ) throw std::runtime_error( "load_camera_matrix error" );
	ifs >>
		t( 0, 0 ) >> t( 0, 1 ) >> t( 0, 2 ) >>
		t( 1, 0 ) >> t( 1, 1 ) >> t( 1, 2 ) >>
		t( 2, 0 ) >> t( 2, 1 ) >> t( 2, 2 );
	return std::move( ret );
}
static
cv::Mat load_camera_coeffs( std::string const &filepath )
{
	cv::Mat ret( 1, 5, CV_32FC1 );
	cv::Mat_< float > t = ret;
	std::ifstream ifs( filepath );
	if( !ifs ) throw std::runtime_error( "load_camera_coeffs error" );
	ifs >>
		t( 0 ) >> t( 1 ) >> t( 2 ) >> t( 3 ) >> t( 4 );
	return std::move( ret );
}

static
std::vector< cv::Mat > load_images( std::string const &base_name )
{
    std::vector< cv::Mat > vec;
	std::vector< char > path( 255 );
	for( unsigned int i = 0; ; ++i )
	{
		while( true )
		{
			int const num = std::snprintf( &path[ 0 ], path.size(), base_name.c_str(), i );
			if( num < 0 ) break;
			if( static_cast< std::size_t >( num ) < path.size() ) break;
			path.resize( path.size() * 2 );
		}
		cv::Mat img = cv::imread( &path[ 0 ] );
		if( img.empty() )
		{
			if( vec.size() == 0 )
			{
				if( i > 1000 ) break;
			}
			else
			{
				break;
			}
			continue;
		}
		if( img.channels() == 3)
		{
			cv::cvtColor( img, img, cv::COLOR_BGR2GRAY );
		}
		vec.emplace_back( std::move( img ) );
	}
	return std::move( vec );
}