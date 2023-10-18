#include <opencv2/opencv.hpp>
#include <sstream>
#include <vector>
#include <iomanip>
#include <fstream>
#include <string>
#include <chrono>
#include "capture_class.hpp"

/*
#if __has_include
#  if __has_include( <filesystem> )
#    include <filesystem>
namespace fs = std::filesystem;
#  elif __has_include( <experimental/filesystem> )
#    include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#  endif
#else
#  include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif
*/

#include <filesystem>
namespace fs = std::filesystem;

#ifdef _MSC_VER
#if !_DEBUG
#pragma comment( lib, "opencv_world310" )
#else
#pragma comment( lib, "opencv_world310d" )
#endif
#endif

//using capture_class = basler_capture;

void save_calib_data( fs::path const &folder_path, std::vector< std::vector< cv::Point2f > > const &coner_vec, cv::Size const &checkerBoardSize, cv::Size const &img_size, float const firstLength, float const secondLength )
{
	std::vector< std::vector< cv::Point3f > > worldPoint;
	for( auto p = 0; p < coner_vec.size(); ++p )
	{
		std::vector< cv::Point3f > tmp;
		for( auto j = 0; j < checkerBoardSize.height; ++j )
		{
			for( auto i = 0; i < checkerBoardSize.width; ++i )
			{
				tmp.emplace_back( firstLength * i, secondLength * j, 0.0f );
			}
		}
		worldPoint.emplace_back( std::move( tmp ) );
	}
	cv::Mat cameraMatrix, distCoeffs;
	std::vector< cv::Mat > rvecs, tvecs;
	cv::calibrateCamera( worldPoint, coner_vec, img_size, cameraMatrix, distCoeffs, rvecs, tvecs);

	auto time = std::to_string( std::chrono::duration_cast< std::chrono::seconds >( std::chrono::system_clock::now().time_since_epoch() ).count() );
	{
		std::ofstream fs_camera( folder_path / "cameraMatrix.dat" );
		assert( cameraMatrix.rows == 3 && cameraMatrix.cols == 3 );
		cv::Mat_< double > cm = cameraMatrix;
		fs_camera <<
			cm( 0, 0 ) << " " << cm( 0, 1 ) << " " << cm( 0, 2 ) << "\n" <<
			cm( 1, 0 ) << " " << cm( 1, 1 ) << " " << cm( 1, 2 ) << "\n" <<
			cm( 2, 0 ) << " " << cm( 2, 1 ) << " " << cm( 2, 2 ) << "\n";
		fs_camera << std::flush;
	}
	{
		std::ofstream fs_coeffs( folder_path / "cameraCoeffs.dat" );
		assert( distCoeffs.rows == 1 );
		for( auto i = 0; i < distCoeffs.cols; ++i )
		{
			fs_coeffs << (i == 0 ? "" : " ") << distCoeffs.at< double >( i );
		}
		fs_coeffs << std::endl;
	}

}

int main()
{
	
	basler_capture cap;

	std::cout <<
R"(commands on window (not on console)
c: capture
d: delete last capture
s: calc and save calib parms
ESC: quit
)"
	<< std::flush;

	auto time = std::to_string( std::chrono::duration_cast< std::chrono::seconds >( std::chrono::system_clock::now().time_since_epoch() ).count() );
	fs::path folder_path( "output_" + time );
	fs::create_directories( folder_path );

	std::cout << "output folder: " << folder_path << std::endl;

	unsigned int num = 0;
	cv::Mat img;
	std::vector< std::vector< cv::Point2f > > coner_vec;
	cv::Size const checkerBoardSize( 4, 5 );
	float const firstLength = 87.5f / 5, secondLength = 105.0f / 6; // チェッカーボードのそれぞれの長さ

	bool flag = true;
	std::thread thr([&] {
		while (flag) {
			cap.capture(img);
		}
	});

	while( true )
	{
		if(!img.data) continue;
		cv::imshow( "img", img );

		auto key = cv::waitKey( 1 );
		if( key == 27 ) break; // ESC
		switch( key )
		{
		case 'c':
			{
				auto show = img.clone();
				cv::cvtColor( show, show, cv::COLOR_GRAY2BGR );
				std::vector< cv::Point2f > coner_tmp;
				bool const fcc = cv::findChessboardCorners( img, checkerBoardSize, coner_tmp );
				std::cout << "findChessboardCorners: " << std::boolalpha << fcc << std::endl;
				if( fcc )
				{
					bool const fqcs = cv::find4QuadCornerSubpix( img, coner_tmp, cv::Size( 4, 4 ) );
					std::cout << "find4QuadCornerSubpix: " << std::boolalpha << fqcs << std::endl;
					if( fqcs )
					{
						cv::drawChessboardCorners( show, checkerBoardSize, coner_tmp, true );

						std::ostringstream oss;
						oss << "img_" << std::setw( 3u ) << std::setfill( '0' ) << num++ << ".png";
						cv::imwrite( (folder_path / oss.str()).string(), img );
						std::cout << oss.str() << ": 保存しました" << std::endl;
						coner_vec.emplace_back( std::move( coner_tmp ) );
					}
				}
				cv::imshow( "last captured", show );
			}
			break;
		case 'd':
			{
				num--;
				coner_vec.pop_back();
				std::cout << "一枚戻した" << std::endl;
			}
			break;
		case 's':
			{
				std::cout << "計算開始しました" << std::endl;
				save_calib_data( folder_path, coner_vec, checkerBoardSize, img.size(), firstLength, secondLength );
				std::cout << "出力しました" << std::endl;
			}
			break;
		}
	}
	flag = false;
	if (thr.joinable())thr.join();
}