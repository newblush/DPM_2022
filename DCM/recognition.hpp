#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <memory>
#include <Fade_2D.h>
#include <limits>
#include <utility>
#include <tuple>
#include <array>
#include "tracking_data.hpp"

namespace recognition
{
	struct key_point
	{
		float x, y;
		float sx, sy;
	};
	struct cluster
	{
		float x, y;
		float sx, sy;
		unsigned int num;
	};

	constexpr auto INVALID_INDEX = std::numeric_limits< unsigned int >::max();
	using Cluster_Index = std::tuple< unsigned int, unsigned int >;
	Cluster_Index const INVALID_CLUSTER_INDEX = { INVALID_INDEX, INVALID_INDEX };
	class election
	{
	private:
		static constexpr unsigned int MAX_ELECTION = 12u;

		std::array< Cluster_Index, MAX_ELECTION > elect;
		std::size_t elect_num;
		
	public:
		election()
			: elect_num( 0u )
		{
		}
		void vote( Cluster_Index const &c )
		{
			if( elect_num < MAX_ELECTION ) elect[ elect_num++ ] = c;
			else std::clog << "ELECTION EXCEEDED!" << std::endl;
		}
		// in recognition.cpp
		Cluster_Index elected( void );
	};

	// in recognition.cpp
	void get_key_point( std::vector< key_point > &ret, cv::Mat &frame_gray, float const area_threshold_min, float const area_threshold_max );
	void get_cluster( std::vector< cluster > &ret, std::vector< key_point > const &kp, float const keypoint_distance_threshold, std::size_t const key_point_grid_num_threshold, unsigned int const max_num_of_key_point_in_cluster, unsigned int const image_width, unsigned int const image_height );
	void get_triangulation( std::vector< GEOM_FADE2D::Triangle2 * > &triangle, std::unique_ptr< GEOM_FADE2D::Fade_2D > &fade2d, std::vector< cluster > const &clu );
	void filter_triangle( std::vector< GEOM_FADE2D::Triangle2 * > &triangle );
	void get_cluster_index( std::vector< Cluster_Index > &cluster_index, std::vector< election > &elect );
	void update_tracking_data( tracking_data &tdata, std::vector< Cluster_Index > const &cluster_index, std::vector< cluster > const &clus );
	
	// in recognition_debug.cpp
	void draw_key_point( cv::Mat &frame_color, std::vector< key_point > const &kp );
	void draw_cluster( cv::Mat &frame_color, std::vector< cluster > const &clu );
	void draw_triangle( cv::Mat &frame_color, std::vector< GEOM_FADE2D::Triangle2 * > const &triangle );
	void draw_cluster_index( cv::Mat &frame_color, std::vector< Cluster_Index > const &index, std::vector< cluster > const &clu );

} // namespace recognition
