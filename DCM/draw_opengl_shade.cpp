#include <iostream>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <iomanip>
#include <GL/glew.h>
#include <GL/gl.h>
#include <GLFW/glfw3.h>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "draw_opengl_shade.hpp"
#include "opengl_utility.hpp"
#include "interval_timer.hpp"
#include "make_point.hpp"

#ifdef _MSC_VER
#pragma comment( lib, "glew32" )
#pragma comment( lib, "glfw3" )
#pragma comment( lib, "OpenGL32.lib" )
#pragma comment( lib, "glu32.lib" )
// #pragma comment( lib, "HighSpeedProjectorLibrary")
#endif


namespace
{
	constexpr char vertex_shader_src[] =
R"(#version 330 core

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;
uniform mat4 Hmat;
uniform mat3 Rmat;
out vec3 tmp_normal;
out vec3 tmp_position;

void main( void )
{
	gl_Position = Hmat * vec4( position.xyz, 1.0 );
	tmp_position = gl_Position.xyz;
	tmp_normal = Rmat * normal;
}
)";
	constexpr char fragment_shader_src1[] =
R"(#version 330 core

in vec3 tmp_normal;
in vec3 tmp_position;
out vec4 color;
uniform vec3 Lvec;

void main( void )
{
	// float v = -dot( tmp_normal, vec3( 0.577, 0.577, -0.577 ) );
	vec3 pos_normalize = normalize( tmp_position );
	vec3 reflect = Lvec - 2 * dot( Lvec, tmp_normal ) * tmp_normal;
	float reflect_tmp = clamp( dot( reflect, -pos_normalize ), 0.0, 1.0 );
	float defuse_tmp = clamp( -dot( tmp_normal, Lvec ), 0.0, 1.0 );
	float v = defuse_tmp / 2 + reflect_tmp * reflect_tmp;
	color = vec4( v, 0.08*v, 0.58*v, 1.0 );
	// color = vec4( 1.0, 0.08*v, 0.58*v, 1.0 );
	// color = texture( tex1, outuv );
}
)";

	constexpr char fragment_shader_src2[] =
		R"(#version 330 core

in vec3 tmp_normal;
in vec3 tmp_position;
out vec4 color;
uniform vec3 Lvec;

void main( void )
{
	// float v = -dot( tmp_normal, vec3( 0.577, 0.577, -0.577 ) );
	vec3 pos_normalize = normalize( tmp_position );
	vec3 reflect = Lvec - 2 * dot( Lvec, tmp_normal ) * tmp_normal;
	float reflect_tmp = clamp( dot( reflect, -pos_normalize ), 0.0, 1.0 );
	float defuse_tmp = clamp( -dot( tmp_normal, Lvec ), 0.0, 1.0 );
	float v1 = defuse_tmp / 2 + reflect_tmp * reflect_tmp;
	reflect = -Lvec - 2 * dot( Lvec, tmp_normal ) * tmp_normal;
	reflect_tmp = clamp( dot( reflect, -pos_normalize ), 0.0, 1.0 );
	defuse_tmp = clamp( -dot( tmp_normal, -Lvec ), 0.0, 1.0 );
	float v2 = defuse_tmp / 2 + reflect_tmp * reflect_tmp;
	color = vec4( v1, 0, v2, 1.0 );
	// color = vec4( 1.0, 0.08*v, 0.58*v, 1.0 );
	// color = texture( tex1, outuv );
}
)";
	
	constexpr auto P_WIDTH = 720u, P_HEIGHT = 540u;
}


cv::Mat get_ocv_img_from_gl_img(GLuint ogl_texture_id)
{
	glBindTexture(GL_TEXTURE_2D, ogl_texture_id);
	GLenum gl_texture_width, gl_texture_height;

	glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, (GLint*)&gl_texture_width);
	glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, (GLint*)&gl_texture_height);

	unsigned char* gl_texture_bytes = (unsigned char*)malloc(sizeof(unsigned char) * gl_texture_width * gl_texture_height * 3);
	glGetTexImage(GL_TEXTURE_2D, 0 /* mipmap level */, GL_BGR, GL_UNSIGNED_BYTE, gl_texture_bytes);

	return cv::Mat(gl_texture_height, gl_texture_width, CV_8UC3, gl_texture_bytes);
}

bool draw_opengl_shade::wait_update()
{
	while( last_tracking_index == tdata_vector[ objdata_index.load() ].tracking_frame.load() )
	{
		if( exit_flag.load() ) return false;
		// std::this_thread::yield();
	}
	return true;
}

void key_callback( GLFWwindow *window, int key, int scancode, int action, int mods )
{
	extern std::atomic< bool > g_please_change_objdata_index;
	if( key == GLFW_KEY_S && action == GLFW_PRESS )
	{
		g_please_change_objdata_index.store( true, std::memory_order::memory_order_release );
	}
}

void draw_opengl_shade::thread_func()
{
	auto const ISINDEX_SIZE = tdata_vector.size();
	std::vector< std::size_t > NUM_OF_OBJECT_vector;
	for( auto const &tdata : tdata_vector)
	{
		NUM_OF_OBJECT_vector.push_back( tdata.point_size.size() );
	}

	if( glfwInit() != GLFW_TRUE )
	{
		std::cerr << "GLFW init Error!!" << std::endl;
		return;
	}
	std::clog << "GLFW init OK!" << std::endl;

	GLFWwindow *window = glfwCreateWindow( P_WIDTH, P_HEIGHT, "rendering", nullptr, nullptr );
	// glfwSetWindowPos( window, 0, 0 );
	glfwSetKeyCallback( window, key_callback );
	glfwMakeContextCurrent( window );
	glfwSwapInterval( 0 );

	glewExperimental = GL_TRUE;
	if( glewInit() != GLEW_OK )
	{
		std::cerr << "GLEW init Error!!" << std::endl;
		glfwTerminate();
		return;
	}
	std::clog << "GLEW init OK!" << std::endl;
	glViewport( 0, 0, P_WIDTH, P_HEIGHT );
	glEnable( GL_DEPTH_TEST );



	glm::mat4 proj = glm::perspective( glm::radians( 22.5f ), static_cast< float >( P_WIDTH ) / P_HEIGHT, 1.0f, 1000.0f );
	/*/
	float _near = 1.0f;
	float _far = 1000.0f;
	std::cout<< cameraMatrix<<std::endl;
	float fx = cameraMatrix.at<double>(0, 0);
	float fy = cameraMatrix.at<double>(1, 1);
	float cx = cameraMatrix.at<double>(0, 2);
	float cy = cameraMatrix.at<double>(1, 2);
	glm::mat4 proj = {fx / cx, 0, 0, 0,
		0, fy / cy, 0, 0,
		0, 0, -(_far + _near) / (_far - _near), -(2 * _far * _near) / (_far - _near),
		0, 0, -1, 0};
	proj = glm::transpose(proj);//*/
	//glm::mat4 proj = glm::ortho(0, (int)P_WIDTH /2, 0, (int)P_HEIGHT/2);
	glm::mat4 view = glm::lookAt( glm::vec3( 0.025f, 0.01f, -1.0f ), glm::vec3(0.0f, 0.0f, 0.0f ), glm::vec3( 0.0f, -1.0f, 0.0f ) );
	

	auto const program1 = ogl::compile_shader( vertex_shader_src, fragment_shader_src2 );

	std::vector< std::vector< std::vector< cv::Point3f > > > objpoint_vector_vector( ISINDEX_SIZE );
	std::vector< std::vector< std::vector< cv::Point2f > > > imagepoint_vector_vector( ISINDEX_SIZE );
	std::vector< std::vector< std::vector< unsigned int > > > objid_vector_vector( ISINDEX_SIZE );
	std::vector< std::vector< GLuint > > point_buffer_vector_vector( ISINDEX_SIZE );
	std::vector< std::vector< GLuint > > index_buffer_vector_vector( ISINDEX_SIZE );
	std::vector< std::vector< GLuint > > normal_buffer_vector_vector( ISINDEX_SIZE );
	for( std::size_t isi = 0u; isi < ISINDEX_SIZE; ++isi )
	{
		auto const NUM_OF_OBJECT = NUM_OF_OBJECT_vector[ isi ];
		std::vector< std::vector< cv::Point3f > > objpoint_vector( NUM_OF_OBJECT );
		std::vector< std::vector< cv::Point2f > > imagepoint_vector( NUM_OF_OBJECT );
		std::vector< std::vector< unsigned int > > objid_vector( NUM_OF_OBJECT );
		std::vector< GLuint > point_buffer_vector( NUM_OF_OBJECT );
		std::vector< GLuint > index_buffer_vector( NUM_OF_OBJECT );
		std::vector< GLuint > normal_buffer_vector( NUM_OF_OBJECT );
		for( auto i = 0u; i < NUM_OF_OBJECT; ++i )
		{
			auto const num = tdata_vector[ isi ].point_size[ i ];
			objpoint_vector.reserve( num );
			imagepoint_vector.reserve( num );
			objid_vector.reserve( num );
			point_buffer_vector[ i ] = ogl::make_gl_buffer( GL_ARRAY_BUFFER, GL_STATIC_DRAW, draw_point_vector_vector[ isi ][ i ] );
			index_buffer_vector[ i ] = ogl::make_gl_buffer( GL_ELEMENT_ARRAY_BUFFER, GL_STATIC_DRAW, draw_index_vector_vector[ isi ][ i ] );
			normal_buffer_vector[ i ] = ogl::make_gl_buffer( GL_ARRAY_BUFFER, GL_STATIC_DRAW, draw_normal_vector_vector[ isi ][ i ] );
		}
		objpoint_vector_vector[ isi ] = objpoint_vector;
		imagepoint_vector_vector[ isi ] = imagepoint_vector;
		objid_vector_vector[ isi ] = objid_vector;
		point_buffer_vector_vector[ isi ] = point_buffer_vector;
		index_buffer_vector_vector[ isi ] = index_buffer_vector;
		normal_buffer_vector_vector[ isi ] = normal_buffer_vector;
	}

	GLuint pbo[ 2 ];
	glGenBuffers( 2, pbo );
	for( auto id : pbo )
	{
		glBindBuffer( GL_PIXEL_PACK_BUFFER, id );
		glBufferData( GL_PIXEL_PACK_BUFFER, P_WIDTH * P_HEIGHT * sizeof( GLubyte ) * 3u, nullptr, GL_DYNAMIC_READ );
	}

	if( !wait_update() ) return;

	interval_timer calc_fps( "draw_opengl_shade" );

	static unsigned int frame_num = 0u;
	while( !exit_flag.load() && !glfwWindowShouldClose( window ) )
	{
		if( !wait_update() ) break;
		
		auto const isi = objdata_index.load( std::memory_order::memory_order_acquire );
		auto const NUM_OF_OBJECT = NUM_OF_OBJECT_vector[ isi ];
		auto &objpoint_vector = objpoint_vector_vector[ isi ];
		auto &imagepoint_vector = imagepoint_vector_vector[ isi ];
		auto &objid_vector = objid_vector_vector[ isi ];
		auto &point_buffer_vector = point_buffer_vector_vector[ isi ];
		auto &index_buffer_vector = index_buffer_vector_vector[ isi ];
		auto &normal_buffer_vector = normal_buffer_vector_vector[ isi ];
		auto &point_vector = point_vector_vector[ isi ];
		auto &draw_point_vector = draw_point_vector_vector[ isi ];
		auto &draw_index_vector = draw_index_vector_vector[ isi ];
		auto &draw_normal_vector = draw_normal_vector_vector[ isi ];
		auto &tdata = tdata_vector[ isi ];

		for( auto i = 0u; i < NUM_OF_OBJECT; ++i )
		{
			objpoint_vector[ i ].clear();
			imagepoint_vector[ i ].clear();
			objid_vector[ i ].clear();
		}

		last_tracking_index = tdata.tracking_frame.load();

		for( auto i = 0u; i < NUM_OF_OBJECT; ++i )
		{
			tracking_data_point_get_tracking_points( tdata.point[ i ].get(), tdata.point_size[ i ], objid_vector[ i ], imagepoint_vector[ i ] );
		}

		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
		for( auto i = 0u; i < NUM_OF_OBJECT; ++i )
		{
			auto &imagepoint = imagepoint_vector[ i ];
			auto const imagepoint_size = imagepoint.size();
			if( imagepoint_size >= 3 ) 
			{
				auto &objpoint = objpoint_vector[ i ];
				auto &objid = objid_vector[ i ];
				for( auto const id : objid )
				{
					auto const * const pp = &point_vector[ i ][ id * 3 ];
					objpoint.emplace_back( pp[ 0 ], pp[ 1 ], pp[ 2 ] );
				}
	
				cv::Mat rvec, tvec, rmat;
				std::vector< cv::Point2f > projectedPoint;
				bool const solvePnP_ret = cv::solvePnP( objpoint, imagepoint, cameraMatrix, cameraCoeffs, rvec, tvec, false );
				cv::Rodrigues( rvec, rmat );
				
				//*/
				glm::mat4 mvp = proj * view * glm::mat4(
					static_cast< float >( rmat.at< double >( 0, 0 ) ), static_cast< float >( rmat.at< double >( 1, 0 ) ), static_cast< float >( rmat.at< double >( 2, 0 ) ), 0.0f,
					static_cast< float >( rmat.at< double >( 0, 1 ) ), static_cast< float >( rmat.at< double >( 1, 1 ) ), static_cast< float >( rmat.at< double >( 2, 1 ) ), 0.0f,
					static_cast< float >( rmat.at< double >( 0, 2 ) ), static_cast< float >( rmat.at< double >( 1, 2 ) ), static_cast< float >( rmat.at< double >( 2, 2 ) ), 0.0f,
					static_cast< float >( tvec.at< double > ( 0 ) ), static_cast< float >( tvec.at< double > ( 1 ) ), static_cast< float >( tvec.at< double > ( 2 ) ), 1.0f
				);//*/
				/*/
				glm::mat4 mvp = proj *  glm::mat4(
					static_cast<float>(rmat.at< double >(0, 0)), static_cast<float>(rmat.at< double >(0, 1)), static_cast<float>(rmat.at< double >(0, 2)), static_cast<float>(tvec.at< double >(0)),
					static_cast<float>(-rmat.at< double >(1, 0)), static_cast<float>(-rmat.at< double >(1, 1)), static_cast<float>(-rmat.at< double >(1, 2)), static_cast<float>(-tvec.at< double >(1)),
					static_cast<float>(-rmat.at< double >(2, 0)), static_cast<float>(-rmat.at< double >(2, 1)), static_cast<float>(-rmat.at< double >(2, 2)), static_cast<float>(-tvec.at< double >(2)),
					0.0f, 0.0f, 0.0f, 1.0f
				)* glm::mat4(1.0f);//*/
				glm::mat3 Rmat(
					static_cast< float >( rmat.at< double >( 0, 0 ) ), static_cast< float >( rmat.at< double >( 1, 0 ) ), static_cast< float >( rmat.at< double >( 2, 0 ) ),
					static_cast< float >( rmat.at< double >( 0, 1 ) ), static_cast< float >( rmat.at< double >( 1, 1 ) ), static_cast< float >( rmat.at< double >( 2, 1 ) ),
					static_cast< float >( rmat.at< double >( 0, 2 ) ), static_cast< float >( rmat.at< double >( 1, 2 ) ), static_cast< float >( rmat.at< double >( 2, 2 ) )
				);
	
				glUseProgram( program1 );
				glUniformMatrix4fv( glGetUniformLocation( program1, "Hmat" ), 1, GL_FALSE, &mvp[ 0 ][ 0 ] );
				glUniformMatrix3fv( glGetUniformLocation( program1, "Rmat" ), 1, GL_FALSE, &Rmat[ 0 ][ 0 ] );
				glUniform3f( glGetUniformLocation( program1, "Lvec"), 0.577f, 0.577f, 0.577f );
	
				glEnableClientState( GL_VERTEX_ARRAY );
	
				glEnableVertexAttribArray( 0 );
				glBindBuffer( GL_ARRAY_BUFFER, point_buffer_vector[ i ] );
				glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, 0, reinterpret_cast< void * >( 0 ) );
	
				glEnableVertexAttribArray( 1 );
				glBindBuffer( GL_ARRAY_BUFFER, normal_buffer_vector[ i ] );
				glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, 0, reinterpret_cast< void * >( 0 ) );
	
				glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, index_buffer_vector[ i ] );
				glDrawElements( GL_TRIANGLES, static_cast< GLsizei >( draw_index_vector[ i ].size() ), GL_UNSIGNED_INT, reinterpret_cast< void * >( 0 ) );
				glBindBuffer( GL_ELEMENT_ARRAY_BUFFER, 0 );

				glDisableClientState( GL_VERTEX_ARRAY );
	
			}
		}
		glFlush();
		glfwSwapBuffers( window );

		cv::Mat img(P_HEIGHT, P_WIDTH, CV_8UC3);
		cv::Mat Homography=(cv::Mat_<double>(3, 3) << 0.65800607, -0.0030378532, 184.76537,
		0.040029012, 0.6231873, 483.24985,
		6.4203676e-05, -9.1431775e-06, 1
			);

		glBindBuffer( GL_PIXEL_PACK_BUFFER, pbo[ frame_num % 2 ] );
		glReadPixels( 0, 0, P_WIDTH, P_HEIGHT, GL_BGR, GL_UNSIGNED_BYTE, nullptr);
	
		glBindBuffer( GL_PIXEL_PACK_BUFFER, pbo[ (frame_num + 1) % 2 ]);
		void const *ptr = glMapBuffer( GL_PIXEL_PACK_BUFFER, GL_READ_ONLY );
	

		(void)ptr; // ここでプロジェクターに出力する
		memcpy(&img.data[0],ptr,P_HEIGHT*P_WIDTH*3);
		cv::flip(img, img, 0);
		cv::Mat img2(768,1024,CV_8UC3);
		cv::warpPerspective(img,img2,Homography,cv::Size(1024,768));
		//cv::flip(img2, img2, -1);

		cv::imshow("cv", img2);
		cv::waitKey(1);
		proj_v3.sendImage(img2.data);


				
		glUnmapBuffer( GL_PIXEL_PACK_BUFFER );

		/*
		if( frame_num % 250 == 0 )
		{
			glfwFocusWindow( window );
		}
		*/
		glfwPollEvents();
	

		

		++frame_num;
		calc_fps.interval();
	}

	for( std::size_t isi = 0u; isi < ISINDEX_SIZE; ++isi )
	{
		auto &NUM_OF_OBJECT = NUM_OF_OBJECT_vector[ isi ];
		auto &point_buffer_vector = point_buffer_vector_vector[ isi ];
		auto &index_buffer_vector = index_buffer_vector_vector[ isi ];
		auto &normal_buffer_vector = normal_buffer_vector_vector[ isi ];

		glDeleteBuffers( static_cast< GLsizei >( NUM_OF_OBJECT ), &point_buffer_vector[ 0 ] );
		glDeleteBuffers( static_cast< GLsizei >( NUM_OF_OBJECT ), &index_buffer_vector[ 0 ] );
		glDeleteBuffers( static_cast< GLsizei >( NUM_OF_OBJECT ), &normal_buffer_vector[ 0 ] );
	}

	glfwTerminate();
}
