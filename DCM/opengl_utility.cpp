#include <GL/glew.h>
#include <GL/gl.h>
#include <vector>
#include <iostream>
#include "opengl_utility.hpp"

namespace ogl
{
	GLuint compile_shader( char const *vertex_shader_src, char const *fragment_shader_src )
	{
		GLint result, log_length;
		std::vector< char > log_buff;

		GLuint vertex_shader = glCreateShader( GL_VERTEX_SHADER );
		char const *pvss = vertex_shader_src;
		glShaderSource( vertex_shader, 1, &pvss, nullptr );
		glCompileShader( vertex_shader );
		glGetShaderiv( vertex_shader, GL_COMPILE_STATUS, &result );
		if( result == GL_FALSE )
		{
			glGetShaderiv( vertex_shader, GL_INFO_LOG_LENGTH, &log_length );
			if( log_length > 0 )
			{
				log_buff.resize( log_length );
				glGetShaderInfoLog( vertex_shader, log_length, nullptr, &log_buff[ 0 ] );
				std::clog << &log_buff[ 0 ] << std::endl;
			}
		}

		GLuint fragment_shader = glCreateShader( GL_FRAGMENT_SHADER );
		char const *pfss = fragment_shader_src;
		glShaderSource( fragment_shader, 1, &pfss, nullptr );
		glCompileShader( fragment_shader );
		glGetShaderiv( fragment_shader, GL_COMPILE_STATUS, &result );
		if( result == GL_FALSE )
		{
			glGetShaderiv( fragment_shader, GL_INFO_LOG_LENGTH, &log_length );
			if( log_length > 0 )
			{
				log_buff.resize( log_length );
				glGetShaderInfoLog( fragment_shader, log_length, nullptr, &log_buff[ 0 ] );
				std::clog << &log_buff[ 0 ] << std::endl;
			}
		}

		GLuint program = glCreateProgram();
		glAttachShader( program, vertex_shader );
		glAttachShader( program, fragment_shader );
		glLinkProgram( program );
		glGetProgramiv( program, GL_LINK_STATUS, &result );
		if( result == GL_FALSE )
		{
			glGetProgramiv( program, GL_INFO_LOG_LENGTH, &log_length );
			if( log_length > 0 )
			{
				log_buff.resize( log_length );
				glGetProgramInfoLog( program, log_length, nullptr, &log_buff[ 0 ] );
				std::clog << &log_buff[ 0 ] << std::endl;
			}
		}
		glDeleteShader( vertex_shader );
		glDeleteShader( fragment_shader );

		return program;
	}
}