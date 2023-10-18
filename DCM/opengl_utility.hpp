#pragma once

#include <GL/glew.h>
#include <GL/gl.h>
#include <vector>

namespace ogl
{
	GLuint compile_shader( char const *vertex_shader_src, char const *fragment_shader_src );

	template< typename T, typename Alloc >
	GLuint make_gl_buffer( GLenum const type, GLenum const usage, std::vector< T, Alloc > const &vec )
	{
		GLuint id;
		glGenBuffers( 1, &id );
		glBindBuffer( type, id );
		auto const size = std::size( vec );
		glBufferData( type, sizeof( vec[ 0 ] ) * size, size ? &vec[ 0 ] : nullptr, usage );
		return id;
	}
} // namespace ogl