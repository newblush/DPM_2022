#pragma once

#include <utility>
#include <tuple>
#include "compier_attribute.hpp"

namespace meta
{

	template< typename T >
	static constexpr
	std::add_const_t< T > &as_const( T &t ) noexcept
	{
		return t;
	}
	template< typename T >
	static
	void as_const( T const && ) = delete;

	template< typename Bool, typename... Rest >
	constexpr
	FORCE_INLINE
	std::enable_if_t<
		std::is_same< Bool, bool >::value, bool
	> _and( Bool b, Rest... rest )
	{
		return b && _and( rest... );
	}
	template< typename T = void >
	constexpr
	FORCE_INLINE
	bool _and( void )
	{
		return true;
	}
	template< typename... Bool >
	constexpr
	FORCE_INLINE
	bool _or( Bool... args )
	{
		return !_and( (!args)... );
	}
	template< typename... Type >
	constexpr
	FORCE_INLINE
	void nothing( Type &&... )
	{
	}

	template< typename Type1, typename Type2 >
	struct cat_tuple;
	template< typename... Types1, typename... Types2 >
	struct cat_tuple< std::tuple< Types1... >, std::tuple< Types2... > >
	{
		using type = std::tuple< Types1..., Types2... >;
	};
	template< typename Type1, typename Type2 >
	using cat_tuple_t = typename cat_tuple< Type1, Type2 >::type;

	template< std::size_t num, typename Type >
	struct multi_tuple
	{
		using type = cat_tuple_t< typename multi_tuple< num / 2u, Type >::type, typename multi_tuple< num - num / 2u, Type >::type >;
	};
	template< typename Type >
	struct multi_tuple< 1u, Type >
	{
		using type = std::tuple< Type >;
	};
	template< std::size_t num, typename Type >
	using multi_tuple_t = typename multi_tuple< num, Type >::type;

	namespace detail
	{
		template< std::size_t I, typename Type, typename Tuple, bool Flag >
		struct is_in_tuple_impl
			: std::integral_constant<
				bool,
				std::is_same<
					std::tuple_element_t< I, Tuple >,
					Type
				>::value ||
				is_in_tuple_impl< I + 1u, Type, Tuple, I + 1u < std::tuple_size< Tuple >::value >::value
			>
		{
		};
		template< std::size_t I, typename Type, typename Tuple >
		struct is_in_tuple_impl< I, Type, Tuple, false >
			: std::integral_constant< bool, false >
		{
		};
	}
	template< typename Type, typename Tuple >
	struct is_in_tuple : detail::is_in_tuple_impl< 0u, Type, Tuple, 0 < std::tuple_size< Tuple >::value >
	{
	};

	namespace detail
	{
		template< std::size_t I, typename Tuple_c, typename Tuple_r, bool Flag >
		struct unique_tuple_impl
		{
			using type =
				typename unique_tuple_impl<
					I + 1u,
					std::conditional_t<
						is_in_tuple<
							std::tuple_element_t< I, Tuple_r >,
							Tuple_c
						>::value,
						Tuple_c,
						cat_tuple_t<
							Tuple_c,
							std::tuple<
								std::tuple_element_t< I, Tuple_r >
							>
						>
					>,
					Tuple_r,
					I + 1u < std::tuple_size< Tuple_r >::value
				>::type;
		};
		template< std::size_t I, typename Tuple_c, typename Tuple_r >
		struct unique_tuple_impl< I, Tuple_c, Tuple_r, false >
		{
			using type = Tuple_c;
		};
	}
	template< typename... Types >
	struct unique_tuple
	{
		using type = typename detail::unique_tuple_impl< 0u, std::tuple<>, std::tuple< Types... >, 0u < sizeof...( Types ) >::type;
	};
	template< typename... Types >
	using unique_tuple_t = typename unique_tuple< Types... >::type;

	namespace detail
	{
		template< std::size_t INDEX, typename Tuple1, typename Tuple2, typename Func >
		FORCE_INLINE
		std::enable_if_t<
			(std::tuple_size< std::decay_t< Tuple1 > >::value <= INDEX)
		> for_each_impl( Tuple1 &&t1, Tuple2 &&t2, Func &f )
		{
			// do nothing
		}
		template< std::size_t INDEX, typename Tuple1, typename Tuple2, typename Func >
		FORCE_INLINE
		std::enable_if_t<
			(std::tuple_size< Tuple1 >::value > INDEX)
		> for_each_impl( Tuple1 &&t1, Tuple2 &&t2, Func &f )
		{
			f( std::get< INDEX >( std::forward< Tuple1 >( t1 ) ), std::get< INDEX >( std::forward< Tuple2 >( t2 ) ) );
			for_each_impl< INDEX + 1u >( std::forward< Tuple1 >( t1 ), std::forward< Tuple2 >( t2 ), f );
		}
	}
	template< typename Tuple1, typename Tuple2, typename Func >
	FORCE_INLINE
	std::enable_if_t<
		std::tuple_size< std::decay_t< Tuple1 > >::value == std::tuple_size< std::decay_t< Tuple2 > >::value &&
		(std::tuple_size< std::decay_t< Tuple1 > >::value > 0)
	> for_each( Tuple1 &&t1, Tuple2 &&t2, Func f )
	{
		detail::for_each_impl< 0u >( std::forward< Tuple1 >( t1 ), std::forward< Tuple2 >( t2 ), f );
	}

	namespace detail
	{
		template< std::size_t INDEX, typename Tuple1, typename Func >
		FORCE_INLINE
		std::enable_if_t<
			(std::tuple_size< std::decay_t< Tuple1 > >::value <= INDEX)
		> for_each_impl( Tuple1 &&t1, Func &f )
		{
			// do nothing
		}
		template< std::size_t INDEX, typename Tuple1, typename Func >
		FORCE_INLINE
		std::enable_if_t<
			(std::tuple_size< std::decay_t< Tuple1 > >::value > INDEX)
		> for_each_impl( Tuple1 &&t1, Func &f )
		{
			f( std::get< INDEX >( std::forward< Tuple1 >( t1 ) ) );
			for_each_impl< INDEX + 1u >( std::forward< Tuple1 >( t1 ), f );
		}
	}
	template< typename Tuple1, typename Func >
	FORCE_INLINE
	std::enable_if_t<
		(std::tuple_size< std::decay_t< Tuple1 > >::value > 0)
	> for_each( Tuple1 &&t1, Func f )
	{
		detail::for_each_impl< 0u >( std::forward< Tuple1 >( t1 ), f );
	}
}
