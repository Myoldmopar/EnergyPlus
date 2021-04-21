#ifndef ObjexxFCL_Array4S_hh_INCLUDED
#define ObjexxFCL_Array4S_hh_INCLUDED

// Array4S: 4D Slice Array Proxy
//
// Project: Objexx Fortran-C++ Library (ObjexxFCL)
//
// Version: 4.2.0
//
// Language: C++
//
// Copyright (c) 2000-2017 Objexx Engineering, Inc. All Rights Reserved.
// Use of this source code or any derivative of it is restricted by license.
// Licensing is available from Objexx Engineering, Inc.:  http://objexx.com

// ObjexxFCL Headers
#include <ObjexxFCL/Array4S.fwd.hh>
#include <ObjexxFCL/ArrayRS.hh>
#include <ObjexxFCL/Array3S.hh>

namespace ObjexxFCL {

// Array4S: 4D Slice Array Proxy
template< typename T >
class Array4S : public ArrayRS< T, 4 >
{

private: // Types

	typedef  ArrayRS< T, 4 >  Super;

private: // Friend

	template< typename > friend class Array4S;

public: // Types

	typedef  typename Super::Base  Base;
	typedef  typename Super::Traits  Traits;
	typedef  typename Super::IR  IR;
	typedef  typename Super::IS  IS;
	typedef  typename Super::DS  DS;

	// STL Style
	typedef  typename Super::value_type  value_type;
	typedef  typename Super::reference  reference;
	typedef  typename Super::const_reference  const_reference;
	typedef  typename Super::pointer  pointer;
	typedef  typename Super::const_pointer  const_pointer;
	typedef  typename Super::size_type  size_type;
	typedef  typename Super::difference_type  difference_type;

	// C++ Style
	typedef  typename Super::Value  Value;
	typedef  typename Super::Reference  Reference;
	typedef  typename Super::ConstReference  ConstReference;
	typedef  typename Super::Pointer  Pointer;
	typedef  typename Super::ConstPointer  ConstPointer;
	typedef  typename Super::Size  Size;
	typedef  typename Super::Difference  Difference;

	using Super::isize;
	using Super::overlap;
	using Super::size;

protected: // Types

	using Super::in_range;
	using Super::slice_k;

	using Super::contiguous_;
	using Super::data_;
	using Super::data_beg_;
	using Super::data_end_;
	using Super::size_;

public: // Creation

	// Default Constructor
	Array4S() :
	 m4_( 1 ),
	 m3_( 1 ),
	 m2_( 1 ),
	 m1_( 1 ),
	 k_( 0 ),
	 u1_( 0 ),
	 u2_( 0 ),
	 u3_( 0 ),
	 u4_( 0 )
	{}

	// Copy Constructor
	Array4S( Array4S const & a ) :
	 Super( a ),
	 m4_( a.m4_ ),
	 m3_( a.m3_ ),
	 m2_( a.m2_ ),
	 m1_( a.m1_ ),
	 k_( a.k_ ),
	 u1_( a.u1_ ),
	 u2_( a.u2_ ),
	 u3_( a.u3_ ),
	 u4_( a.u4_ )
	{
		data_set();
	}

	// Data Constructor
	Array4S( T const * data, std::int64_t const k, DS const & d1, DS const & d2, DS const & d3, DS const & d4 ) :
	 Super( data, d1.z() * d2.z() * d3.z() * d4.z() ),
	 m4_( d4.m() ),
	 m3_( d3.m() ),
	 m2_( d2.m() ),
	 m1_( d1.m() ),
	 k_( k + d1.k() + d2.k() + d3.k() + d4.k() ),
	 u1_( d1.u() ),
	 u2_( d2.u() ),
	 u3_( d3.u() ),
	 u4_( d4.u() )
	{
		contiguous_ = computed_contiguous();
		data_set();
	}

	// Non-Const Data Constructor
	Array4S( T * data, std::int64_t const k, DS const & d1, DS const & d2, DS const & d3, DS const & d4 ) :
	 Super( data, d1.z() * d2.z() * d3.z() * d4.z() ),
	 m4_( d4.m() ),
	 m3_( d3.m() ),
	 m2_( d2.m() ),
	 m1_( d1.m() ),
	 k_( k + d1.k() + d2.k() + d3.k() + d4.k() ),
	 u1_( d1.u() ),
	 u2_( d2.u() ),
	 u3_( d3.u() ),
	 u4_( d4.u() )
	{
		contiguous_ = computed_contiguous();
		data_set();
	}

	// Array Constructor
	template< template< typename > class A >
	Array4S( A< T > const & a ) :
	 Super( a.data(), a.size() ),
	 m4_( 1 ),
	 m3_( a.size4() ),
	 m2_( a.size3() * m3_ ),
	 m1_( a.size2() * m2_ ),
	 k_( -( m1_ + m2_ + m3_ + m4_ ) ),
	 u1_( a.isize1() ),
	 u2_( a.isize2() ),
	 u3_( a.isize3() ),
	 u4_( a.isize4() )
	{
		contiguous_ = true;
		data_set();
	}

	// Destructor
	virtual
	~Array4S()
	{}

public: // Assignment: Array

	// Copy Assignment Template
	template< typename U, class = typename std::enable_if< std::is_assignable< T&, U >::value >::type >
	Array4S &
	operator =( Array4S< U > const & a )
	{
		assert( conformable( a ) );
		for ( int i1 = 1; i1 <= u1_; ++i1 ) {
			for ( int i2 = 1; i2 <= u2_; ++i2 ) {
				for ( int i3 = 1; i3 <= u3_; ++i3 ) {
					for ( int i4 = 1; i4 <= u4_; ++i4 ) {
						operator ()( i1, i2, i3, i4 ) = a( i1, i2, i3, i4 );
					}
				}
			}
		}
		return *this;
	}


	// Array Assignment Template
	template< template< typename > class A, typename U, class = typename std::enable_if< std::is_assignable< T&, U >::value >::type >
	Array4S &
	operator =( A< U > const & a )
	{
		assert( conformable( a ) );
		for ( int i1 = 1, j1 = a.l1(); i1 <= u1_; ++i1, ++j1 ) {
			for ( int i2 = 1, j2 = a.l2(); i2 <= u2_; ++i2, ++j2 ) {
				for ( int i3 = 1, j3 = a.l3(); i3 <= u3_; ++i3, ++j3 ) {
					for ( int i4 = 1, j4 = a.l4(); i4 <= u4_; ++i4, ++j4 ) {
						operator ()( i1, i2, i3, i4 ) = a( j1, j2, j3, j4 );
					}
				}
			}
		}
		return *this;
	}

	// Initializer List Assignment Template
	template< typename U, class = typename std::enable_if< std::is_assignable< T&, U >::value >::type >
	Array4S &
	operator =( std::initializer_list< U > const l )
	{
		assert( size_ == l.size() );
		auto r( l.begin() );
		for ( int i1 = 1; i1 <= u1_; ++i1 ) {
			for ( int i2 = 1; i2 <= u2_; ++i2 ) {
				for ( int i3 = 1; i3 <= u3_; ++i3 ) {
					for ( int i4 = 1; i4 <= u4_; ++i4, ++r ) {
						operator ()( i1, i2, i3, i4 ) = *r;
					}
				}
			}
		}
		return *this;
	}

	// += Array Template
	template< template< typename > class A, typename U, class = typename std::enable_if< std::is_assignable< T&, U >::value >::type >
	Array4S &
	operator +=( A< U > const & a )
	{
		assert( conformable( a ) );
		for ( int i1 = 1, j1 = a.l1(); i1 <= u1_; ++i1, ++j1 ) {
			for ( int i2 = 1, j2 = a.l2(); i2 <= u2_; ++i2, ++j2 ) {
				for ( int i3 = 1, j3 = a.l3(); i3 <= u3_; ++i3, ++j3 ) {
					for ( int i4 = 1, j4 = a.l4(); i4 <= u4_; ++i4, ++j4 ) {
						operator ()( i1, i2, i3, i4 ) += a( j1, j2, j3, j4 );
					}
				}
			}
		}
		return *this;
	}

	// -= Array Template
	template< template< typename > class A, typename U, class = typename std::enable_if< std::is_assignable< T&, U >::value >::type >
	Array4S &
	operator -=( A< U > const & a )
	{
		assert( conformable( a ) );
		for ( int i1 = 1, j1 = a.l1(); i1 <= u1_; ++i1, ++j1 ) {
			for ( int i2 = 1, j2 = a.l2(); i2 <= u2_; ++i2, ++j2 ) {
				for ( int i3 = 1, j3 = a.l3(); i3 <= u3_; ++i3, ++j3 ) {
					for ( int i4 = 1, j4 = a.l4(); i4 <= u4_; ++i4, ++j4 ) {
						operator ()( i1, i2, i3, i4 ) -= a( j1, j2, j3, j4 );
					}
				}
			}
		}
		return *this;
	}

	// *= Array Template
	template< template< typename > class A, typename U, class = typename std::enable_if< std::is_assignable< T&, U >::value >::type >
	Array4S &
	operator *=( A< U > const & a )
	{
		assert( conformable( a ) );
		for ( int i1 = 1, j1 = a.l1(); i1 <= u1_; ++i1, ++j1 ) {
			for ( int i2 = 1, j2 = a.l2(); i2 <= u2_; ++i2, ++j2 ) {
				for ( int i3 = 1, j3 = a.l3(); i3 <= u3_; ++i3, ++j3 ) {
					for ( int i4 = 1, j4 = a.l4(); i4 <= u4_; ++i4, ++j4 ) {
						operator ()( i1, i2, i3, i4 ) *= a( j1, j2, j3, j4 );
					}
				}
			}
		}
		return *this;
	}

	// /= Array Template
	template< template< typename > class A, typename U, class = typename std::enable_if< std::is_assignable< T&, U >::value >::type >
	Array4S &
	operator /=( A< U > const & a )
	{
		assert( conformable( a ) );
		for ( int i1 = 1, j1 = a.l1(); i1 <= u1_; ++i1, ++j1 ) {
			for ( int i2 = 1, j2 = a.l2(); i2 <= u2_; ++i2, ++j2 ) {
				for ( int i3 = 1, j3 = a.l3(); i3 <= u3_; ++i3, ++j3 ) {
					for ( int i4 = 1, j4 = a.l4(); i4 <= u4_; ++i4, ++j4 ) {
						assert( a( j1, j2, j3, j4 ) != T( 0 ) );
						operator ()( i1, i2, i3, i4 ) /= a( j1, j2, j3, j4 );
					}
				}
			}
		}
		return *this;
	}

public: // Assignment: Logical

	// &&= Array Template
	template< typename U, class = typename std::enable_if< std::is_assignable< T&, U >::value >::type >
	Array4S &
	and_equals( Array4S const & a )
	{
		assert( conformable( a ) );
		for ( int i1 = 1; i1 <= u1_; ++i1 ) {
			for ( int i2 = 1; i2 <= u2_; ++i2 ) {
				for ( int i3 = 1; i3 <= u3_; ++i3 ) {
					for ( int i4 = 1; i4 <= u4_; ++i4 ) {
						auto & v( operator ()( i1, i2, i3, i4 ) );
						v = v && a( i1, i2, i3, i4 );
					}
				}
			}
		}
		return *this;
	}

	// ||= Array Template
	template< typename U, class = typename std::enable_if< std::is_assignable< T&, U >::value >::type >
	Array4S &
	or_equals( Array4S const & a )
	{
		assert( conformable( a ) );
		for ( int i1 = 1; i1 <= u1_; ++i1 ) {
			for ( int i2 = 1; i2 <= u2_; ++i2 ) {
				for ( int i3 = 1; i3 <= u3_; ++i3 ) {
					for ( int i4 = 1; i4 <= u4_; ++i4 ) {
						auto & v( operator ()( i1, i2, i3, i4 ) );
						v = v || a( i1, i2, i3, i4 );
					}
				}
			}
		}
		return *this;
	}

public: // Assignment: Value

	// = Value
	Array4S &
	operator =( T const & t )
	{
		for ( int i1 = 1; i1 <= u1_; ++i1 ) {
			for ( int i2 = 1; i2 <= u2_; ++i2 ) {
				for ( int i3 = 1; i3 <= u3_; ++i3 ) {
					for ( int i4 = 1; i4 <= u4_; ++i4 ) {
						operator ()( i1, i2, i3, i4 ) = t;
					}
				}
			}
		}
		return *this;
	}

	// += Value
	Array4S &
	operator +=( T const & t )
	{
		for ( int i1 = 1; i1 <= u1_; ++i1 ) {
			for ( int i2 = 1; i2 <= u2_; ++i2 ) {
				for ( int i3 = 1; i3 <= u3_; ++i3 ) {
					for ( int i4 = 1; i4 <= u4_; ++i4 ) {
						operator ()( i1, i2, i3, i4 ) += t;
					}
				}
			}
		}
		return *this;
	}

	// -= Value
	Array4S &
	operator -=( T const & t )
	{
		for ( int i1 = 1; i1 <= u1_; ++i1 ) {
			for ( int i2 = 1; i2 <= u2_; ++i2 ) {
				for ( int i3 = 1; i3 <= u3_; ++i3 ) {
					for ( int i4 = 1; i4 <= u4_; ++i4 ) {
						operator ()( i1, i2, i3, i4 ) -= t;
					}
				}
			}
		}
		return *this;
	}

	// *= Value
	Array4S &
	operator *=( T const & t )
	{
		for ( int i1 = 1; i1 <= u1_; ++i1 ) {
			for ( int i2 = 1; i2 <= u2_; ++i2 ) {
				for ( int i3 = 1; i3 <= u3_; ++i3 ) {
					for ( int i4 = 1; i4 <= u4_; ++i4 ) {
						operator ()( i1, i2, i3, i4 ) *= t;
					}
				}
			}
		}
		return *this;
	}

	// /= Value
	template< typename U, class = typename std::enable_if< std::is_floating_point< U >::value && std::is_assignable< T&, U >::value >::type >
	Array4S &
	operator /=( U const & u )
	{
		assert( u != U( 0 ) );
		U const inv_u( U( 1 ) / u );
		for ( int i1 = 1; i1 <= u1_; ++i1 ) {
			for ( int i2 = 1; i2 <= u2_; ++i2 ) {
				for ( int i3 = 1; i3 <= u3_; ++i3 ) {
					for ( int i4 = 1; i4 <= u4_; ++i4 ) {
						operator ()( i1, i2, i3, i4 ) *= inv_u;
					}
				}
			}
		}
		return *this;
	}

	// /= Value
	template< typename U, class = typename std::enable_if< ! std::is_floating_point< U >::value && std::is_assignable< T&, U >::value >::type, typename = void >
	Array4S &
	operator /=( U const & u )
	{
		assert( u != U( 0 ) );
		for ( int i1 = 1; i1 <= u1_; ++i1 ) {
			for ( int i2 = 1; i2 <= u2_; ++i2 ) {
				for ( int i3 = 1; i3 <= u3_; ++i3 ) {
					for ( int i4 = 1; i4 <= u4_; ++i4 ) {
						operator ()( i1, i2, i3, i4 ) /= u;
					}
				}
			}
		}
		return *this;
	}

public: // Subscript

	// array( i1, i2, i3, i4 ) const
	T const &
	operator ()( int const i1, int const i2, int const i3, int const i4 ) const
	{
		assert( contains( i1, i2, i3, i4 ) );
		return data_[ k_ + ( m1_ * i1 ) + ( m2_ * i2 ) + ( m3_ * i3 ) + ( m4_ * i4 ) ];
	}

	// array( i1, i2, i3, i4 )
	T &
	operator ()( int const i1, int const i2, int const i3, int const i4 )
	{
		assert( contains( i1, i2, i3, i4 ) );
		return data_[ k_ + ( m1_ * i1 ) + ( m2_ * i2 ) + ( m3_ * i3 ) + ( m4_ * i4 ) ];
	}

	// Linear Index
	size_type
	index( int const i1, int const i2, int const i3, int const i4 ) const
	{
		return k_ + ( m1_ * i1 ) + ( m2_ * i2 ) + ( m3_ * i3 ) + ( m4_ * i4 );
	}

public: // Slice Proxy Generators

	// array( s1, s2, s3, s4 ) const
	Array4S< T >
	operator ()( IS const & s1, IS const & s2, IS const & s3, IS const & s4 ) const
	{
		DS const d1( u1_, s1, m1_ );
		DS const d2( u2_, s2, m2_ );
		DS const d3( u3_, s3, m3_ );
		DS const d4( u4_, s4, m4_ );
		return Array4S( data_, k_, d1, d2, d3, d4 );
	}

	// array( i1, s2, s3, s4 ) const
	Array3S< T >
	operator ()( int const i1, IS const & s2, IS const & s3, IS const & s4 ) const
	{
		std::int64_t k( k_ );
		k += slice_k( u1_, i1, m1_ );
		DS const d2( u2_, s2, m2_ );
		DS const d3( u3_, s3, m3_ );
		DS const d4( u4_, s4, m4_ );
		return Array3S< T >( data_, k, d2, d3, d4 );
	}

	// array( s1, i2, s3, s4 ) const
	Array3S< T >
	operator ()( IS const & s1, int const i2, IS const & s3, IS const & s4 ) const
	{
		std::int64_t k( k_ );
		DS const d1( u1_, s1, m1_ );
		k += slice_k( u2_, i2, m2_ );
		DS const d3( u3_, s3, m3_ );
		DS const d4( u4_, s4, m4_ );
		return Array3S< T >( data_, k, d1, d3, d4 );
	}

	// array( s1, s2, i3, s4 ) const
	Array3S< T >
	operator ()( IS const & s1, IS const & s2, int const i3, IS const & s4 ) const
	{
		std::int64_t k( k_ );
		DS const d1( u1_, s1, m1_ );
		DS const d2( u2_, s2, m2_ );
		k += slice_k( u3_, i3, m3_ );
		DS const d4( u4_, s4, m4_ );
		return Array3S< T >( data_, k, d1, d2, d4 );
	}

	// array( s1, s2, s3, i4 ) const
	Array3S< T >
	operator ()( IS const & s1, IS const & s2, IS const & s3, int const i4 ) const
	{
		std::int64_t k( k_ );
		DS const d1( u1_, s1, m1_ );
		DS const d2( u2_, s2, m2_ );
		DS const d3( u3_, s3, m3_ );
		k += slice_k( u4_, i4, m4_ );
		return Array3S< T >( data_, k, d1, d2, d3 );
	}

	// array( i1, i2, s3, s4 ) const
	Array2S< T >
	operator ()( int const i1, int const i2, IS const & s3, IS const & s4 ) const
	{
		std::int64_t k( k_ );
		k += slice_k( u1_, i1, m1_ );
		k += slice_k( u2_, i2, m2_ );
		DS const d3( u3_, s3, m3_ );
		DS const d4( u4_, s4, m4_ );
		return Array2S< T >( data_, k, d3, d4 );
	}

	// array( i1, s2, i3, s4 ) const
	Array2S< T >
	operator ()( int const i1, IS const & s2, int const i3, IS const & s4 ) const
	{
		std::int64_t k( k_ );
		k += slice_k( u1_, i1, m1_ );
		DS const d2( u2_, s2, m2_ );
		k += slice_k( u3_, i3, m3_ );
		DS const d4( u4_, s4, m4_ );
		return Array2S< T >( data_, k, d2, d4 );
	}

	// array( i1, s2, s3, i4 ) const
	Array2S< T >
	operator ()( int const i1, IS const & s2, IS const & s3, int const i4 ) const
	{
		std::int64_t k( k_ );
		k += slice_k( u1_, i1, m1_ );
		DS const d2( u2_, s2, m2_ );
		DS const d3( u3_, s3, m3_ );
		k += slice_k( u4_, i4, m4_ );
		return Array2S< T >( data_, k, d2, d3 );
	}

	// array( s1, i2, i3, s4 ) const
	Array2S< T >
	operator ()( IS const & s1, int const i2, int const i3, IS const & s4 ) const
	{
		std::int64_t k( k_ );
		DS const d1( u1_, s1, m1_ );
		k += slice_k( u2_, i2, m2_ );
		k += slice_k( u3_, i3, m3_ );
		DS const d4( u4_, s4, m4_ );
		return Array2S< T >( data_, k, d1, d4 );
	}

	// array( s1, i2, s3, i4 ) const
	Array2S< T >
	operator ()( IS const & s1, int const i2, IS const & s3, int const i4 ) const
	{
		std::int64_t k( k_ );
		DS const d1( u1_, s1, m1_ );
		k += slice_k( u2_, i2, m2_ );
		DS const d3( u3_, s3, m3_ );
		k += slice_k( u4_, i4, m4_ );
		return Array2S< T >( data_, k, d1, d3 );
	}

	// array( s1, s2, i3, i4 ) const
	Array2S< T >
	operator ()( IS const & s1, IS const & s2, int const i3, int const i4 ) const
	{
		std::int64_t k( k_ );
		DS const d1( u1_, s1, m1_ );
		DS const d2( u2_, s2, m2_ );
		k += slice_k( u3_, i3, m3_ );
		k += slice_k( u4_, i4, m4_ );
		return Array2S< T >( data_, k, d1, d2 );
	}

	// array( s1, i2, i3, i4 ) const
	Array1S< T >
	operator ()( IS const & s1, int const i2, int const i3, int const i4 ) const
	{
		std::int64_t k( k_ );
		DS const d1( u1_, s1, m1_ );
		k += slice_k( u2_, i2, m2_ );
		k += slice_k( u3_, i3, m3_ );
		k += slice_k( u4_, i4, m4_ );
		return Array1S< T >( data_, k, d1 );
	}

	// array( i1, s2, i3, i4 ) const
	Array1S< T >
	operator ()( int const i1, IS const & s2, int const i3, int const i4 ) const
	{
		std::int64_t k( k_ );
		k += slice_k( u1_, i1, m1_ );
		DS const d2( u2_, s2, m2_ );
		k += slice_k( u3_, i3, m3_ );
		k += slice_k( u4_, i4, m4_ );
		return Array1S< T >( data_, k, d2 );
	}

	// array( i1, i2, s3, i4 ) const
	Array1S< T >
	operator ()( int const i1, int const i2, IS const & s3, int const i4 ) const
	{
		std::int64_t k( k_ );
		k += slice_k( u1_, i1, m1_ );
		k += slice_k( u2_, i2, m2_ );
		DS const d3( u3_, s3, m3_ );
		k += slice_k( u4_, i4, m4_ );
		return Array1S< T >( data_, k, d3 );
	}

	// array( i1, i2, i3, s4 ) const
	Array1S< T >
	operator ()( int const i1, int const i2, int const i3, IS const & s4 ) const
	{
		std::int64_t k( k_ );
		k += slice_k( u1_, i1, m1_ );
		k += slice_k( u2_, i2, m2_ );
		k += slice_k( u3_, i3, m3_ );
		DS const d4( u4_, s4, m4_ );
		return Array1S< T >( data_, k, d4 );
	}

	// array( s1, s2, s3, s4 )
	Array4S< T >
	operator ()( IS const & s1, IS const & s2, IS const & s3, IS const & s4 )
	{
		DS const d1( u1_, s1, m1_ );
		DS const d2( u2_, s2, m2_ );
		DS const d3( u3_, s3, m3_ );
		DS const d4( u4_, s4, m4_ );
		return Array4S( data_, k_, d1, d2, d3, d4 );
	}

	// array( i1, s2, s3, s4 )
	Array3S< T >
	operator ()( int const i1, IS const & s2, IS const & s3, IS const & s4 )
	{
		std::int64_t k( k_ );
		k += slice_k( u1_, i1, m1_ );
		DS const d2( u2_, s2, m2_ );
		DS const d3( u3_, s3, m3_ );
		DS const d4( u4_, s4, m4_ );
		return Array3S< T >( data_, k, d2, d3, d4 );
	}

	// array( s1, i2, s3, s4 )
	Array3S< T >
	operator ()( IS const & s1, int const i2, IS const & s3, IS const & s4 )
	{
		std::int64_t k( k_ );
		DS const d1( u1_, s1, m1_ );
		k += slice_k( u2_, i2, m2_ );
		DS const d3( u3_, s3, m3_ );
		DS const d4( u4_, s4, m4_ );
		return Array3S< T >( data_, k, d1, d3, d4 );
	}

	// array( s1, s2, i3, s4 )
	Array3S< T >
	operator ()( IS const & s1, IS const & s2, int const i3, IS const & s4 )
	{
		std::int64_t k( k_ );
		DS const d1( u1_, s1, m1_ );
		DS const d2( u2_, s2, m2_ );
		k += slice_k( u3_, i3, m3_ );
		DS const d4( u4_, s4, m4_ );
		return Array3S< T >( data_, k, d1, d2, d4 );
	}

	// array( s1, s2, s3, i4 )
	Array3S< T >
	operator ()( IS const & s1, IS const & s2, IS const & s3, int const i4 )
	{
		std::int64_t k( k_ );
		DS const d1( u1_, s1, m1_ );
		DS const d2( u2_, s2, m2_ );
		DS const d3( u3_, s3, m3_ );
		k += slice_k( u4_, i4, m4_ );
		return Array3S< T >( data_, k, d1, d2, d3 );
	}

	// array( i1, i2, s3, s4 )
	Array2S< T >
	operator ()( int const i1, int const i2, IS const & s3, IS const & s4 )
	{
		std::int64_t k( k_ );
		k += slice_k( u1_, i1, m1_ );
		k += slice_k( u2_, i2, m2_ );
		DS const d3( u3_, s3, m3_ );
		DS const d4( u4_, s4, m4_ );
		return Array2S< T >( data_, k, d3, d4 );
	}

	// array( i1, s2, i3, s4 )
	Array2S< T >
	operator ()( int const i1, IS const & s2, int const i3, IS const & s4 )
	{
		std::int64_t k( k_ );
		k += slice_k( u1_, i1, m1_ );
		DS const d2( u2_, s2, m2_ );
		k += slice_k( u3_, i3, m3_ );
		DS const d4( u4_, s4, m4_ );
		return Array2S< T >( data_, k, d2, d4 );
	}

	// array( i1, s2, s3, i4 )
	Array2S< T >
	operator ()( int const i1, IS const & s2, IS const & s3, int const i4 )
	{
		std::int64_t k( k_ );
		k += slice_k( u1_, i1, m1_ );
		DS const d2( u2_, s2, m2_ );
		DS const d3( u3_, s3, m3_ );
		k += slice_k( u4_, i4, m4_ );
		return Array2S< T >( data_, k, d2, d3 );
	}

	// array( s1, i2, i3, s4 )
	Array2S< T >
	operator ()( IS const & s1, int const i2, int const i3, IS const & s4 )
	{
		std::int64_t k( k_ );
		DS const d1( u1_, s1, m1_ );
		k += slice_k( u2_, i2, m2_ );
		k += slice_k( u3_, i3, m3_ );
		DS const d4( u4_, s4, m4_ );
		return Array2S< T >( data_, k, d1, d4 );
	}

	// array( s1, i2, s3, i4 )
	Array2S< T >
	operator ()( IS const & s1, int const i2, IS const & s3, int const i4 )
	{
		std::int64_t k( k_ );
		DS const d1( u1_, s1, m1_ );
		k += slice_k( u2_, i2, m2_ );
		DS const d3( u3_, s3, m3_ );
		k += slice_k( u4_, i4, m4_ );
		return Array2S< T >( data_, k, d1, d3 );
	}

	// array( s1, s2, i3, i4 )
	Array2S< T >
	operator ()( IS const & s1, IS const & s2, int const i3, int const i4 )
	{
		std::int64_t k( k_ );
		DS const d1( u1_, s1, m1_ );
		DS const d2( u2_, s2, m2_ );
		k += slice_k( u3_, i3, m3_ );
		k += slice_k( u4_, i4, m4_ );
		return Array2S< T >( data_, k, d1, d2 );
	}

	// array( s1, i2, i3, i4 )
	Array1S< T >
	operator ()( IS const & s1, int const i2, int const i3, int const i4 )
	{
		std::int64_t k( k_ );
		DS const d1( u1_, s1, m1_ );
		k += slice_k( u2_, i2, m2_ );
		k += slice_k( u3_, i3, m3_ );
		k += slice_k( u4_, i4, m4_ );
		return Array1S< T >( data_, k, d1 );
	}

	// array( i1, s2, i3, i4 )
	Array1S< T >
	operator ()( int const i1, IS const & s2, int const i3, int const i4 )
	{
		std::int64_t k( k_ );
		k += slice_k( u1_, i1, m1_ );
		DS const d2( u2_, s2, m2_ );
		k += slice_k( u3_, i3, m3_ );
		k += slice_k( u4_, i4, m4_ );
		return Array1S< T >( data_, k, d2 );
	}

	// array( i1, i2, s3, i4 )
	Array1S< T >
	operator ()( int const i1, int const i2, IS const & s3, int const i4 )
	{
		std::int64_t k( k_ );
		k += slice_k( u1_, i1, m1_ );
		k += slice_k( u2_, i2, m2_ );
		DS const d3( u3_, s3, m3_ );
		k += slice_k( u4_, i4, m4_ );
		return Array1S< T >( data_, k, d3 );
	}

	// array( i1, i2, i3, s4 )
	Array1S< T >
	operator ()( int const i1, int const i2, int const i3, IS const & s4 )
	{
		std::int64_t k( k_ );
		k += slice_k( u1_, i1, m1_ );
		k += slice_k( u2_, i2, m2_ );
		k += slice_k( u3_, i3, m3_ );
		DS const d4( u4_, s4, m4_ );
		return Array1S< T >( data_, k, d4 );
	}

public: // Predicate

	// Contains Indexed Element?
	bool
	contains( int const i1, int const i2, int const i3, int const i4 ) const
	{
		if ( ! in_range( u1(), i1 ) ) return false;
		if ( ! in_range( u2(), i2 ) ) return false;
		if ( ! in_range( u3(), i3 ) ) return false;
		if ( ! in_range( u4(), i4 ) ) return false;
		return true;
	}

	// Conformable?
	template< typename U >
	bool
	conformable( Array4S< U > const & a ) const
	{
		return ( ( size1() == a.size1() ) && ( size2() == a.size2() ) && ( size3() == a.size3() ) && ( size4() == a.size4() ) );
	}

	// Conformable?
	template< class A >
	bool
	conformable( A const & a ) const
	{
		return ( ( a.rank() == 4 ) && ( size1() == a.size1() ) && ( size2() == a.size2() ) && ( size3() == a.size3() ) && ( size4() == a.size4() ) );
	}

	// Equal Dimensions?
	template< typename U >
	bool
	equal_dimensions( Array4S< U > const & a ) const
	{
		return conformable( a );
	}

	// Equal Dimensions?
	template< class A >
	bool
	equal_dimensions( A const & a ) const
	{
		return conformable( a );
	}

public: // Inspector

	// IndexRange of a Dimension
	IR
	I( int const d ) const
	{
		switch ( d ) {
		case 1:
			return I1();
		case 2:
			return I2();
		case 3:
			return I3();
		case 4:
			return I4();
		default:
			assert( false );
			return I1();
		}
	}

	// Upper Index of a Dimension
	int
	u( int const d ) const
	{
		switch ( d ) {
		case 1:
			return u1_;
		case 2:
			return u2_;
		case 3:
			return u3_;
		case 4:
			return u4_;
		default:
			assert( false );
			return u1_;
		}
	}

	// Size of a Dimension
	size_type
	size( int const d ) const
	{
		switch ( d ) {
		case 1:
			return size1();
		case 2:
			return size2();
		case 3:
			return size3();
		case 4:
			return size4();
		default:
			assert( false );
			return size1();
		}
	}

	// Size of a Dimension
	int
	isize( int const d ) const
	{
		switch ( d ) {
		case 1:
			return isize1();
		case 2:
			return isize2();
		case 3:
			return isize3();
		case 4:
			return isize4();
		default:
			assert( false );
			return isize1();
		}
	}

	// IndexRange of Dimension 1
	IR
	I1() const
	{
		return IR( 1, u1_ );
	}

	// Lower Index of Dimension 1
	int
	l1() const
	{
		return 1;
	}

	// Upper Index of Dimension 1
	int
	u1() const
	{
		return u1_;
	}

	// Size of Dimension 1
	size_type
	size1() const
	{
		return u1_;
	}

	// Size of Dimension 1
	int
	isize1() const
	{
		return u1_;
	}

	// IndexRange of Dimension 2
	IR
	I2() const
	{
		return IR( 1, u2_ );
	}

	// Lower Index of Dimension 2
	int
	l2() const
	{
		return 1;
	}

	// Upper Index of Dimension 2
	int
	u2() const
	{
		return u2_;
	}

	// Size of Dimension 2
	size_type
	size2() const
	{
		return u2_;
	}

	// Size of Dimension 2
	int
	isize2() const
	{
		return u2_;
	}

	// IndexRange of Dimension 3
	IR
	I3() const
	{
		return IR( 1, u3_ );
	}

	// Lower Index of Dimension 3
	int
	l3() const
	{
		return 1;
	}

	// Upper Index of Dimension 3
	int
	u3() const
	{
		return u3_;
	}

	// Size of Dimension 3
	size_type
	size3() const
	{
		return u3_;
	}

	// Size of Dimension 3
	int
	isize3() const
	{
		return u3_;
	}

	// IndexRange of Dimension 4
	IR
	I4() const
	{
		return IR( 1, u4_ );
	}

	// Lower Index of Dimension 4
	int
	l4() const
	{
		return 1;
	}

	// Upper Index of Dimension 4
	int
	u4() const
	{
		return u4_;
	}

	// Size of Dimension 4
	size_type
	size4() const
	{
		return u4_;
	}

	// Size of Dimension 4
	int
	isize4() const
	{
		return u4_;
	}

	// Shift for Proxy
	std::ptrdiff_t
	shift() const
	{
		return ( ( ( static_cast< std::ptrdiff_t >( u2_ + 1 ) * u3_ ) + 1 ) * u4_ ) + 1;
	}

public: // MArray Generators

	// Template Helpers
	template< typename U > class Wrapper {};
	typedef  typename std::conditional< std::is_class< T >::value, T, Wrapper< T > >::type  ClassT;

public: // Comparison: Predicate

	// Slice == Slice
	friend
	bool
	eq( Array4S const & a, Array4S const & b )
	{
		assert( a.conformable( b ) );
		if ( ( &a == &b ) || a.empty() ) return true;
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( ! ( a( i1, i2, i3, i4 ) == b( i1, i2, i3, i4 ) ) ) return false;
					}
				}
			}
		}
		return true;
	}

	// Slice != Slice
	friend
	bool
	ne( Array4S const & a, Array4S const & b )
	{
		return ! eq( a, b );
	}

	// Slice < Slice
	friend
	bool
	lt( Array4S const & a, Array4S const & b )
	{
		assert( a.conformable( b ) );
		if ( ( &a == &b ) || a.empty() ) return false;
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( ! ( a( i1, i2, i3, i4 ) < b( i1, i2, i3, i4 ) ) ) return false;
					}
				}
			}
		}
		return true;
	}

	// Slice <= Slice
	friend
	bool
	le( Array4S const & a, Array4S const & b )
	{
		assert( a.conformable( b ) );
		if ( ( &a == &b ) || a.empty() ) return true;
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( ! ( a( i1, i2, i3, i4 ) <= b( i1, i2, i3, i4 ) ) ) return false;
					}
				}
			}
		}
		return true;
	}

	// Slice > Slice
	friend
	bool
	gt( Array4S const & a, Array4S const & b )
	{
		return lt( b, a );
	}

	// Slice >= Slice
	friend
	bool
	ge( Array4S const & a, Array4S const & b )
	{
		return le( b, a );
	}

	// Slice == Value
	friend
	bool
	eq( Array4S const & a, T const & t )
	{
		if ( a.empty() ) return true;
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( ! ( a( i1, i2, i3, i4 ) == t ) ) return false;
					}
				}
			}
		}
		return true;
	}

	// Slice != Value
	friend
	bool
	ne( Array4S const & a, T const & t )
	{
		return ! eq( a, t );
	}

	// Slice < Value
	friend
	bool
	lt( Array4S const & a, T const & t )
	{
		if ( a.empty() ) return false;
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( ! ( a( i1, i2, i3, i4 ) < t ) ) return false;
					}
				}
			}
		}
		return true;
	}

	// Slice <= Value
	friend
	bool
	le( Array4S const & a, T const & t )
	{
		if ( a.empty() ) return true;
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( ! ( a( i1, i2, i3, i4 ) <= t ) ) return false;
					}
				}
			}
		}
		return true;
	}

	// Slice > Value
	friend
	bool
	gt( Array4S const & a, T const & t )
	{
		return lt( t, a );
	}

	// Slice >= Value
	friend
	bool
	ge( Array4S const & a, T const & t )
	{
		return le( t, a );
	}

	// Value == Slice
	friend
	bool
	eq( T const & t, Array4S const & a )
	{
		return eq( a, t );
	}

	// Value != Slice
	friend
	bool
	ne( T const & t, Array4S const & a )
	{
		return ! eq( a, t );
	}

	// Value < Slice
	friend
	bool
	lt( T const & t, Array4S const & a )
	{
		if ( a.empty() ) return false;
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( ! ( t < a( i1, i2, i3, i4 ) ) ) return false;
					}
				}
			}
		}
		return true;
	}

	// Value <= Slice
	friend
	bool
	le( T const & t, Array4S const & a )
	{
		if ( a.empty() ) return true;
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( ! ( t <= a( i1, i2, i3, i4 ) ) ) return false;
					}
				}
			}
		}
		return true;
	}

	// Value > Slice
	friend
	bool
	gt( T const & t, Array4S const & a )
	{
		return lt( a, t );
	}

	// Value >= Slice
	friend
	bool
	ge( T const & t, Array4S const & a )
	{
		return le( a, t );
	}

public: // Comparison: Predicate: Any

	// Any Slice == Slice
	friend
	bool
	any_eq( Array4S const & a, Array4S const & b )
	{
		assert( a.conformable( b ) );
		if ( a.empty() ) return false;
		if ( &a == &b ) return true;
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( a( i1, i2, i3, i4 ) == b( i1, i2, i3, i4 ) ) return true;
					}
				}
			}
		}
		return false;
	}

	// Any Slice != Slice
	friend
	bool
	any_ne( Array4S const & a, Array4S const & b )
	{
		return ! eq( a, b );
	}

	// Any Slice < Slice
	friend
	bool
	any_lt( Array4S const & a, Array4S const & b )
	{
		assert( a.conformable( b ) );
		if ( a.empty() ) return false;
		if ( &a == &b ) return false;
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( a( i1, i2, i3, i4 ) < b( i1, i2, i3, i4 ) ) return true;
					}
				}
			}
		}
		return false;
	}

	// Any Slice <= Slice
	friend
	bool
	any_le( Array4S const & a, Array4S const & b )
	{
		assert( a.conformable( b ) );
		if ( a.empty() ) return false;
		if ( &a == &b ) return true;
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( a( i1, i2, i3, i4 ) <= b( i1, i2, i3, i4 ) ) return true;
					}
				}
			}
		}
		return false;
	}

	// Any Slice > Slice
	friend
	bool
	any_gt( Array4S const & a, Array4S const & b )
	{
		return any_lt( b, a );
	}

	// Any Slice >= Slice
	friend
	bool
	any_ge( Array4S const & a, Array4S const & b )
	{
		return any_le( b, a );
	}

	// Any Slice == Value
	friend
	bool
	any_eq( Array4S const & a, T const & t )
	{
		if ( a.empty() ) return false;
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( a( i1, i2, i3, i4 ) == t ) return true;
					}
				}
			}
		}
		return false;
	}

	// Any Slice != Value
	friend
	bool
	any_ne( Array4S const & a, T const & t )
	{
		return ! eq( a, t );
	}

	// Any Slice < Value
	friend
	bool
	any_lt( Array4S const & a, T const & t )
	{
		if ( a.empty() ) return false;
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( a( i1, i2, i3, i4 ) < t ) return true;
					}
				}
			}
		}
		return false;
	}

	// Any Slice <= Value
	friend
	bool
	any_le( Array4S const & a, T const & t )
	{
		if ( a.empty() ) return false;
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( a( i1, i2, i3, i4 ) <= t ) return true;
					}
				}
			}
		}
		return false;
	}

	// Any Slice > Value
	friend
	bool
	any_gt( Array4S const & a, T const & t )
	{
		return any_lt( t, a );
	}

	// Any Slice >= Value
	friend
	bool
	any_ge( Array4S const & a, T const & t )
	{
		return any_le( t, a );
	}

	// Any Value == Slice
	friend
	bool
	any_eq( T const & t, Array4S const & a )
	{
		return any_eq( a, t );
	}

	// Any Value != Slice
	friend
	bool
	any_ne( T const & t, Array4S const & a )
	{
		return ! eq( a, t );
	}

	// Any Value < Slice
	friend
	bool
	any_lt( T const & t, Array4S const & a )
	{
		if ( a.empty() ) return false;
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( t < a( i1, i2, i3, i4 ) ) return true;
					}
				}
			}
		}
		return false;
	}

	// Any Value <= Slice
	friend
	bool
	any_le( T const & t, Array4S const & a )
	{
		if ( a.empty() ) return false;
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( t <= a( i1, i2, i3, i4 ) ) return true;
					}
				}
			}
		}
		return false;
	}

	// Any Value > Slice
	friend
	bool
	any_gt( T const & t, Array4S const & a )
	{
		return any_lt( a, t );
	}

	// Any Value >= Slice
	friend
	bool
	any_ge( T const & t, Array4S const & a )
	{
		return any_le( a, t );
	}

public: // Comparison: Predicate: All

	// All Slice == Slice
	friend
	bool
	all_eq( Array4S const & a, Array4S const & b )
	{
		return eq( a, b );
	}

	// All Slice != Slice
	friend
	bool
	all_ne( Array4S const & a, Array4S const & b )
	{
		return ! any_eq( a, b );
	}

	// All Slice < Slice
	friend
	bool
	all_lt( Array4S const & a, Array4S const & b )
	{
		return lt( a, b );
	}

	// All Slice <= Slice
	friend
	bool
	all_le( Array4S const & a, Array4S const & b )
	{
		return le( a, b );
	}

	// All Slice > Slice
	friend
	bool
	all_gt( Array4S const & a, Array4S const & b )
	{
		return gt( a, b );
	}

	// All Slice >= Slice
	friend
	bool
	all_ge( Array4S const & a, Array4S const & b )
	{
		return ge( a, b );
	}

	// All Slice == Value
	friend
	bool
	all_eq( Array4S const & a, T const & t )
	{
		return eq( a, t );
	}

	// All Slice != Value
	friend
	bool
	all_ne( Array4S const & a, T const & t )
	{
		return ! any_eq( a, t );
	}

	// All Slice < Value
	friend
	bool
	all_lt( Array4S const & a, T const & t )
	{
		return lt( a, t );
	}

	// All Slice <= Value
	friend
	bool
	all_le( Array4S const & a, T const & t )
	{
		return le( a, t );
	}

	// All Slice > Value
	friend
	bool
	all_gt( Array4S const & a, T const & t )
	{
		return gt( a, t );
	}

	// All Slice >= Value
	friend
	bool
	all_ge( Array4S const & a, T const & t )
	{
		return ge( a, t );
	}

	// All Value == Slice
	friend
	bool
	all_eq( T const & t, Array4S const & a )
	{
		return eq( t, a );
	}

	// All Value != Slice
	friend
	bool
	all_ne( T const & t, Array4S const & a )
	{
		return ! any_eq( t, a );
	}

	// All Value < Slice
	friend
	bool
	all_lt( T const & t, Array4S const & a )
	{
		return lt( t, a );
	}

	// All Value <= Slice
	friend
	bool
	all_le( T const & t, Array4S const & a )
	{
		return le( t, a );
	}

	// All Value > Slice
	friend
	bool
	all_gt( T const & t, Array4S const & a )
	{
		return gt( t, a );
	}

	// All Value >= Slice
	friend
	bool
	all_ge( T const & t, Array4S const & a )
	{
		return ge( t, a );
	}

public: // Comparison: Count

	// Count Slice == Slice
	friend
	size_type
	count_eq( Array4S const & a, Array4S const & b )
	{
		assert( a.conformable( b ) );
		if ( a.empty() ) return 0;
		if ( &a == &b ) return a.size_;
		size_type n( 0u );
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( a( i1, i2, i3, i4 ) == b( i1, i2, i3, i4 ) ) ++n;
					}
				}
			}
		}
		return n;
	}

	// Count Slice != Slice
	friend
	size_type
	count_ne( Array4S const & a, Array4S const & b )
	{
		assert( a.conformable( b ) );
		if ( a.empty() ) return 0;
		if ( &a == &b ) return 0;
		size_type n( 0u );
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( a( i1, i2, i3, i4 ) != b( i1, i2, i3, i4 ) ) ++n;
					}
				}
			}
		}
		return n;
	}

	// Count Slice < Slice
	friend
	size_type
	count_lt( Array4S const & a, Array4S const & b )
	{
		assert( a.conformable( b ) );
		if ( a.empty() ) return 0;
		if ( &a == &b ) return 0;
		size_type n( 0u );
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( a( i1, i2, i3, i4 ) < b( i1, i2, i3, i4 ) ) ++n;
					}
				}
			}
		}
		return n;
	}

	// Count Slice <= Slice
	friend
	size_type
	count_le( Array4S const & a, Array4S const & b )
	{
		assert( a.conformable( b ) );
		if ( a.empty() ) return 0;
		if ( &a == &b ) return a.size_;
		size_type n( 0u );
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( a( i1, i2, i3, i4 ) <= b( i1, i2, i3, i4 ) ) ++n;
					}
				}
			}
		}
		return n;
	}

	// Count Slice > Slice
	friend
	size_type
	count_gt( Array4S const & a, Array4S const & b )
	{
		assert( a.conformable( b ) );
		if ( a.empty() ) return 0;
		if ( &a == &b ) return 0;
		size_type n( 0u );
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( a( i1, i2, i3, i4 ) > b( i1, i2, i3, i4 ) ) ++n;
					}
				}
			}
		}
		return n;
	}

	// Count Slice >= Slice
	friend
	size_type
	count_ge( Array4S const & a, Array4S const & b )
	{
		assert( a.conformable( b ) );
		if ( a.empty() ) return 0;
		if ( &a == &b ) return a.size_;
		size_type n( 0u );
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( a( i1, i2, i3, i4 ) >= b( i1, i2, i3, i4 ) ) ++n;
					}
				}
			}
		}
		return n;
	}

	// Count Slice == Value
	friend
	size_type
	count_eq( Array4S const & a, T const & t )
	{
		if ( a.empty() ) return 0;
		size_type n( 0u );
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( a( i1, i2, i3, i4 ) == t ) ++n;
					}
				}
			}
		}
		return n;
	}

	// Count Value == Slice
	friend
	size_type
	count_eq( T const & t, Array4S const & a )
	{
		return count_eq( a, t );
	}

	// Count Slice != Value
	friend
	size_type
	count_ne( Array4S const & a, T const & t )
	{
		if ( a.empty() ) return 0;
		size_type n( 0u );
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( a( i1, i2, i3, i4 ) != t ) ++n;
					}
				}
			}
		}
		return n;
	}

	// Count Value != Slice
	friend
	size_type
	count_ne( T const & t, Array4S const & a )
	{
		return count_ne( a, t );
	}

	// Count Slice < Value
	friend
	size_type
	count_lt( Array4S const & a, T const & t )
	{
		if ( a.empty() ) return 0;
		size_type n( 0u );
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( a( i1, i2, i3, i4 ) < t ) ++n;
					}
				}
			}
		}
		return n;
	}

	// Count Value < Slice
	friend
	size_type
	count_lt( T const & t, Array4S const & a )
	{
		return count_gt( a, t );
	}

	// Count Slice <= Value
	friend
	size_type
	count_le( Array4S const & a, T const & t )
	{
		if ( a.empty() ) return 0;
		size_type n( 0u );
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( a( i1, i2, i3, i4 ) <= t ) ++n;
					}
				}
			}
		}
		return n;
	}

	// Count Value <= Slice
	friend
	size_type
	count_le( T const & t, Array4S const & a )
	{
		return count_ge( a, t );
	}

	// Count Slice > Value
	friend
	size_type
	count_gt( Array4S const & a, T const & t )
	{
		if ( a.empty() ) return 0;
		size_type n( 0u );
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( a( i1, i2, i3, i4 ) > t ) ++n;
					}
				}
			}
		}
		return n;
	}

	// Count Value > Slice
	friend
	size_type
	count_gt( T const & t, Array4S const & a )
	{
		return count_lt( a, t );
	}

	// Count Slice >= Value
	friend
	size_type
	count_ge( Array4S const & a, T const & t )
	{
		if ( a.empty() ) return 0;
		size_type n( 0u );
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						if ( a( i1, i2, i3, i4 ) >= t ) ++n;
					}
				}
			}
		}
		return n;
	}

	// Count Value >= Slice
	friend
	size_type
	count_ge( T const & t, Array4S const & a )
	{
		return count_le( a, t );
	}

public: // Comparison: Predicate: MArray

private: // Methods

	// Contiguous?
	bool
	computed_contiguous() const
	{
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunsequenced"
#endif
		std::int64_t u( u4_ );
		return ( m4_ == 1 ) && ( m3_ == u ) && ( m2_ == ( u *= u3_ ) ) && ( m1_ == ( u *= u2_ ) );
#ifdef __clang__
#pragma clang diagnostic pop
#endif
	}

	// Memory Range Set
	void
	data_set()
	{
		if ( size_ > 0u ) { // Non-empty slice
			data_beg_ = data_end_ = data_ + k_;
			data_beg_ += m1_ * ( m1_ >= 0 ? 1 : u1_ );
			data_end_ += m1_ * ( m1_ <= 0 ? 1 : u1_ );
			data_beg_ += m2_ * ( m2_ >= 0 ? 1 : u2_ );
			data_end_ += m2_ * ( m2_ <= 0 ? 1 : u2_ );
			data_beg_ += m3_ * ( m3_ >= 0 ? 1 : u3_ );
			data_end_ += m3_ * ( m3_ <= 0 ? 1 : u3_ );
			data_beg_ += m4_ * ( m4_ >= 0 ? 1 : u4_ );
			data_end_ += m4_ * ( m4_ <= 0 ? 1 : u4_ );
		} else {
			data_ = data_beg_ = data_end_ = nullptr;
		}
	}

private: // Data

	std::int64_t const m4_; // Multiplier of dim 4
	std::int64_t const m3_; // Multiplier of dim 3
	std::int64_t const m2_; // Multiplier of dim 2
	std::int64_t const m1_; // Multiplier of dim 1
	std::int64_t const k_; // Constant
	int const u1_; // Upper index of dim 1
	int const u2_; // Upper index of dim 2
	int const u3_; // Upper index of dim 3
	int const u4_; // Upper index of dim 4

}; // Array4S

// Conformable?
template< typename U, typename V >
inline
bool
conformable( Array4S< U > const & a, Array4S< V > const & b )
{
	return a.conformable( b );
}

// Stream >> Array4S
template< typename T >
inline
std::istream &
operator >>( std::istream & stream, Array4S< T > & a )
{
	if ( stream && ( a.size() > 0u ) ) {
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						stream >> a( i1, i2, i3, i4 );
						if ( ! stream ) break;
					} if ( ! stream ) break;
				} if ( ! stream ) break;
			} if ( ! stream ) break;
		}
	}
	return stream;
}

// Stream << Array4S
template< typename T >
inline
std::ostream &
operator <<( std::ostream & stream, Array4S< T > const & a )
{
	typedef  TypeTraits< T >  Traits;
	if ( stream && ( a.size() > 0u ) ) {
		std::ios_base::fmtflags const old_flags( stream.flags() );
		std::streamsize const old_precision( stream.precision( Traits::precision ) );
		stream << std::right << std::showpoint << std::uppercase;
		int const w( Traits::iwidth );
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						stream << std::setw( w ) << a( i1, i2, i3, i4 ) << ' ';
						if ( ! stream ) break;
					} if ( ! stream ) break;
				} if ( ! stream ) break;
			} if ( ! stream ) break;
		}
		stream.precision( old_precision );
		stream.flags( old_flags );
	}
	return stream;
}

// Read an Array4S from a Binary File
template< typename T >
inline
std::istream &
read_binary( std::istream & stream, Array4S< T > & a )
{
	std::size_t const n( a.size() );
	if ( stream && ( n > 0u ) ) {
		std::size_t const type_size( sizeof( T ) / sizeof( std::istream::char_type ) );
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						stream.read( ( std::istream::char_type * )&a( i1, i2, i3, i4 ), type_size );
						if ( ! stream ) break;
					} if ( ! stream ) break;
				} if ( ! stream ) break;
			} if ( ! stream ) break;
		}
	}
	return stream;
}


namespace fmt {

// List-Directed Format: Array4S
template< typename T >
inline
std::string
LD( Array4S< T > const & a )
{
	std::string s;
	std::size_t const n( a.size() );
	if ( n > 0u ) {
		s.reserve( n * TypeTraits< T >::width );
		for ( int i1 = 1, e1 = a.u1(); i1 <= e1; ++i1 ) {
			for ( int i2 = 1, e2 = a.u2(); i2 <= e2; ++i2 ) {
				for ( int i3 = 1, e3 = a.u3(); i3 <= e3; ++i3 ) {
					for ( int i4 = 1, e4 = a.u4(); i4 <= e4; ++i4 ) {
						s.append( fmt::LD( a( i1, i2, i3, i4 ) ) );
					}
				}
			}
		}
	}
	return s;
}

} // fmt

} // ObjexxFCL

#endif // ObjexxFCL_Array4S_hh_INCLUDED
