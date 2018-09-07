#ifndef __AP_MATH_H__
#define __AP_MATH_H__

#include "definitions.h"
#include <limits>
#include <math.h>
#include <stdint.h>

#include "rotation.h"
#include "vector2.h"
#include "vector3.h"
#include "vectorN.h"
#include "matrix3.h"
#include "quaternion.h"
#include "polygon.h"
#include "edc.h"
#include "location.h"



#define	M_E		    2.7182818284590452354	/* e */
#define	M_LOG2E		1.4426950408889634074	/* log 2e */
#define	M_LOG10E	0.43429448190325182765	/* log 10e */
#define	M_LN2		0.693147180559945309417	/* log e2 */
#define	M_LN10		2.30258509299404568402	/* log e10 */
#define	M_TWOPI		6.28318530717958647692	/* 2*pi */
#define	M_PI_4		0.78539816339744830962	/* pi/4 */
#define	M_3PI_4		2.3561944901923448370	/* 3/4 * pi */
#define	M_SQRTPI	1.77245385090551602792981 /* sqrt(pi) */
#define	M_1_PI		0.31830988618379067154	/* 1/pi */
#define	M_2_PI		0.63661977236758134308	/* 2/pi */
#define	M_2_SQRTPI	1.12837916709551257390	/* 2/sqrt(pi) */
#define	M_SQRT2		1.41421356237309504880	/* sqrt(2) */
#define	M_SQRT1_2	0.70710678118654752440	/* 1/sqrt(2) */
#define	M_LN2LO		1.9082149292705877000E-10 /* lower bits of log e2 */
#define	M_LN2HI		6.9314718036912381649E-1 /* log e2 */
#define	M_SQRT3   	1.73205080756887719000	/* sqrt(3) */
#define	M_IVLN10	0.43429448190325182765 /* 1 / log(10) */
#define	M_LOG2_E	0.693147180559945309417
#define	M_INVLN2	1.4426950408889633870E0  /* 1 / log e2 */

#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))

// are two floats equal
static inline bool is_equal(const float fVal1, const float fVal2) { return fabsf(fVal1 - fVal2) < FLT_EPSILON ? true : false; }

// is a float is zero
static inline bool is_zero(const float fVal1) { return fabsf(fVal1) < FLT_EPSILON ? true : false; }

// a varient of asin() that always gives a valid answer.
float           safe_asin(float v);

// a varient of sqrt() that always gives a valid answer.
float           safe_sqrt(float v);

// return determinant of square matrix
float                   detnxn(const float C[], const uint8_t n);

// Output inverted nxn matrix when returns true, otherwise matrix is Singular
bool                    inversenxn(const float x[], float y[], const uint8_t n);

// invOut is an inverted 4x4 matrix when returns true, otherwise matrix is Singular
bool                    inverse3x3(float m[], float invOut[]);

// invOut is an inverted 3x3 matrix when returns true, otherwise matrix is Singular
bool                    inverse4x4(float m[],float invOut[]);

// matrix multiplication of two NxN matrices
float* mat_mul(float *A, float *B, uint8_t n);

/*
  wrap an angle in centi-degrees
 */
int32_t wrap_360_cd(int32_t error);
int32_t wrap_180_cd(int32_t error);
float wrap_360_cd_float(float angle);
float wrap_180_cd_float(float angle);

/*
  wrap an angle defined in radians to -PI ~ PI (equivalent to +- 180 degrees)
 */
float wrap_PI(float angle_in_radians);

/*
  wrap an angle defined in radians to the interval [0,2*PI)
 */
float wrap_2PI(float angle);

// constrain a value
// constrain a value
static inline float constrain_float(float amt, float low, float high)
{
	// the check for NaN as a float prevents propogation of
	// floating point errors through any function that uses
	// constrain_float(). The normal float semantics already handle -Inf
	// and +Inf
	if (isnan(amt)) {
		return (low+high)*0.5f;
	}
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
// constrain a int16_t value
static inline int16_t constrain_int16(int16_t amt, int16_t low, int16_t high) {
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// constrain a int32_t value
static inline int32_t constrain_int32(int32_t amt, int32_t low, int32_t high) {
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

//matrix algebra
bool inverse(float x[], float y[], uint16_t dim);


// Matrix mul vector
float * mat_mul_vec(float x[], float v[], uint16_t m, uint16_t n);

// degrees -> radians
static inline float radians(float deg) {
	return deg * DEG_TO_RAD;
}

// radians -> degrees
static inline float degrees(float rad) {
	return rad * RAD_TO_DEG;
}

// square
static inline float sq(float v) {
	return v*v;
}

// 2D vector length
static inline float pythagorous2(float a, float b) {
	return sqrtf(sq(a)+sq(b));
}

// 3D vector length
static inline float pythagorous3(float a, float b, float c) {
	return sqrtf(sq(a)+sq(b)+sq(c));
}

//template<typename A, typename B>
//static inline auto MIN(const A &one, const B &two) -> decltype(one < two ? one : two) {
//    return one < two ? one : two;
//}
/* The following three functions used to be arduino core macros */

//template<typename A, typename B>
//static inline auto MAX(const A &one, const B &two) -> decltype(one > two ? one : two) {
//    return one > two ? one : two;
//}

inline uint32_t hz_to_nsec(uint32_t freq)
{
    return NSEC_PER_SEC / freq;
}

inline uint32_t nsec_to_hz(uint32_t nsec)
{
    return NSEC_PER_SEC / nsec;
}

inline uint32_t usec_to_nsec(uint32_t usec)
{
    return usec * NSEC_PER_USEC;
}

inline uint32_t nsec_to_usec(uint32_t nsec)
{
    return nsec / NSEC_PER_USEC;
}

inline uint32_t hz_to_usec(uint32_t freq)
{
    return USEC_PER_SEC / freq;
}

inline uint32_t usec_to_hz(uint32_t usec)
{
    return USEC_PER_SEC / usec;
}

#endif
