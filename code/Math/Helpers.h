//
#pragma once


#include <math.h>
#include "Vector.h"



template<typename T>
inline T Max(const T& x, const T& y)
{
	return x > y ? x : y;
}

template<>
inline Vec2 Max(const Vec2& x, const Vec2& y)
{
	return Vec2(Max(x.x, y.x), Max(x.y, y.y));
}

template<>
inline Vec3 Max(const Vec3& x, const Vec3& y)
{
	return Vec3(Max(x.x, y.x), Max(x.y, y.y), Max(x.z, y.z));
}

template<>
inline Vec4 Max(const Vec4& x, const Vec4& y)
{
	return Vec4(Max(x.x, y.x), Max(x.y, y.y), Max(x.z, y.z), Max(x.w, y.w));
}


template<typename T>
inline T Min(const T& x, const T& y)
{
	return x < y ? x : y;
}


template<>
inline Vec2 Min(const Vec2& x, const Vec2& y)
{
	return Vec2(Min(x.x, y.x), Min(x.y, y.y));
}

template<>
inline Vec3 Min(const Vec3& x, const Vec3& y)
{
	return Vec3(Min(x.x, y.x), Min(x.y, y.y), Min(x.z, y.z));
}

template<>
inline Vec4 Min(const Vec4& x, const Vec4& y)
{
	return Vec4(Min(x.x, y.x), Min(x.y, y.y), Min(x.z, y.z), Min(x.w, y.w));
}

template<typename T>
T Clamp(const T& x, const T& lower, const T& upper)
{
	return Max(Min(x, upper), lower);
}

template<typename T>
T Abs(const T& x)
{
	return x > 0 ? x : -x;
}

template<typename T>
float Sqrt(const T& x)
{
	assert(x >= 0);
	return sqrtf(static_cast<float>(x));
}


template<typename T>
T Sign(const T& x)
{
	return static_cast<float>(x >= 0 ? 1 : -1);
}


bool SphereRayIntersection(Vec3 sphereCenter, float radius, Vec3 rayOrigin, Vec3 rayDir, float& t0, float& t1);

bool AABBRayIntersection(Vec3 boundMin, Vec3 boundMax, Vec3 rayOrigin, Vec3 rayDir, float& t0, float& t1);